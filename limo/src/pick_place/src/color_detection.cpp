/**
 * @file dynamic_color_detector.cpp
 * @brief Color-based box detection with dynamic color ranges from reference images
 * 
 * Learns color ranges and aspect ratios from PNG reference images.
 * Uses confidence scoring to select best match per object template.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <string>
#include <map>
#include <algorithm>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

class DynamicColorDetector
{
public:
    DynamicColorDetector() : nh_("~"), it_(nh_)
    {
        loadParameters();
        
        if (!loadReferenceImages())
        {
            ROS_ERROR("Failed to load reference images. Exiting.");
            ros::shutdown();
            return;
        }
        
        // Initialize publishers
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detection/object", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detection/pose", 10);
        debug_image_pub_ = it_.advertise("/detection/debug_image", 10);
        mask_image_pub_ = it_.advertise("/detection/mask_image", 10);
        
        // Subscribe to camera info
        camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &DynamicColorDetector::cameraInfoCallback, this);
        
        // Setup synchronized subscribers
        rgb_sub_.subscribe(nh_, "/camera/color/image_raw", 10);
        depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 10);
        
        sync_.reset(new Sync(SyncPolicy(10), rgb_sub_, depth_sub_));
        sync_->registerCallback(boost::bind(&DynamicColorDetector::imageCallback, this, _1, _2));
        
        ROS_INFO("Dynamic Color Detector initialized with confidence scoring");
        ROS_INFO("Loaded %zu object templates", object_templates_.size());
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
    ros::Publisher marker_pub_;
    ros::Publisher pose_pub_;
    image_transport::Publisher debug_image_pub_;
    image_transport::Publisher mask_image_pub_;
    ros::Subscriber camera_info_sub_;
    
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    
    // Camera parameters
    double fx_, fy_, cx_, cy_;
    bool camera_info_received_;
    std::string target_frame_;
    
    // Detection parameters
    int min_contour_area_;
    int max_contour_area_;
    double aspect_ratio_tolerance_;
    bool show_debug_image_;
    int blur_kernel_size_;
    int morph_kernel_size_;
    int ellipse_kernel_size_;
    double min_box_size_;
    double max_box_size_;
    double min_depth_;
    double max_depth_;
    double fallback_depth_;
    bool use_fallback_depth_;
    
    // Scoring weights
    double color_match_weight_;
    double aspect_ratio_weight_;
    double area_weight_;
    double depth_quality_weight_;
    double score_threshold_;
    
    std::string objects_path_;
    int hsv_tolerance_;
    int value_min_;
    
    // Object template
    struct ObjectTemplate {
        std::string name;
        cv::Scalar hsv_lower;
        cv::Scalar hsv_upper;
        cv::Scalar viz_color;
        int adaptive_sat_min;
        double area;
        double aspect_ratio;
        double aspect_ratio_min;
        double aspect_ratio_max;
        double expected_hue;
        cv::Mat reference_image;
    };
    
    std::vector<ObjectTemplate> object_templates_;
    
    // Detection candidate with confidence score
    struct DetectionCandidate {
        int template_idx;
        cv::Rect bbox;
        cv::Point2f center;
        double aspect_ratio;
        float depth;
        double confidence;
        
        // Individual score components
        double color_score;
        double aspect_ratio_score;
        double area_score;
        double depth_quality_score;
        
        // 3D information
        double x, y, z;
        double width, height, box_depth;
    };

    void loadParameters()
    {
        std::string node_name = ros::this_node::getName();
        
        target_frame_ = nh_.param(node_name + "/target_frame_id", std::string("camera_depth_optical_frame"));
        min_contour_area_ = nh_.param(node_name + "/min_contour_area", 100);
        max_contour_area_ = nh_.param(node_name + "/max_contour_area", 50000);
        aspect_ratio_tolerance_ = nh_.param(node_name + "/aspect_ratio_tolerance", 0.3);
        show_debug_image_ = nh_.param(node_name + "/show_debug_image", true);
        min_box_size_ = nh_.param(node_name + "/min_box_size", 0.01);
        max_box_size_ = nh_.param(node_name + "/max_box_size", 1.0);
        blur_kernel_size_ = nh_.param(node_name + "/blur_kernel_size", 5);
        morph_kernel_size_ = nh_.param(node_name + "/morph_kernel_size", 3);
        ellipse_kernel_size_ = nh_.param(node_name + "/ellipse_kernel_size", 3);
        min_depth_ = nh_.param(node_name + "/min_depth", 0.3);
        max_depth_ = nh_.param(node_name + "/max_depth", 3.0);
        fallback_depth_ = nh_.param(node_name + "/fallback_depth", 0.5);
        use_fallback_depth_ = nh_.param(node_name + "/use_fallback_depth", true);
        
        objects_path_ = nh_.param(node_name + "/objects_path", std::string(""));
        hsv_tolerance_ = nh_.param(node_name + "/hsv_tolerance", 15);
        value_min_ = nh_.param(node_name + "/value_min", 40);
        
        // Scoring weights (sum should be 1.0)
        color_match_weight_ = nh_.param(node_name + "/color_match_weight", 0.35);
        aspect_ratio_weight_ = nh_.param(node_name + "/aspect_ratio_weight", 0.30);
        area_weight_ = nh_.param(node_name + "/area_weight", 0.20);
        depth_quality_weight_ = nh_.param(node_name + "/depth_quality_weight", 0.15);
        score_threshold_ = nh_.param(node_name + "/score_threshold", 0.5);
        
        fx_ = fy_ = 554.0;
        cx_ = cy_ = 320.0;
        camera_info_received_ = false;
    }

    bool loadReferenceImages()
    {
        if (objects_path_.empty())
        {
            ROS_ERROR("objects_path parameter is not set!");
            return false;
        }
        
        std::string expanded_path = objects_path_;
        if (expanded_path.find("$(find") != std::string::npos)
        {
            size_t start = expanded_path.find("$(find ") + 7;
            size_t end = expanded_path.find(")", start);
            std::string pkg_name = expanded_path.substr(start, end - start);
            std::string pkg_path = ros::package::getPath(pkg_name);
            expanded_path = pkg_path + expanded_path.substr(end + 1);
        }
        
        fs::path dir_path(expanded_path);
        
        if (!fs::exists(dir_path) || !fs::is_directory(dir_path))
        {
            ROS_ERROR("Objects path does not exist: %s", expanded_path.c_str());
            return false;
        }
        
        ROS_INFO("Loading reference images from: %s", expanded_path.c_str());
        
        std::vector<fs::path> png_files;
        for (fs::directory_iterator it(dir_path); it != fs::directory_iterator(); ++it)
        {
            if (fs::is_regular_file(it->path()) && 
                (it->path().extension() == ".png" || it->path().extension() == ".PNG"))
            {
                png_files.push_back(it->path());
            }
        }
        
        std::sort(png_files.begin(), png_files.end());
        
        if (png_files.empty())
        {
            ROS_ERROR("No PNG files found in: %s", expanded_path.c_str());
            return false;
        }
        
        ROS_INFO("Found %zu PNG reference images", png_files.size());
        
        for (const auto& png_path : png_files)
        {
            ObjectTemplate obj_template;
            if (processReferenceImage(png_path.string(), obj_template))
            {
                object_templates_.push_back(obj_template);
                ROS_INFO("  [%zu] %s: H=%.0f±%d, S>%d, V>%d, A=%.2f, AR=%.2f±%.2f",
                         object_templates_.size(),
                         obj_template.name.c_str(),
                         obj_template.expected_hue,
                         hsv_tolerance_,
                         obj_template.adaptive_sat_min,
                         value_min_,
                         obj_template.area,
                         obj_template.aspect_ratio,
                         aspect_ratio_tolerance_);
            }
        }
        
        return !object_templates_.empty();
    }

    bool processReferenceImage(const std::string& image_path, ObjectTemplate& obj_template)
    {
        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        if (img.empty())
        {
            ROS_WARN("Failed to load image: %s", image_path.c_str());
            return false;
        }
        
        fs::path p(image_path);
        obj_template.name = p.stem().string();
        obj_template.reference_image = img.clone();
        obj_template.area = img.cols * img.rows;
        obj_template.aspect_ratio = static_cast<double>(img.cols) / img.rows;
        obj_template.aspect_ratio_min = obj_template.aspect_ratio * (1.0 - aspect_ratio_tolerance_);
        obj_template.aspect_ratio_max = obj_template.aspect_ratio * (1.0 + aspect_ratio_tolerance_);
        
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        
        std::vector<cv::Mat> hsv_channels;
        cv::split(hsv, hsv_channels);
        
        cv::Mat loose_mask = (hsv_channels[2] > value_min_);
        std::vector<uchar> sat_vals;
        for (int r = 0; r < hsv.rows; ++r)
        {
            const uchar* s = hsv_channels[1].ptr<uchar>(r);
            const uchar* m = loose_mask.ptr<uchar>(r);
            for (int c = 0; c < hsv.cols; ++c)
            {
                if (m[c])
                    sat_vals.push_back(s[c]);
            }
        }
        std::nth_element(sat_vals.begin(), sat_vals.begin() + sat_vals.size() / 3, sat_vals.end());
        double sat_ref = sat_vals[sat_vals.size() / 3];
        int adaptive_sat_min = static_cast<int>(sat_ref * 0.8);
        adaptive_sat_min = std::max(20, adaptive_sat_min);
        adaptive_sat_min = std::min(200, adaptive_sat_min);
        obj_template.adaptive_sat_min = adaptive_sat_min;

        cv::Mat mask = (hsv_channels[1] >= adaptive_sat_min) & (hsv_channels[2] >= value_min_);
        
        int hist_size = 180;
        float h_ranges[] = {0, 180};
        const float* ranges[] = {h_ranges};
        cv::Mat h_hist;
        cv::calcHist(&hsv_channels[0], 1, 0, mask, h_hist, 1, &hist_size, ranges, true, false);
        
        double max_val;
        cv::Point max_loc;
        cv::minMaxLoc(h_hist, nullptr, &max_val, nullptr, &max_loc);
        
        if (max_val < 10)
        {
            ROS_WARN("Image %s has insufficient color information", obj_template.name.c_str());
            return false;
        }
        
        int dominant_hue = max_loc.y;
        obj_template.expected_hue = dominant_hue;
        
        int h_lower = std::max(0, dominant_hue - hsv_tolerance_);
        int h_upper = std::min(179, dominant_hue + hsv_tolerance_);
        
        obj_template.hsv_lower = cv::Scalar(h_lower, adaptive_sat_min, value_min_);
        obj_template.hsv_upper = cv::Scalar(h_upper, 255, 255);
        
        cv::Mat dominant_color_hsv(1, 1, CV_8UC3, cv::Scalar(dominant_hue, 200, 200));
        cv::Mat dominant_color_bgr;
        cv::cvtColor(dominant_color_hsv, dominant_color_bgr, cv::COLOR_HSV2BGR);
        obj_template.viz_color = cv::Scalar(
            dominant_color_bgr.at<cv::Vec3b>(0, 0)[0],
            dominant_color_bgr.at<cv::Vec3b>(0, 0)[1],
            dominant_color_bgr.at<cv::Vec3b>(0, 0)[2]
        );
        
        return true;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        if (!camera_info_received_)
        {
            fx_ = msg->K[0];
            fy_ = msg->K[4];
            cx_ = msg->K[2];
            cy_ = msg->K[5];
            camera_info_received_ = true;
            ROS_INFO("Camera parameters updated: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", fx_, fy_, cx_, cy_);
        }
    }

    // Helper: circular hue distance (h values in 0..179)
    int hueDistance(int h1, int h2) {
        int d = abs(h1 - h2);
        return std::min(d, 180 - d);
    }

    // Helper: check if detected bbox is near image border
    bool touchesImageBorder(const cv::Rect& roi, int img_w, int img_h, int margin = 2)
    {
        return (roi.x <= margin || roi.y <= margin || roi.x + roi.width  >= img_w - margin || roi.y + roi.height >= img_h - margin);
    }

    // Create a robust color mask for a template
    cv::Mat createRobustColorMask(const cv::Mat& hsv, const cv::Scalar& hsv_center, int hue_tol,
                                int sat_min, int val_min, int blur_ksize, int morph_ksize)
    {
        // hsv is CV_8UC3 (H:0-179, S:0-255, V:0-255)
        CV_Assert(hsv.type() == CV_8UC3);

        // Split channels
        std::vector<cv::Mat> ch;
        cv::split(hsv, ch);
        cv::Mat H = ch[0], S = ch[1], V = ch[2];

        // Preprocess V: CLAHE to reduce illumination variation
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8,8));
        cv::Mat V_clahe;
        clahe->apply(V, V_clahe);

        // Merge quick processed HSV for blur
        std::vector<cv::Mat> merged = { H, S, V_clahe };
        cv::Mat hsv_proc;
        cv::merge(merged, hsv_proc);
        if (blur_ksize > 1) {
            cv::GaussianBlur(hsv_proc, hsv_proc, cv::Size(blur_ksize, blur_ksize), 0);
            cv::split(hsv_proc, merged);
            H = merged[0]; S = merged[1]; V_clahe = merged[2];
        }

        // Per-pixel test using hue circular distance and sat/val thresholds
        cv::Mat mask = cv::Mat::zeros(H.size(), CV_8UC1);
        int target_h = static_cast<int>(hsv_center[0] + 0.5);
        for (int r = 0; r < H.rows; ++r) {
            const uchar* Hp = H.ptr<uchar>(r);
            const uchar* Sp = S.ptr<uchar>(r);
            const uchar* Vp = V_clahe.ptr<uchar>(r);
            uchar* Mp = mask.ptr<uchar>(r);
            for (int c = 0; c < H.cols; ++c) {
                int hval = Hp[c];
                if (Sp[c] < sat_min || Vp[c] < val_min) {
                    Mp[c] = 0;
                    continue;
                }
                if (hueDistance(hval, target_h) <= hue_tol) {
                    Mp[c] = 255;
                } else {
                    Mp[c] = 0;
                }
            }
        }

        // Morphology to clean up
        if (morph_ksize > 0) {
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_ksize, morph_ksize));
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        }

        return mask;
    }

    // Edge detection for contours
    cv::Mat enhanceMaskBoundaries(const cv::Mat& mask)
    {
        // Strategy: Use dilation difference to sharpen edges
        // dilated - original = edge region
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ellipse_kernel_size_, ellipse_kernel_size_));
        cv::Mat dilated;
        cv::dilate(mask, dilated, kernel, cv::Point(-1, -1), 1);
        // Edge pixels = where dilation adds new white
        cv::Mat edge_region = dilated - mask;
        // Strengthen boundaries by combining edge region with original
        cv::Mat enhanced = mask + (0.25 * edge_region);  // Add back 25% of edge region
        enhanced = cv::min(enhanced, 255);  // Clip to [0, 255]
        return enhanced;
    }

    // Color scoring: percent of pixels inside mask + histogram backprojection refinement
    double computeColorScore(const cv::Mat& hsv_roi, const cv::Mat& mask, const cv::Scalar& hsv_center)
    {
        // 1) fraction of good pixels
        double total = hsv_roi.rows * hsv_roi.cols;
        double good = static_cast<double>(cv::countNonZero(mask));
        double frac = (total > 0.0) ? (good / total) : 0.0;

        // 2) histogram backprojection (H channel only) for finer matching
        std::vector<cv::Mat> ch;
        cv::split(hsv_roi, ch);
        cv::Mat H = ch[0];
        int h_center = static_cast<int>(hsv_center[0] + 0.5);

        // Build small 1D histogram around target hue
        int bins = 16;
        cv::Mat hist;
        int ranges[] = {0,180};
        int histSize[] = {bins};
        const int* histSizes = histSize;
        const float hrange[] = {0.f, 180.f};
        const float* rangesPtr[] = { hrange };
        cv::calcHist(&H, 1, 0, cv::Mat(), hist, 1, histSize, rangesPtr, true, false);

        // We expect more mass near the target hue. Compute normalized backprojection score:
        cv::Mat backproj;
        cv::calcBackProject(&H, 1, 0, hist, backproj, rangesPtr);
        // backproj values scaled 0..255, compute mean in masked area
        cv::Scalar mean_bp = cv::mean(backproj, mask);
        double bp_score = (mean_bp[0] / 255.0); // 0..1

        // Combine: more weight to fraction for robustness, but BP refines intensity
        double score = 0.7 * frac + 0.3 * bp_score;
        return score;
    }

    // AR soft score
    double aspectRatioScore(double ar, double ar_ref, double sigma = 0.35)
    {
        double err = std::abs(ar - ar_ref) / ar_ref;
        return std::exp(-(err * err) / (2.0 * sigma * sigma));
    }

    // Area score
    double areaScore(double median_depth, double area, double ref_area)
    {
        double expected_area = ref_area *std::pow(min_depth_ / median_depth, 2); // Hard estimation, ref image is taken at min depth
        double area_ratio = area / expected_area;
        return std::exp(-std::pow(std::log(area_ratio), 2) / (2 * 0.5 * 0.5));
    }

    // Depth quality: use median depth and low variance
    double depthQualityFromMask(const cv::Mat& depth_roi)
    {
        std::vector<float> depths;
        // Filter valid depth values
        for(int r = 0; r < depth_roi.rows; ++r){
            const uint16_t* dp = depth_roi.ptr<uint16_t>(r);
            for(int c = 0; c < depth_roi.cols; ++c){
                uint16_t d_mm = dp[c];
                if(d_mm > min_depth_*1000 && d_mm < max_depth_*1000) {
                    float d_m = d_mm / 1000.0f;  // Convert to meters
                    depths.push_back(d_m);
                }
            }
        }
        if(depths.empty()) return 0.0;
        std::sort(depths.begin(), depths.end());
        double median = depths[depths.size() / 2];
        // Trim outliers (top 5%)
        size_t outlier_idx = static_cast<size_t>(depths.size() * 0.95);
        std::vector<float> trimmed(depths.begin(), depths.begin() + outlier_idx);
        if(trimmed.empty()) return 0.0;
        double mean = 0;
        for(double v : trimmed) mean += v;
        mean /= trimmed.size();
        double var = 0;
        for(double v : trimmed) var += (v - mean) * (v - mean);
        var /= trimmed.size();
        double sd = std::sqrt(var);
        double q = std::max(0.0, 1.0 - (sd / std::max(0.01, median * 0.5))); // quality matrix
        return std::min(1.0, std::max(0.0, q));
    }

    double calculateConfidence(const DetectionCandidate& candidate, const ObjectTemplate& obj_template,
                            const cv::Mat& hsv_roi, const cv::Mat& mask_roi, const cv::Mat& depth_roi)
    {
        // 1. Color matching score (HSV histogram comparison)
        double color_score = computeColorScore(hsv_roi, mask_roi, obj_template.hsv_lower);
        // 2. Aspect ratio matching score
        double aspect_ratio_score = aspectRatioScore(candidate.aspect_ratio, obj_template.aspect_ratio, aspect_ratio_tolerance_);
        // 3. Area confidence (prefer medium-sized detections)
        double area_score = areaScore(candidate.depth, candidate.bbox.area(), obj_template.area);
        // 4. Depth quality score
        double depth_quality_score = depthQualityFromMask(depth_roi);
        // Weighted combination
        double total_confidence = 
            color_score * color_match_weight_ +
            aspect_ratio_score * aspect_ratio_weight_ +
            area_score * area_weight_ +
            depth_quality_score * depth_quality_weight_;
        return total_confidence;
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& rgb_msg, const sensor_msgs::Image::ConstPtr& depth_msg)
    {
        cv_bridge::CvImagePtr cv_rgb, cv_depth;
        try
        {
            cv_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            
            if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                depth_msg->encoding == "16UC1" || depth_msg->encoding == "mono16")
            {
                cv_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                     depth_msg->encoding == "32FC1")
            {
                cv_bridge::CvImagePtr cv_depth_float = cv_bridge::toCvCopy(depth_msg, 
                                                        sensor_msgs::image_encodings::TYPE_32FC1);
                cv_depth = boost::make_shared<cv_bridge::CvImage>();
                cv_depth->header = depth_msg->header;
                cv_depth->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                cv_depth->image = cv::Mat(cv_depth_float->image.size(), CV_16UC1);
                
                for (int i = 0; i < cv_depth_float->image.rows; ++i)
                {
                    for (int j = 0; j < cv_depth_float->image.cols; ++j)
                    {
                        float depth_m = cv_depth_float->image.at<float>(i, j);
                        if (std::isfinite(depth_m) && depth_m > 0)
                            cv_depth->image.at<uint16_t>(i, j) = static_cast<uint16_t>(depth_m * 1000.0f);
                        else
                            cv_depth->image.at<uint16_t>(i, j) = 0;
                    }
                }
            }
            else
            {
                ROS_ERROR_THROTTLE(5.0, "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
                return;
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat blurred;
        if (blur_kernel_size_ > 0)
            cv::GaussianBlur(cv_rgb->image, blurred, cv::Size(blur_kernel_size_, blur_kernel_size_), 0);
        else
            blurred = cv_rgb->image.clone();

        cv::Mat hsv_image;
        cv::cvtColor(blurred, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat debug_image = cv_rgb->image.clone();
        cv::Mat combined_mask = cv::Mat::zeros(hsv_image.size(), CV_8UC1);
        
        // Store all candidates for each template
        std::map<int, std::vector<DetectionCandidate>> candidates_per_template;
        
        // Detect candidates for each object template
        for (size_t template_idx = 0; template_idx < object_templates_.size(); ++template_idx)
        {
            const auto& obj_template = object_templates_[template_idx];
            
            cv::Mat mask = createRobustColorMask(
                hsv_image,
                (obj_template.hsv_lower + obj_template.hsv_upper) * 0.5, // use center as scalar
                hsv_tolerance_,
                obj_template.adaptive_sat_min,
                value_min_,
                blur_kernel_size_,
                morph_kernel_size_);
            
            if (morph_kernel_size_ > 0)
            {
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_size_, morph_kernel_size_));
                cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
                cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            }
            cv::bitwise_or(combined_mask, mask, combined_mask);
            
            std::vector<std::vector<cv::Point>> contours;
            cv::Mat enhanced_mask = enhanceMaskBoundaries(mask);
            cv::findContours(enhanced_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            for (const auto& contour : contours)
            {
                double area = cv::contourArea(contour);
                
                if (area >= min_contour_area_ && area <= max_contour_area_)
                {
                    cv::Rect bbox = cv::boundingRect(contour);
                    bbox &= cv::Rect(0, 0, debug_image.cols, debug_image.rows);
                    if (touchesImageBorder(bbox, debug_image.cols, debug_image.rows))
                        continue; // Hard rejection if bounding box is near image edges
                    double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
                    // Create a detection candidate
                    DetectionCandidate candidate;
                    candidate.template_idx = template_idx;
                    candidate.bbox = bbox;
                    candidate.center = cv::Point2f(bbox.x + bbox.width/2.0f, bbox.y + bbox.height/2.0f);
                    candidate.aspect_ratio = aspect_ratio;
                    candidate.depth = getMedianDepth(cv_depth->image, bbox);
                    // Handle invalid depth
                    if (candidate.depth < min_depth_ || candidate.depth > max_depth_)
                    {
                        if (use_fallback_depth_)
                            candidate.depth = fallback_depth_;
                        else
                            continue;
                    }
                    // Calculate 3D position
                    candidate.x = (candidate.center.x - cx_) * candidate.depth / fx_;
                    candidate.y = (candidate.center.y - cy_) * candidate.depth / fy_;
                    candidate.z = candidate.depth;
                    // Estimate dimensions
                    candidate.width = (bbox.width * candidate.depth) / fx_;
                    candidate.height = (bbox.height * candidate.depth) / fy_;
                    candidate.box_depth = std::min(candidate.width, candidate.height) * 0.8;
                    candidate.width = std::max(min_box_size_, std::min(max_box_size_, candidate.width));
                    candidate.height = std::max(min_box_size_, std::min(max_box_size_, candidate.height));
                    candidate.box_depth = std::max(min_box_size_, std::min(max_box_size_, candidate.box_depth));
                    // Extract HSV ROI, mask ROI and depth ROI for scoring
                    cv::Rect safe_bbox_hsv = bbox & cv::Rect(0, 0, hsv_image.cols, hsv_image.rows);
                    cv::Mat hsv_roi = hsv_image(safe_bbox_hsv);
                    cv::Rect safe_bbox_mask = bbox & cv::Rect(0, 0, enhanced_mask.cols, enhanced_mask.rows);
                    cv::Mat mask_roi = enhanced_mask(safe_bbox_mask);
                    cv::Rect safe_bbox_depth = bbox & cv::Rect(0, 0, cv_depth->image.cols, cv_depth->image.rows);
                    cv::Mat depth_roi = cv_depth->image(safe_bbox_depth);
                    // Calculate confidence score
                    candidate.confidence = calculateConfidence(candidate, obj_template, hsv_roi, mask_roi, depth_roi);
                    // Add to candidate list
                    candidates_per_template[template_idx].push_back(candidate);
                }
            }
        }
        
        // Select best candidate for each template
        std::vector<DetectionCandidate> best_detections;
        
        for (const auto& pair : candidates_per_template)
        {
            int template_idx = pair.first;
            const auto& candidates = pair.second;
            
            if (candidates.empty())
                continue;
            
            // Find candidate with highest confidence
            auto best_it = std::max_element(candidates.begin(), candidates.end(),
                [](const DetectionCandidate& a, const DetectionCandidate& b) {
                    return a.confidence < b.confidence;
                });
            
            ROS_INFO("Template '%s': %zu candidates, best confidence=%.3f",
                     object_templates_[template_idx].name.c_str(),
                     candidates.size(),
                     best_it->confidence);
            
            if (best_it->confidence > score_threshold_) // accept only detection with confidence higher than threshold
                best_detections.push_back(*best_it);
        }
        
        // Publish results
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;
        
        for (const auto& detection : best_detections)
        {
            const auto& obj_template = object_templates_[detection.template_idx];
            
            ROS_INFO("BEST MATCH '%s': pos=[%.3f, %.3f, %.3f]m, size=[%.3f x %.3f x %.3f]m, "
                     "AR=%.2f, confidence=%.3f",
                     obj_template.name.c_str(),
                     detection.x, detection.y, detection.z,
                     detection.width, detection.box_depth, detection.height,
                     detection.aspect_ratio, detection.confidence);
            
            // Create markers
            marker_array.markers.push_back(createBoxMarker(
                marker_id++, rgb_msg->header, 
                detection.x, detection.y, detection.z, 
                detection.width, detection.box_depth, detection.height, 
                obj_template.viz_color));
            
            marker_array.markers.push_back(createTextMarker(
                marker_id++, rgb_msg->header, 
                detection.x, detection.y, detection.z + detection.height/2 + 0.05, 
                obj_template.name, detection.confidence));
            
            // Draw on debug image
            cv::rectangle(debug_image, detection.bbox, obj_template.viz_color, 3);
            
            std::stringstream label;
            label << obj_template.name << " (" << std::fixed << std::setprecision(2) 
                  << (detection.confidence * 100) << "%)";
            cv::putText(debug_image, label.str(), 
                       cv::Point(detection.bbox.x, detection.bbox.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, obj_template.viz_color, 2);
            
            cv::circle(debug_image, detection.center, 5, obj_template.viz_color, -1);
            
            std::stringstream info;
            info << std::fixed << std::setprecision(2) << detection.z << "m";
            cv::putText(debug_image, info.str(), 
                       cv::Point(detection.bbox.x, detection.bbox.y + detection.bbox.height + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, obj_template.viz_color, 2);
        }

        ROS_INFO_THROTTLE(1.0, "Frame: %zu unique objects detected", best_detections.size());

        if (!marker_array.markers.empty())
            marker_pub_.publish(marker_array);

        if (show_debug_image_ && debug_image_pub_.getNumSubscribers() > 0)
        {
            sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(
                rgb_msg->header, "bgr8", debug_image).toImageMsg();
            debug_image_pub_.publish(debug_msg);
        }

        if (mask_image_pub_.getNumSubscribers() > 0)
        {
            sensor_msgs::ImagePtr mask_msg = cv_bridge::CvImage(
                rgb_msg->header, "mono8", combined_mask).toImageMsg();
            mask_image_pub_.publish(mask_msg);
        }
    }

    float getMedianDepth(const cv::Mat& depth_image, const cv::Rect& roi)
    {
        cv::Rect safe_roi = roi & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
        if (safe_roi.width <= 0 || safe_roi.height <= 0)
            return 0.0f;
        
        cv::Mat roi_depth = depth_image(safe_roi);
        std::vector<float> valid_depths;
        const uint16_t MIN_DEPTH_MM = 100;
        
        for (int i = 0; i < roi_depth.rows; ++i)
        {
            for (int j = 0; j < roi_depth.cols; ++j)
            {
                uint16_t depth_mm = roi_depth.at<uint16_t>(i, j);
                if (depth_mm > MIN_DEPTH_MM)
                    valid_depths.push_back(depth_mm / 1000.0f);
            }
        }
        
        if (valid_depths.empty())
        {
            int expand = std::max(10, std::max(roi.width, roi.height) / 2);
            cv::Rect expanded(
                std::max(0, roi.x - expand),
                std::max(0, roi.y - expand),
                std::min(depth_image.cols - (roi.x - expand), roi.width + 2*expand),
                std::min(depth_image.rows - (roi.y - expand), roi.height + 2*expand)
            );
            expanded = expanded & cv::Rect(0, 0, depth_image.cols, depth_image.rows);
            
            cv::Mat exp_depth = depth_image(expanded);
            for (int i = 0; i < exp_depth.rows; ++i)
                for (int j = 0; j < exp_depth.cols; ++j)
                {
                    uint16_t depth_mm = exp_depth.at<uint16_t>(i, j);
                    if (depth_mm > MIN_DEPTH_MM)
                        valid_depths.push_back(depth_mm / 1000.0f);
                }
            
            if (valid_depths.empty())
                return 0.0f;
        }
        
        std::sort(valid_depths.begin(), valid_depths.end());
        return valid_depths[valid_depths.size() / 2];
    }

    visualization_msgs::Marker createBoxMarker(int id, const std_msgs::Header& header,
                                               double x, double y, double z,
                                               double width, double depth, double height,
                                               const cv::Scalar& color)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.header.frame_id = target_frame_;
        marker.ns = "detected_boxes";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = width;
        marker.scale.y = depth;
        marker.scale.z = height;
        
        marker.color.r = color[2] / 255.0;
        marker.color.g = color[1] / 255.0;
        marker.color.b = color[0] / 255.0;
        marker.color.a = 0.5;
        
        marker.lifetime = ros::Duration(0.5);
        return marker;
    }

    visualization_msgs::Marker createTextMarker(int id, const std_msgs::Header& header, double x, double y, double z, const std::string& text, double confidence)
    {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.header.frame_id = target_frame_;
        marker.ns = "labels";
        marker.id = id;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        
        std::stringstream ss;
        ss << text << "\n" << std::fixed << std::setprecision(0) << (confidence * 100) << "%";
        marker.text = ss.str();
        marker.scale.z = 0.05;
        
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.lifetime = ros::Duration(0.5);
        return marker;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_detection");
    
    try
    {
        DynamicColorDetector detector;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}