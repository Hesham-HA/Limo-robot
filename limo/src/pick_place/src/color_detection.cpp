/**
 * @file dynamic_color_detector.cpp
 * @brief Color-based box detection with dynamic color ranges from reference images
 * 
 * Learns color ranges and aspect ratios from PNG reference images
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
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

class DynamicColorDetector
{
public:
    DynamicColorDetector() : nh_("~"), it_(nh_)
    {
        // Load parameters
        loadParameters();
        
        // Load reference images and extract color ranges
        if (!loadReferenceImages())
        {
            ROS_ERROR("Failed to load reference images. Exiting.");
            ros::shutdown();
            return;
        }
        
        // Initialize publishers
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/detection/object", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/detection/pose", 10);
        debug_image_pub_ = it_.advertise("/detection/debug_image", 1);
        mask_image_pub_ = it_.advertise("/detection/mask_image", 1);
        
        // Subscribe to camera info
        camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, 
                                         &DynamicColorDetector::cameraInfoCallback, this);
        
        // Setup synchronized subscribers for RGB and Depth
        rgb_sub_.subscribe(nh_, "/camera/color/image_raw", 1);
        depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 1);
        
        // Approximate time synchronization policy
        sync_.reset(new Sync(SyncPolicy(10), rgb_sub_, depth_sub_));
        sync_->registerCallback(boost::bind(&DynamicColorDetector::imageCallback, this, _1, _2));
        
        ROS_INFO("Dynamic Color Detector initialized");
        ROS_INFO("Loaded %zu object templates", object_templates_.size());
    }

private:
    // ROS node handles
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
    // Publishers
    ros::Publisher marker_pub_;
    ros::Publisher pose_pub_;
    image_transport::Publisher debug_image_pub_;
    image_transport::Publisher mask_image_pub_;
    
    // Subscribers
    ros::Subscriber camera_info_sub_;
    
    // Message filters
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
    
    // Box size estimation
    double min_box_size_;
    double max_box_size_;
    double fallback_depth_;
    bool use_fallback_depth_;
    
    // Reference images path
    std::string objects_path_;
    int hsv_tolerance_;
    int saturation_min_;
    int value_min_;
    
    // Object template structure
    struct ObjectTemplate {
        std::string name;
        cv::Scalar hsv_lower;
        cv::Scalar hsv_upper;
        cv::Scalar viz_color;  // BGR for visualization
        double aspect_ratio;
        double aspect_ratio_min;
        double aspect_ratio_max;
        cv::Mat reference_image;
    };
    
    std::vector<ObjectTemplate> object_templates_;

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
        fallback_depth_ = nh_.param(node_name + "/fallback_depth", 0.5);
        use_fallback_depth_ = nh_.param(node_name + "/use_fallback_depth", true);
        
        // Reference images parameters
        objects_path_ = nh_.param(node_name + "/objects_path", std::string(""));
        hsv_tolerance_ = nh_.param(node_name + "/hsv_tolerance", 15);
        saturation_min_ = nh_.param(node_name + "/saturation_min", 40);
        value_min_ = nh_.param(node_name + "/value_min", 40);
        
        // Initialize camera parameters
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
        
        // Expand path if it contains package reference
        std::string expanded_path = objects_path_;
        if (expanded_path.find("$(find") != std::string::npos)
        {
            // Simple package path expansion
            size_t start = expanded_path.find("$(find ") + 7;
            size_t end = expanded_path.find(")", start);
            std::string pkg_name = expanded_path.substr(start, end - start);
            std::string pkg_path = ros::package::getPath(pkg_name);
            expanded_path = pkg_path + expanded_path.substr(end + 1);
        }
        
        fs::path dir_path(expanded_path);
        
        if (!fs::exists(dir_path) || !fs::is_directory(dir_path))
        {
            ROS_ERROR("Objects path does not exist or is not a directory: %s", expanded_path.c_str());
            return false;
        }
        
        ROS_INFO("Loading reference images from: %s", expanded_path.c_str());
        
        // Iterate through PNG files in directory
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
        
        // Process each image
        for (const auto& png_path : png_files)
        {
            ObjectTemplate obj_template;
            if (processReferenceImage(png_path.string(), obj_template))
            {
                object_templates_.push_back(obj_template);
                ROS_INFO("  [%zu] %s: H=%.0f±%d, S>%d, V>%d, AR=%.2f±%.2f",
                         object_templates_.size(),
                         obj_template.name.c_str(),
                         (obj_template.hsv_lower[0] + obj_template.hsv_upper[0]) / 2.0,
                         hsv_tolerance_,
                         saturation_min_,
                         value_min_,
                         obj_template.aspect_ratio,
                         aspect_ratio_tolerance_);
            }
        }
        
        return !object_templates_.empty();
    }

    bool processReferenceImage(const std::string& image_path, ObjectTemplate& obj_template)
    {
        // Load image
        cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
        if (img.empty())
        {
            ROS_WARN("Failed to load image: %s", image_path.c_str());
            return false;
        }
        
        // Extract object name from filename (without extension)
        fs::path p(image_path);
        obj_template.name = p.stem().string();
        obj_template.reference_image = img.clone();
        
        // Calculate aspect ratio
        obj_template.aspect_ratio = static_cast<double>(img.cols) / img.rows;
        obj_template.aspect_ratio_min = obj_template.aspect_ratio * (1.0 - aspect_ratio_tolerance_);
        obj_template.aspect_ratio_max = obj_template.aspect_ratio * (1.0 + aspect_ratio_tolerance_);
        
        // Convert to HSV
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        
        // Calculate dominant color using histogram
        std::vector<cv::Mat> hsv_channels;
        cv::split(hsv, hsv_channels);
        
        // Compute histogram for Hue channel (ignoring low saturation/value pixels)
        cv::Mat mask = (hsv_channels[1] > saturation_min_) & (hsv_channels[2] > value_min_);
        
        int hist_size = 180;
        float h_ranges[] = {0, 180};
        const float* ranges[] = {h_ranges};
        cv::Mat h_hist;
        cv::calcHist(&hsv_channels[0], 1, 0, mask, h_hist, 1, &hist_size, ranges, true, false);
        
        // Find peak in histogram (dominant hue)
        double max_val;
        cv::Point max_loc;
        cv::minMaxLoc(h_hist, nullptr, &max_val, nullptr, &max_loc);
        
        if (max_val < 10) // Not enough colored pixels
        {
            ROS_WARN("Image %s has insufficient color information", obj_template.name.c_str());
            return false;
        }
        
        int dominant_hue = max_loc.y;
        
        // Calculate mean saturation and value for colored pixels
        cv::Scalar mean_sv = cv::mean(hsv, mask);
        
        // Define HSV range around dominant color
        int h_lower = std::max(0, dominant_hue - hsv_tolerance_);
        int h_upper = std::min(179, dominant_hue + hsv_tolerance_);
        
        // Handle hue wrap-around (red color wraps at 0/180)
        if (dominant_hue < hsv_tolerance_)
        {
            // Red lower range (need two ranges but we'll use wider single range)
            h_lower = 0;
            h_upper = dominant_hue + hsv_tolerance_;
        }
        else if (dominant_hue > 180 - hsv_tolerance_)
        {
            // Red upper range
            h_lower = dominant_hue - hsv_tolerance_;
            h_upper = 179;
        }
        
        obj_template.hsv_lower = cv::Scalar(h_lower, saturation_min_, value_min_);
        obj_template.hsv_upper = cv::Scalar(h_upper, 255, 255);
        
        // Generate visualization color (use dominant color from image)
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
            
            ROS_INFO("Camera parameters updated: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f", 
                     fx_, fy_, cx_, cy_);
        }
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                      const sensor_msgs::Image::ConstPtr& depth_msg)
    {
        // Convert ROS images to OpenCV
        cv_bridge::CvImagePtr cv_rgb, cv_depth;
        try
        {
            cv_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            
            // Handle different depth encodings
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

        // Apply Gaussian blur
        cv::Mat blurred;
        if (blur_kernel_size_ > 0)
            cv::GaussianBlur(cv_rgb->image, blurred, cv::Size(blur_kernel_size_, blur_kernel_size_), 0);
        else
            blurred = cv_rgb->image.clone();

        // Convert to HSV
        cv::Mat hsv_image;
        cv::cvtColor(blurred, hsv_image, cv::COLOR_BGR2HSV);

        // Debug visualization
        cv::Mat debug_image = cv_rgb->image.clone();
        cv::Mat combined_mask = cv::Mat::zeros(hsv_image.size(), CV_8UC1);
        
        // Marker array
        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;
        int total_detections = 0;

        // Detect each object template
        for (const auto& obj_template : object_templates_)
        {
            // Create mask for this color range
            cv::Mat mask;
            cv::inRange(hsv_image, obj_template.hsv_lower, obj_template.hsv_upper, mask);
            
            // Morphological operations
            if (morph_kernel_size_ > 0)
            {
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
                    cv::Size(morph_kernel_size_, morph_kernel_size_));
                cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
                cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            }
            
            cv::bitwise_or(combined_mask, mask, combined_mask);
            
            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            // Filter by area and aspect ratio
            for (const auto& contour : contours)
            {
                double area = cv::contourArea(contour);
                
                if (area >= min_contour_area_ && area <= max_contour_area_)
                {
                    cv::Rect bbox = cv::boundingRect(contour);
                    double aspect_ratio = static_cast<double>(bbox.width) / bbox.height;
                    
                    // Check if aspect ratio matches template
                    if (aspect_ratio >= obj_template.aspect_ratio_min && 
                        aspect_ratio <= obj_template.aspect_ratio_max)
                    {
                        cv::Point2f center(bbox.x + bbox.width/2.0f, bbox.y + bbox.height/2.0f);
                        float depth = getMedianDepth(cv_depth->image, bbox);
                        
                        // Handle invalid depth
                        if (depth < 0.3 || depth > 3.0)
                        {
                            if (use_fallback_depth_)
                            {
                                ROS_WARN_THROTTLE(2.0, "Using fallback depth for %s", 
                                                 obj_template.name.c_str());
                                depth = fallback_depth_;
                            }
                            else
                                continue;
                        }
                        
                        // Convert to 3D
                        double x = (center.x - cx_) * depth / fx_;
                        double y = (center.y - cy_) * depth / fy_;
                        double z = depth;
                        
                        // Estimate dimensions
                        double width = (bbox.width * depth) / fx_;
                        double height = (bbox.height * depth) / fy_;
                        double box_depth = std::min(width, height) * 0.8;
                        
                        width = std::max(min_box_size_, std::min(max_box_size_, width));
                        height = std::max(min_box_size_, std::min(max_box_size_, height));
                        box_depth = std::max(min_box_size_, std::min(max_box_size_, box_depth));
                        
                        ROS_INFO("Detected '%s' at [%.3f, %.3f, %.3f]m, size [%.3f x %.3f x %.3f]m, AR=%.2f",
                                 obj_template.name.c_str(), x, y, z, width, box_depth, height, aspect_ratio);
                        
                        total_detections++;
                        
                        // Create markers
                        marker_array.markers.push_back(createBoxMarker(
                            marker_id++, rgb_msg->header, x, y, z, 
                            width, box_depth, height, obj_template.viz_color));
                        
                        marker_array.markers.push_back(createTextMarker(
                            marker_id++, rgb_msg->header, x, y, z + height/2 + 0.05, 
                            obj_template.name));
                        
                        // Draw on debug image
                        cv::rectangle(debug_image, bbox, obj_template.viz_color, 2);
                        cv::putText(debug_image, obj_template.name, 
                                   cv::Point(bbox.x, bbox.y - 10),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.6, obj_template.viz_color, 2);
                        cv::circle(debug_image, center, 5, obj_template.viz_color, -1);
                        
                        std::stringstream ss;
                        ss << std::fixed << std::setprecision(2) << z << "m AR:" << aspect_ratio;
                        cv::putText(debug_image, ss.str(), 
                                   cv::Point(bbox.x, bbox.y + bbox.height + 20),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.4, obj_template.viz_color, 1);
                    }
                }
            }
        }

        ROS_INFO_THROTTLE(1.0, "Frame detections: %d", total_detections);

        // Publish
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
            // Try expanded ROI
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

    visualization_msgs::Marker createTextMarker(int id, const std_msgs::Header& header,
                                                double x, double y, double z,
                                                const std::string& text)
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
        
        marker.text = text;
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