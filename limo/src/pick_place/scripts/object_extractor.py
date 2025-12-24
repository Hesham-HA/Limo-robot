#!/usr/bin/env python3
import rospy
import tf
import math
from find_object_2d.msg import ObjectsStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo

class TfExample:
    def __init__(self):
        # Parameters
        self.target_frame_id = rospy.get_param('~target_frame_id', '')
        self.obj_frame_prefix = rospy.get_param('~object_prefix', 'object')
        
        # Initialize camera intrinsic parameters
        self.camera_fx = 554.0  # Focal length X (pixels)
        self.camera_fy = 554.0  # Focal length Y (pixels)
        # Default depth for object (will be updated from TF if available)
        self.default_depth = 0.5  # meters
        # Scale factor to convert from 2D detection to 3D estimate
        self.scale_factor = 1.0
        # Minimum dimensions (safety bounds for real-world objects)
        self.min_dimension = 0.03  # 3cm
        self.max_dimension = 0.5   # 50cm
        
        # TF listener with larger cache time
        self.tf_listener = tf.TransformListener(cache_time=rospy.Duration(10.0))
        # Wait for TF to be ready
        rospy.sleep(1.0)
        
        # Subscribers
        rospy.Subscriber('objectsStamped', ObjectsStamped, self.objects_detected_callback, queue_size=1)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.extract_camera_info)
        
        # Publisher for bounding box markers
        self.marker_pub = rospy.Publisher('detection/object', MarkerArray, queue_size=1)
        
        rospy.loginfo("TF Example Node initialized")
    
    def extract_camera_info(self, msg: CameraInfo):
        """
        Extract camera intrinsic parameters from CameraInfo message.
        """
        try:
            # Intrinsic matrix K
            self.camera_fx = msg.K[0]
            self.camera_fy = msg.K[4]
            # Image dimensions (safety bounds)
            width = msg.width
            height = msg.height
            rospy.loginfo(f"Camera parameters: fx={self.camera_fx}, fy={self.camera_fy}")
            rospy.loginfo(f"Scale factor: {self.scale_factor}")
            rospy.loginfo(f"Image size: {width}x{height} pixels")
            rospy.loginfo(f"Object dimension bounds: [{self.min_dimension}, {self.max_dimension}]m")
        except Exception as e:
            rospy.logwarn(f"Failed to extract camera info: {e}")
    
    def extract_object_info(self, data, index):
        """
        Extract object information from the data array.
        Format: [objectId, objectWidth, objectHeight, h11, h12, h13, h21, h22, h23, h31, h32, h33]
        Returns: (object_id, width_pixels, height_pixels, homography_matrix)
        """
        obj_id = int(data[index])
        width_px = data[index + 1]
        height_px = data[index + 2]
        
        # Homography matrix (3x3)
        homography = [
            [data[index + 3], data[index + 4], data[index + 5]],   # h11, h12, h13
            [data[index + 6], data[index + 7], data[index + 8]],   # h21, h22, h23
            [data[index + 9], data[index + 10], data[index + 11]]  # h31, h32, h33
        ]
        
        # h31 = dx (translation in x)
        # h32 = dy (translation in y)
        dx = homography[2][0]
        dy = homography[2][1]
        
        return obj_id, width_px, height_px, homography, dx, dy
    
    def estimate_3d_dimensions(self, width_px, height_px, depth, homography):
        """
        Estimate 3D dimensions from 2D detection and depth.
        
        The homography tells us about the transformation, and we can estimate
        the real-world size based on the pixel size and depth.
        
        Real_width = (width_pixels * depth) / focal_length_x
        Real_height = (height_pixels * depth) / focal_length_y
        """
        # Calculate scale from homography if available
        # The scale can be estimated from the homography matrix diagonal elements
        h11 = homography[0][0]
        h22 = homography[1][1]
        scale = math.sqrt(h11 * h11 + h22 * h22) / math.sqrt(2.0)
        
        # Apply scale factor to the homography-based estimate
        scale *= self.scale_factor
        
        # Estimate real-world dimensions using pinhole camera model
        real_width = (width_px * depth) / self.camera_fx * scale
        real_height = (height_px * depth) / self.camera_fy * scale
        
        # The depth dimension (assuming cylindrical object like a can)
        # Use the minimum of width/height as diameter, assume similar depth
        real_depth = min(real_width, real_height)
        
        # Clamp dimensions to reasonable bounds
        real_width = max(self.min_dimension, min(self.max_dimension, real_width))
        real_height = max(self.min_dimension, min(self.max_dimension, real_height))
        real_depth = max(self.min_dimension, min(self.max_dimension, real_depth))
        
        return real_width, real_height, real_depth
    
    def objects_detected_callback(self, msg):
        """
        Callback for ObjectsStamped messages.
        Retrieves object transforms and publishes 3D bounding boxes for RViz.
        """
        if not msg.objects.data:
            return
        
        # Determine target frame
        target_frame_id = self.target_frame_id if self.target_frame_id else msg.header.frame_id
        
        # Create marker array for visualization
        marker_array = MarkerArray()
        multi_sub_id = ord('b')
        previous_id = -1
        marker_id = 0
        
        # Process each detected object (data comes in groups of 12)
        for i in range(0, len(msg.objects.data), 12):
            # Extract object information
            obj_id, width_px, height_px, homography, dx, dy = self.extract_object_info(msg.objects.data, i)
            
            rospy.loginfo(f"Detected object {obj_id}: {width_px:.1f}x{height_px:.1f} pixels, "
                         f"dx={dx:.1f}, dy={dy:.1f}")
            
            # Handle multiple detections of same object
            multi_suffix = ''
            if obj_id == previous_id:
                multi_suffix = '_' + chr(multi_sub_id)
                multi_sub_id += 1
            else:
                multi_sub_id = ord('b')
            previous_id = obj_id
            
            # Create object frame name
            object_frame_id = f"{self.obj_frame_prefix}_{obj_id}{multi_suffix}"
            
            try:
                # Get latest common time to avoid extrapolation
                common_time = self.tf_listener.getLatestCommonTime(target_frame_id, object_frame_id)
                
                # Get transformation
                (trans, rot) = self.tf_listener.lookupTransform(
                    target_frame_id, object_frame_id, common_time)
                
                # Use the actual depth from TF transform
                depth = abs(trans[2]) if abs(trans[2]) > 0.1 else self.default_depth
                
                # Estimate 3D dimensions from 2D detection
                box_width, box_height, box_depth = self.estimate_3d_dimensions(
                    width_px, height_px, depth, homography)
                
                rospy.loginfo(f"{object_frame_id} at depth {depth:.3f}m: "
                             f"estimated size [{box_width:.3f} x {box_depth:.3f} x {box_height:.3f}]m")
                
                # Log pose
                rospy.loginfo(f"  Position: [{trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f}]")
                rospy.loginfo(f"  Orientation: [{rot[0]:.3f}, {rot[1]:.3f}, {rot[2]:.3f}, {rot[3]:.3f}]")
                
                # Create bounding box marker with calculated dimensions
                marker = self.create_bounding_box_marker(
                    marker_id, object_frame_id, target_frame_id, 
                    trans, rot, rospy.Time.now(),
                    box_width, box_depth, box_height)  # Pass calculated dimensions
                marker_array.markers.append(marker)
                
                # Create text label marker
                text_marker = self.create_text_marker(
                    marker_id + 1000, object_frame_id, target_frame_id,
                    trans, rot, rospy.Time.now(), obj_id, box_height)
                marker_array.markers.append(text_marker)
                
                marker_id += 1
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logwarn_throttle(5.0, f"TF Exception for {object_frame_id}: {ex}")
                continue
        
        # Publish markers
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def create_bounding_box_marker(self, marker_id, object_frame, target_frame, trans, rot, stamp, width, depth, height):
        """
        Create a 3D bounding box marker for RViz visualization.
        """
        marker = Marker()
        marker.header.frame_id = target_frame
        marker.header.stamp = stamp
        marker.ns = "bounding_boxes"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]
        
        # Set orientation
        marker.pose.orientation.x = rot[0]
        marker.pose.orientation.y = rot[1]
        marker.pose.orientation.z = rot[2]
        marker.pose.orientation.w = rot[3]
        
        # Set scale (bounding box dimensions) - using calculated values
        marker.scale.x = width
        marker.scale.y = depth
        marker.scale.z = height
        
        # Set color (semi-transparent green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Semi-transparent
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker
    
    def create_text_marker(self, marker_id, object_frame, target_frame, trans, rot, stamp, obj_id, height):
        """
        Create a text label marker showing object ID and dimensions.
        """
        marker = Marker()
        marker.header.frame_id = target_frame
        marker.header.stamp = stamp
        marker.ns = "labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position text above the bounding box
        marker.pose.position.x = trans[0]
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2] + height/2 + 0.1
        
        marker.pose.orientation.w = 1.0
        
        # Text content
        marker.text = f"Object {obj_id}"
        
        # Text size
        marker.scale.z = 0.1
        
        # Color (white)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(0.5)
        
        return marker


def main():
    rospy.init_node('object_extractor', anonymous=False)    
    try:
        tf_example = TfExample()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()