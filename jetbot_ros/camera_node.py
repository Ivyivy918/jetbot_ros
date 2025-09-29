#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
import threading
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Image storage
        self.left_image = None
        self.right_image = None
        self.image_lock = threading.Lock()
        
        # Camera objects and threading
        self.camera_left = None
        self.camera_right = None
        self.capture_thread = None
        self.running = False
        
        # Parameter declarations
        self.declare_parameter('obstacle_threshold', 2000)
        self.declare_parameter('detection_area_ratio', 0.6)
        self.declare_parameter('debug_mode', False)
        self.declare_parameter('baseline_mm', 60)  
        self.declare_parameter('focal_length', 400)
        self.declare_parameter('stop_distance_cm', 15.0)
        
        # Parameter values
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.detection_area_ratio = self.get_parameter('detection_area_ratio').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.baseline_mm = self.get_parameter('baseline_mm').value
        self.focal_length = self.get_parameter('focal_length').value
        self.stop_distance_cm = self.get_parameter('stop_distance_cm').value
        
        # Publishers
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_detected', 10)
        self.distance_pub = self.create_publisher(Float32, 'obstacle_distance', 10)
        self.left_image_pub = self.create_publisher(Image, '/camera_left/image_raw', 10)
        self.right_image_pub = self.create_publisher(Image, '/camera_right/image_raw', 10)
        
        # Processing timer
        self.create_timer(0.5, self.process_images)  # 10Hz
        
        # Stereo vision matcher
        self.stereo_matcher = cv2.StereoBM_create(numDisparities=64, blockSize=15)
        
        # Initialize cameras
        self.init_physical_cameras()
    
    def init_physical_cameras(self):
        """Initialize CSI cameras with GStreamer pipeline"""
        try:
            self.get_logger().info("Initializing dual CSI cameras...")
            
            # Left camera (CSI-0)
            gst_left = (
                "nvarguscamerasrc sensor_id=0 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_left = cv2.VideoCapture(gst_left, cv2.CAP_GSTREAMER)
            
            # Right camera (CSI-1)
            gst_right = (
                "nvarguscamerasrc sensor_id=1 ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            self.camera_right = cv2.VideoCapture(gst_right, cv2.CAP_GSTREAMER)
            
            # Check camera initialization
            if self.camera_left.isOpened() and self.camera_right.isOpened():
                self.get_logger().info("Dual CSI cameras initialized successfully")
                self.start_capture_thread()
            else:
                self.get_logger().error("Failed to initialize cameras. Check connections.")
                
        except Exception as e:
            self.get_logger().error(f"Camera initialization error: {e}")
    
    def start_capture_thread(self):
        """Start camera capture thread"""
        self.running = True
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()
        self.get_logger().info("Camera capture thread started")
    
    def capture_loop(self):
        """Main camera capture loop"""
        while self.running:
            try:
                frames_captured = False
                
                # Read left camera
                if self.camera_left and self.camera_left.isOpened():
                    ret_left, frame_left = self.camera_left.read()
                    if ret_left:
                        with self.image_lock:
                            self.left_image = frame_left
                            frames_captured = True
                        
                        # Publish left camera image
                        try:
                            img_msg = self.bridge.cv2_to_imgmsg(frame_left, 'bgr8')
                            self.left_image_pub.publish(img_msg)
                        except:
                            pass
                
                # Read right camera  
                if self.camera_right and self.camera_right.isOpened():
                    ret_right, frame_right = self.camera_right.read()
                    if ret_right:
                        with self.image_lock:
                            self.right_image = frame_right
                            frames_captured = True
                        
                        # Publish right camera image
                        try:
                            img_msg = self.bridge.cv2_to_imgmsg(frame_right, 'bgr8')
                            self.right_image_pub.publish(img_msg)
                        except:
                            pass
                
                if not frames_captured:
                    self.get_logger().warning("Failed to capture camera frames")
                    
            except Exception as e:
                self.get_logger().error(f"Camera capture error: {e}")
                break
            
            time.sleep(0.033)  # ~30 FPS
    
    def process_images(self):
        """Process dual camera images and publish detection results"""
        with self.image_lock:
            if self.left_image is None or self.right_image is None:
                print("Waiting for camera frames...")
                self.publish_detection_results(False, -1.0)
                return
            
            left_img = self.left_image.copy()
            right_img = self.right_image.copy()
        
        try:
            # Calculate distances from individual cameras
            left_distance_cm = self.calculate_single_camera_distance(left_img, "left")
            right_distance_cm = self.calculate_single_camera_distance(right_img, "right")
            
            # Calculate stereo vision distance
            stereo_distance_cm = self.calculate_stereo_distance(left_img, right_img)
            
            # Display detection results
            self.display_detection_results(left_distance_cm, right_distance_cm, stereo_distance_cm)
            
            # Determine if motor should stop
            obstacle_detected = self.should_stop_motor(left_distance_cm, right_distance_cm, stereo_distance_cm)
            
            # Select most reliable distance for publishing
            final_distance = self.get_best_distance(left_distance_cm, right_distance_cm, stereo_distance_cm)
            
            # Publish results to motor node
            self.publish_detection_results(obstacle_detected, final_distance / 100.0 if final_distance > 0 else -1.0)
            
            # Debug mode: show images
            if self.debug_mode:
                self.show_debug_images(left_img, right_img, obstacle_detected, final_distance)
                
        except Exception as e:
            print(f"Processing error: {e}")
            self.publish_detection_results(False, -1.0)
    
    def calculate_single_camera_distance(self, image, camera_name):
        """Calculate obstacle distance using single camera (cm) - enhanced version"""
        try:
            height, width = image.shape[:2]
            
            # Use multiple detection methods for accuracy
            distance_estimates = []
            
            # Method 1: Contour-based detection
            contour_distance = self.detect_by_contour(image)
            if contour_distance > 0:
                distance_estimates.append(contour_distance)
            
            # Method 2: Brightness-based detection (effective for close objects)
            brightness_distance = self.detect_by_brightness(image)
            if brightness_distance > 0:
                distance_estimates.append(brightness_distance)
            
            # Method 3: Edge density detection
            edge_distance = self.detect_by_edge_density(image)
            if edge_distance > 0:
                distance_estimates.append(edge_distance)
            
            # Method 4: Blur detection (close objects tend to blur)
            blur_distance = self.detect_by_blur(image)
            if blur_distance > 0:
                distance_estimates.append(blur_distance)
            
            # Return minimum estimate (most conservative)
            if distance_estimates:
                final_distance = min(distance_estimates)
                return final_distance
            else:
                return -1.0
                
        except Exception as e:
            self.get_logger().debug(f"{camera_name} camera distance calculation error: {e}")
            return -1.0
    
    def detect_by_contour(self, image):
        """Distance detection based on contour area"""
        try:
            height, width = image.shape[:2]
            
            # Detection region (front center)
            roi_y_start = int(height * 0.4)  # Expanded detection area
            roi_x_start = int(width * 0.25)
            roi_x_end = int(width * 0.75)
            roi = image[roi_y_start:height, roi_x_start:roi_x_end]
            
            # Edge detection
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 30, 100)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                max_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(max_contour)
                
                # Distance mapping based on contour area
                if area > 8000:      # Very large contour = very close
                    return 3.0
                elif area > 6000:    # Large contour = close
                    return 5.0
                elif area > 4000:    # Medium-large contour = somewhat close
                    return 8.0
                elif area > 2500:    # Medium contour = moderately close
                    return 12.0
                elif area > 1500:    # Medium-small contour = medium distance
                    return 18.0
                elif area > 800:     # Small contour = somewhat far
                    return 25.0
                elif area > 400:     # Very small contour = far
                    return 40.0
                elif area > 100:     # Tiny contour = very far
                    return 60.0
                else:
                    return -1.0
            
            return -1.0
                
        except Exception as e:
            return -1.0
    
    def detect_by_brightness(self, image):
        """Distance detection based on brightness changes - effective for close objects"""
        try:
            height, width = image.shape[:2]
            
            # Extract center region
            center_y = height // 2
            center_x = width // 2
            roi_size = min(height, width) // 4
            
            center_roi = image[
                max(0, center_y - roi_size):min(height, center_y + roi_size),
                max(0, center_x - roi_size):min(width, center_x + roi_size)
            ]
            
            # Convert to grayscale and calculate statistics
            gray = cv2.cvtColor(center_roi, cv2.COLOR_BGR2GRAY)
            mean_brightness = np.mean(gray)
            std_brightness = np.std(gray)
            
            # Calculate brightness gradient
            grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
            gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
            mean_gradient = np.mean(gradient_magnitude)
            
            # Close objects typically have high brightness variation and low std dev
            if mean_brightness > 180 and std_brightness < 30:  # Bright and uniform = very close
                return 2.0
            elif mean_gradient > 50:  # High gradient = object present and close
                if mean_gradient > 100:
                    return 4.0
                elif mean_gradient > 80:
                    return 7.0
                elif mean_gradient > 60:
                    return 12.0
                else:
                    return 20.0
            
            return -1.0
                
        except Exception as e:
            return -1.0
    
    def detect_by_edge_density(self, image):
        """Distance detection based on edge density"""
        try:
            height, width = image.shape[:2]
            
            # Multiple detection regions
            regions = [
                (int(height * 0.3), height, int(width * 0.3), int(width * 0.7)),  # Lower center
                (int(height * 0.4), int(height * 0.8), int(width * 0.25), int(width * 0.75)),  # Center
            ]
            
            max_density = 0
            
            for y1, y2, x1, x2 in regions:
                roi = image[y1:y2, x1:x2]
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray, 50, 150)
                edge_count = cv2.countNonZero(edges)
                
                # Calculate edge density
                roi_area = (y2 - y1) * (x2 - x1)
                density = edge_count / roi_area if roi_area > 0 else 0
                max_density = max(max_density, density)
            
            # Distance estimation based on edge density
            if max_density > 0.15:      # Very high density = very close
                return 3.0
            elif max_density > 0.12:    # High density = close
                return 6.0
            elif max_density > 0.08:    # Medium-high density = somewhat close
                return 10.0
            elif max_density > 0.05:    # Medium density = medium distance
                return 18.0
            elif max_density > 0.02:    # Low density = far
                return 35.0
            elif max_density > 0.01:    # Very low density = very far
                return 60.0
            
            return -1.0
                
        except Exception as e:
            return -1.0
    
    def detect_by_blur(self, image):
        """Distance detection based on blur - objects too close will be out of focus"""
        try:
            height, width = image.shape[:2]
            
            # Extract center region
            center_y = height // 2
            center_x = width // 2
            roi_size = min(height, width) // 6
            
            center_roi = image[
                max(0, center_y - roi_size):min(height, center_y + roi_size),
                max(0, center_x - roi_size):min(width, center_x + roi_size)
            ]
            
            # Convert to grayscale
            gray = cv2.cvtColor(center_roi, cv2.COLOR_BGR2GRAY)
            
            # Calculate Laplacian variance to measure sharpness
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            # Calculate pixel intensity standard deviation
            pixel_std = np.std(gray)
            
            # Very blurry usually means object is too close (out of focus)
            if laplacian_var < 50 and pixel_std > 20:  # Blurry but has content = too close
                return 2.0
            elif laplacian_var < 100 and pixel_std > 30:  # Slightly blurry = close
                return 5.0
            elif laplacian_var < 200:  # Mild blur = somewhat close
                return 10.0
            
            return -1.0
                
        except Exception as e:
            return -1.0
    
    def calculate_stereo_distance(self, left_image, right_image):
        """Calculate distance using stereo vision (cm)"""
        try:
            # Convert to grayscale
            gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            
            # Stereo matching
            stereo = cv2.StereoBM_create(numDisparities=48, blockSize=11)
            disparity = stereo.compute(gray_left, gray_right)
            
            # Extract central region disparity
            height, width = disparity.shape
            center_y = height // 2
            center_x = width // 2
            roi_size = 30
            
            center_disparity = disparity[
                center_y-roi_size:center_y+roi_size,
                center_x-roi_size:center_x+roi_size
            ]
            
            # Filter valid disparities
            valid_disp = center_disparity[(center_disparity > 0) & (center_disparity < 300)]
            
            if len(valid_disp) < 20:
                return -1.0
            
            # Calculate average disparity
            avg_disparity = np.median(valid_disp) / 16.0
            
            if avg_disparity <= 0.5:
                return -1.0
            
            # Distance calculation (adjusted parameters for real-world conditions)
            focal_length = 200  # Adjusted focal length
            baseline_mm = 60    # 6cm baseline
            
            distance_mm = (baseline_mm * focal_length) / avg_disparity
            distance_cm = distance_mm / 10.0
            
            # Reasonable range check
            if 3.0 <= distance_cm <= 200.0:
                return distance_cm
            else:
                return -1.0
                
        except Exception as e:
            self.get_logger().debug(f"Stereo vision calculation error: {e}")
            return -1.0
    
    def display_detection_results(self, left_distance, right_distance, stereo_distance):
        """Display detection results - show only most reliable distance"""
        
        # Determine most reliable distance
        reliable_distance = -1
        distance_source = "no detection"
        
        # Priority 1: Stereo vision (within reasonable range)
        if stereo_distance > 0 and 5.0 <= stereo_distance <= 300.0:
            reliable_distance = stereo_distance
            distance_source = "stereo"
        
        # Priority 2: Dual camera consistency check
        elif left_distance > 0 and right_distance > 0:
            distance_diff = abs(left_distance - right_distance)
            if distance_diff <= 8.0:  # Difference less than 8cm considered consistent
                reliable_distance = (left_distance + right_distance) / 2
                distance_source = "dual cam"
            else:
                # When difference is large, choose more reliable value
                min_distance = min(left_distance, right_distance)
                max_distance = max(left_distance, right_distance)
                if min_distance < 3.0 and max_distance > 10.0:
                    reliable_distance = max_distance
                    distance_source = "single cam"
                else:
                    reliable_distance = min_distance
                    distance_source = "single cam"
        
        # Priority 3: Single camera detection
        elif left_distance > 0:
            reliable_distance = left_distance
            distance_source = "left cam"
        elif right_distance > 0:
            reliable_distance = right_distance
            distance_source = "right cam"
        
        # Print reliable distance
        if reliable_distance > 0:
            print(f"Distance: {reliable_distance:.1f}cm ({distance_source})")
        else:
            print("No obstacle detected")
    
    def should_stop_motor(self, left_distance, right_distance, stereo_distance):
        """Determine if motor should stop - intelligent priority judgment"""
        
        # Priority 1: Stereo vision distance (most reliable)
        if stereo_distance > 0:
            should_stop = stereo_distance <= self.stop_distance_cm
            if should_stop:
                print(f"Stereo detection: Obstacle too close ({stereo_distance:.1f}cm ≤ {self.stop_distance_cm}cm) - Motor stop")
            return should_stop
        
        # Priority 2: If stereo vision invalid, check if both cameras detect very close objects (<5cm)
        very_close_detections = []
        if left_distance > 0 and left_distance <= 5.0:
            very_close_detections.append(left_distance)
        if right_distance > 0 and right_distance <= 5.0:
            very_close_detections.append(right_distance)
        
        # Only trust single camera results if both cameras detect very close objects
        if len(very_close_detections) >= 2:
            min_distance = min(very_close_detections)
            should_stop = min_distance <= self.stop_distance_cm
            if should_stop:
                print(f"Dual cam close detection: Obstacle too close ({min_distance:.1f}cm ≤ {self.stop_distance_cm}cm) - Motor stop")
            return should_stop
        
        # Priority 3: If only one camera detects very close, need additional verification
        if very_close_detections:
            min_distance = min(very_close_detections)
            # Only trust single camera for extremely close distances (<3cm)
            if min_distance <= 3.0:
                print(f"Single cam critical detection: Obstacle too close ({min_distance:.1f}cm ≤ 3cm) - Motor stop")
                return True
        
        # Priority 4: Check consistency between both cameras
        if left_distance > 0 and right_distance > 0:
            # Calculate difference between camera distances
            distance_diff = abs(left_distance - right_distance)
            avg_distance = (left_distance + right_distance) / 2
            
            # If both cameras show similar distances and both are below threshold
            if distance_diff <= 5.0 and avg_distance <= self.stop_distance_cm:
                print(f"Dual cam consistent detection: Obstacle too close ({avg_distance:.1f}cm ≤ {self.stop_distance_cm}cm) - Motor stop")
                return True
        
        # Priority 5: Single camera detection at medium distance
        single_camera_distances = []
        if left_distance > 0:
            single_camera_distances.append(left_distance)
        if right_distance > 0:
            single_camera_distances.append(right_distance)
        
        if single_camera_distances:
            min_single = min(single_camera_distances)
            # For medium distances, use more conservative judgment
            if min_single <= self.stop_distance_cm * 0.7:  # 70% threshold
                print(f"Single cam conservative detection: Obstacle close ({min_single:.1f}cm ≤ {self.stop_distance_cm * 0.7:.1f}cm) - Motor stop")
                return True
        
        # If no clear stop conditions, continue running
        return False
    
    def get_best_distance(self, left_distance, right_distance, stereo_distance):
        """Select most reliable distance value - intelligent priority selection"""
        
        # Priority 1: Stereo vision result (most reliable and within reasonable range)
        if stereo_distance > 0 and 5.0 <= stereo_distance <= 300.0:
            return stereo_distance
        
        # Priority 2: If stereo vision invalid, check single camera consistency
        if left_distance > 0 and right_distance > 0:
            distance_diff = abs(left_distance - right_distance)
            avg_distance = (left_distance + right_distance) / 2
            
            # If both cameras show similar distances, use average
            if distance_diff <= 8.0:  # Difference less than 8cm considered consistent
                return avg_distance
            else:
                # If difference is large, choose more conservative (closer) value, but reasonable
                min_distance = min(left_distance, right_distance)
                max_distance = max(left_distance, right_distance)
                
                # If minimum value too small (possible false detection), choose larger value
                if min_distance < 3.0 and max_distance > 10.0:
                    return max_distance
                else:
                    return min_distance
        
        # Priority 3: Only one single camera valid
        if left_distance > 0:
            return left_distance
        if right_distance > 0:
            return right_distance
        
        # Priority 4: If stereo vision distance abnormal but exists, log but don't use
        if stereo_distance > 0:
            print(f"    └─ Stereo vision distance abnormal: {stereo_distance:.1f}cm (ignored)")
        
        return -1.0
    
    def publish_detection_results(self, obstacle_detected, distance):
        """Publish detection results to motor node"""
        try:
            # Publish obstacle detection status
            obstacle_msg = Bool()
            obstacle_msg.data = bool(obstacle_detected)
            self.obstacle_pub.publish(obstacle_msg)
            
            # Publish distance
            distance_msg = Float32()
            distance_msg.data = float(distance) if distance > 0 else -1.0
            self.distance_pub.publish(distance_msg)
            
        except Exception as e:
            self.get_logger().error(f"Publishing results error: {e}")
    
    def show_debug_images(self, left_image, right_image, obstacle_detected, distance):
        """Display debug images"""
        try:
            # Draw detection regions and results on left image
            debug_left = left_image.copy()
            debug_right = right_image.copy()
            height, width = debug_left.shape[:2]
            
            # Draw detection region
            roi_y_start = int(height * 0.5)
            roi_x_start = int(width * 0.3)
            roi_x_end = int(width * 0.7)
            
            color = (0, 0, 255) if obstacle_detected else (0, 255, 0)
            
            # Draw detection regions on both images
            cv2.rectangle(debug_left, (roi_x_start, roi_y_start), 
                         (roi_x_end, height), color, 2)
            cv2.rectangle(debug_right, (roi_x_start, roi_y_start), 
                         (roi_x_end, height), color, 2)
            
            # Display detection results
            status_text = "STOP" if obstacle_detected else "CLEAR"
            cv2.putText(debug_left, f"LEFT: {status_text}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(debug_right, f"RIGHT: {status_text}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Display distance
            if distance > 0:
                distance_text = f"Distance: {distance:.1f}cm"
                cv2.putText(debug_left, distance_text, (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # Combined display
            combined = cv2.hconcat([debug_left, debug_right])
            cv2.imshow('Dual Camera Detection', combined)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Debug image display error: {e}")
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        
        if self.capture_thread:
            self.capture_thread.join(timeout=1)
        
        if self.camera_left:
            self.camera_left.release()
        if self.camera_right:
            self.camera_right.release()
        
        cv2.destroyAllWindows()
        
    def destroy_node(self):
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        print("=== Dual Camera Obstacle Detection Node ===")
        print(f"Stop distance threshold: {node.stop_distance_cm} cm")
        print("-" * 50)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutdown signal received")
    except Exception as e:
        print(f"Execution error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()