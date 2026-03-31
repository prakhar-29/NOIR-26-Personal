#!/usr/bin/env python3
"""
Optimized OpenVINO Cone Detection ROS2 Node
High-performance inference with temporal filtering
"""

import cv2
import numpy as np
try:
    from openvino import Core
except ImportError:
    from openvino.runtime import Core  # Fallback for older versions
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from collections import deque
import time

class DetectionTracker:
    """Track detections across frames to filter false positives"""
    
    def __init__(self, history_size=5, min_frames=3):
        self.history = deque(maxlen=history_size)
        self.min_frames = min_frames
    
    def update(self, detections):
        """Add new detections to history"""
        self.history.append(detections)
    
    def get_stable_detections(self):
        """Return only detections that appear consistently"""
        if len(self.history) < self.min_frames:
            return []
        
        stable = []
        current = self.history[-1]
        
        for det in current:
            matches = 0
            cx_current = (det['bbox'][0] + det['bbox'][2]) / 2
            cy_current = (det['bbox'][1] + det['bbox'][3]) / 2
            
            for past_dets in self.history:
                for past_det in past_dets:
                    if past_det['class_id'] != det['class_id']:
                        continue
                    
                    cx_past = (past_det['bbox'][0] + past_det['bbox'][2]) / 2
                    cy_past = (past_det['bbox'][1] + past_det['bbox'][3]) / 2
                    
                    distance = np.sqrt((cx_current - cx_past)**2 + (cy_current - cy_past)**2)
                    
                    if distance < 50:
                        matches += 1
                        break
            
            if matches >= self.min_frames:
                stable.append(det)
        
        return stable


class OpenVINOConeDetectionNode(Node):
    def __init__(self):
        super().__init__('openvino_cone_detection_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'openvino_models/yolov8s_cone_fp32.xml')
        self.declare_parameter('device', 'CPU')  # CPU, GPU, or AUTO
        self.declare_parameter('camera_device', '/dev/video2')  # CHANGED: now accepts path
        self.declare_parameter('conf_threshold', 0.018)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('min_box_area', 500)
        self.declare_parameter('max_box_area', 200000)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 640)
        self.declare_parameter('use_stability_filter', True)
        self.declare_parameter('history_size', 5)
        self.declare_parameter('min_detection_frames', 3)
        self.declare_parameter('target_color', 'all')
        self.declare_parameter('show_visualization', False)
        self.declare_parameter('use_async_inference', False)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.device = self.get_parameter('device').value
        self.camera_device = self.get_parameter('camera_device').value  # CHANGED
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.min_box_area = self.get_parameter('min_box_area').value
        self.max_box_area = self.get_parameter('max_box_area').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.use_stability_filter = self.get_parameter('use_stability_filter').value
        self.history_size = self.get_parameter('history_size').value
        self.min_detection_frames = self.get_parameter('min_detection_frames').value
        self.target_color_name = self.get_parameter('target_color').value
        self.show_visualization = self.get_parameter('show_visualization').value
        self.use_async = self.get_parameter('use_async_inference').value
        
        # Class definitions (1-indexed to match original code)
        # 0 = reserved for "no detection"
        self.CLASS_NAMES = {
            1: "yellow",
            2: "blue", 
            3: "orange",
            4: "large_orange",
            5: "red",
            6: "green",
            7: "others"
        }
        
        self.CLASS_COLORS = {
            1: (0, 255, 255),
            2: (255, 0, 0),
            3: (0, 165, 255),
            4: (0, 140, 255),
            5: (0, 0, 255),
            6: (0, 255, 0),
            7: (128, 128, 128)
        }
        
        # Setup target color filtering
        self.COLOR_TO_ID = {}
        for id_val, label in self.CLASS_NAMES.items():
            if label not in self.COLOR_TO_ID:
                self.COLOR_TO_ID[label] = []
            self.COLOR_TO_ID[label].append(id_val)
        
        if self.target_color_name.lower() == 'all':
            self.target_ids = list(self.CLASS_NAMES.keys())
            self.get_logger().info("Detecting ALL cone colors")
        else:
            if self.target_color_name not in self.COLOR_TO_ID:
                self.get_logger().error(f"Invalid color '{self.target_color_name}'")
                raise ValueError(f"Invalid target color: {self.target_color_name}")
            self.target_ids = self.COLOR_TO_ID[self.target_color_name]
        
        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, 'cone_coordinates', 10)
        
        # Initialize OpenVINO
        self.get_logger().info("Loading OpenVINO model...")
        self.core = Core()
        
        # Load and compile model
        model = self.core.read_model(self.model_path)
        
        # Optimize for performance with version-compatible settings
        config = {}
        if self.device == 'CPU':
            # Try to use optimal settings, fallback if not supported
            try:
                # Check available properties
                available_props = self.core.get_property(self.device, 'SUPPORTED_PROPERTIES')
                
                # Always safe properties
                config['PERFORMANCE_HINT'] = 'LATENCY'
                
                # Add optional properties if supported
                if 'CPU_BIND_THREAD' in available_props:
                    config['CPU_BIND_THREAD'] = 'YES'
                if 'CPU_THROUGHPUT_STREAMS' in available_props:
                    config['CPU_THROUGHPUT_STREAMS'] = '1'
                if 'INFERENCE_PRECISION_HINT' in available_props:
                    config['INFERENCE_PRECISION_HINT'] = 'f32'
                    
                self.get_logger().info(f"Using CPU config: {list(config.keys())}")
            except Exception as e:
                # Fallback to basic config
                self.get_logger().warn(f"Some optimizations not available: {e}")
                config = {'PERFORMANCE_HINT': 'LATENCY'}
        else:
            config = {'PERFORMANCE_HINT': 'LATENCY'}
        
        self.compiled_model = self.core.compile_model(model, self.device, config)
        
        if self.use_async:
            self.infer_request = self.compiled_model.create_infer_request()
            self.get_logger().info("Using ASYNC inference mode")
        else:
            self.infer_request = self.compiled_model.create_infer_request()
            self.get_logger().info("Using SYNC inference mode")
        
        self.get_logger().info(f"Model loaded on {self.device}")
        
        # Warmup
        self.get_logger().info("Warming up model...")
        dummy = np.random.randn(1, 3, 640, 640).astype(np.float32)
        for _ in range(5):
            self.infer_request.infer({0: dummy})
        self.get_logger().info("Warmup complete")
        
        # Initialize tracker
        if self.use_stability_filter:
            self.tracker = DetectionTracker(self.history_size, self.min_detection_frames)
        else:
            self.tracker = None
        
        # Performance tracking
        self.inference_times = deque(maxlen=30)
        self.frame_count = 0
        
        # Pre-allocate input tensor for performance
        self.input_tensor = np.zeros((1, 3, 640, 640), dtype=np.float32)
        
        self.get_logger().info(
            f"OpenVINO Cone Detection Node initialized\n"
            f"  Device: {self.device}\n"
            f"  Camera: {self.camera_device}\n"
            f"  Target colors: {self.target_color_name}\n"
            f"  Confidence threshold: {self.conf_threshold}\n"
            f"  Stability filter: {self.use_stability_filter}\n"
            f"  Async inference: {self.use_async}"
        )
    
    def preprocess_frame(self, frame):
        """Optimized preprocessing"""
        # Resize
        resized = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_LINEAR)
        
        # Convert to RGB and normalize in one go
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # Transpose and normalize (write directly to pre-allocated tensor)
        np.divide(rgb.transpose(2, 0, 1), 255.0, out=self.input_tensor[0], casting='unsafe')
        
        return self.input_tensor
    
    def postprocess_detections(self, output, frame_shape):
        """Process YOLOv8 output with filtering"""
        predictions = output[0].T  # [8400, 11]
        
        boxes = predictions[:, :4]
        scores = predictions[:, 4:]
        
        # Vectorized operations
        class_ids = np.argmax(scores, axis=1)
        confidences = np.max(scores, axis=1)
        
        # Filter by confidence
        mask = confidences > self.conf_threshold
        
        if not np.any(mask):
            return []
        
        boxes = boxes[mask]
        class_ids = class_ids[mask]
        confidences = confidences[mask]
        
        # Convert coordinates
        h_orig, w_orig = frame_shape[:2]
        scale_x = w_orig / 640
        scale_y = h_orig / 640
        
        cx = boxes[:, 0] * scale_x
        cy = boxes[:, 1] * scale_y
        w = boxes[:, 2] * scale_x
        h = boxes[:, 3] * scale_y
        
        x1 = np.clip(cx - w/2, 0, w_orig).astype(np.int32)
        y1 = np.clip(cy - h/2, 0, h_orig).astype(np.int32)
        x2 = np.clip(cx + w/2, 0, w_orig).astype(np.int32)
        y2 = np.clip(cy + h/2, 0, h_orig).astype(np.int32)
        
        # Filter by area
        areas = (x2 - x1) * (y2 - y1)
        area_mask = (areas > self.min_box_area) & (areas < self.max_box_area)
        
        x1 = x1[area_mask]
        y1 = y1[area_mask]
        x2 = x2[area_mask]
        y2 = y2[area_mask]
        class_ids = class_ids[area_mask]
        confidences = confidences[area_mask]
        
        if len(x1) == 0:
            return []
        
        # NMS
        boxes_xyxy = np.column_stack([x1, y1, x2, y2])
        indices = cv2.dnn.NMSBoxes(
            boxes_xyxy.tolist(),
            confidences.tolist(),
            self.conf_threshold,
            self.iou_threshold
        )
        
        detections = []
        if len(indices) > 0:
            for idx in indices.flatten():
                # Model outputs 0-indexed, convert to 1-indexed for compatibility
                cls_id = int(class_ids[idx]) + 1
                
                # Filter by target color
                if cls_id not in self.target_ids:
                    continue
                
                detections.append({
                    'bbox': boxes_xyxy[idx],
                    'class_id': cls_id,
                    'confidence': float(confidences[idx]),
                    'class_name': self.CLASS_NAMES[cls_id]
                })
        
        detections.sort(key=lambda x: x['confidence'], reverse=True)
        return detections
    
    def publish_detections(self, detections):
        """Publish detection results
        
        Message format: [cone_index, x_coord, y_coord, color_id, ...]
        - cone_index: 1, 2, 3... (detection order)
        - x_coord, y_coord: center of bounding box in pixels
        - color_id: 1=yellow, 2=blue, 3=orange, 4=large_orange, 5=red, 6=green, 7=others
        - Publishes [0.0, 0.0, 0.0, 0.0] when no cones detected
        """
        msg = Float32MultiArray()
        
        if detections:
            data = []
            for idx, det in enumerate(detections, start=1):
                x1, y1, x2, y2 = det['bbox']
                cx = float((x1 + x2) / 2)
                cy = float((y1 + y2) / 2)
                cls_id = float(det['class_id'])  # Already 1-indexed
                
                # Format: [cone_index, x_coord, y_coord, color_id]
                data.extend([float(idx), cx, cy, cls_id])
            
            msg.data = data
            self.pub.publish(msg)
            
            # Log
            color_info = []
            for idx, det in enumerate(detections, start=1):
                x1, y1, x2, y2 = det['bbox']
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                color_info.append(f"Cone{idx}:{det['class_name']}({cx},{cy})")
            
            self.get_logger().info(f"Published: {', '.join(color_info)}")
        else:
            # No detections - publish zeros (0 reserved for "no detection")
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub.publish(msg)
    
    def draw_detections(self, frame, detections):
        """Draw visualizations"""
        annotated = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det['bbox']]
            cls_id = det['class_id']
            conf = det['confidence']
            name = det['class_name']
            
            color = self.CLASS_COLORS[cls_id]
            
            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            # Draw center
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            
            # Label
            label = f"{name} {conf:.3f}"
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(annotated, (x1, y1 - h - 10), (x1 + w + 10, y1), color, -1)
            cv2.putText(annotated, label, (x1 + 5, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return annotated
    
    def run(self):
        """Main loop"""
        # CHANGED: Open camera with device path (string or int)
        # OpenCV can handle both "/dev/video2" and device paths
        cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            self.get_logger().error(f"Cannot open camera: {self.camera_device}")
            return
        
        # Set camera properties for performance
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer for lower latency
        
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Camera resolution: {actual_width}x{actual_height}")
        
        try:
            while rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn("Failed to read frame")
                    continue
                
                self.frame_count += 1
                
                # Preprocess
                input_tensor = self.preprocess_frame(frame)
                
                # Inference
                t0 = time.perf_counter()
                
                if self.use_async:
                    self.infer_request.start_async({0: input_tensor})
                    self.infer_request.wait()
                else:
                    self.infer_request.infer({0: input_tensor})
                
                output = self.infer_request.get_output_tensor(0).data
                t1 = time.perf_counter()
                
                inference_ms = (t1 - t0) * 1000
                self.inference_times.append(inference_ms)
                
                # Post-process
                detections = self.postprocess_detections(output, frame.shape)
                
                # Apply stability filter
                if self.tracker:
                    self.tracker.update(detections)
                    final_detections = self.tracker.get_stable_detections()
                else:
                    final_detections = detections
                
                # Publish
                self.publish_detections(final_detections)
                
                # Visualization
                if self.show_visualization:
                    annotated = self.draw_detections(frame, final_detections)
                    
                    # Add FPS overlay
                    if len(self.inference_times) >= 10:
                        avg_fps = 1000.0 / np.mean(list(self.inference_times))
                        cv2.putText(annotated, f"FPS: {avg_fps:.1f}", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    cv2.imshow("OpenVINO Cone Detection", annotated)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
        except KeyboardInterrupt:
            self.get_logger().info("Stopped by user")
        finally:
            cap.release()
            if self.show_visualization:
                cv2.destroyAllWindows()
            
            if self.inference_times:
                avg_inf = np.mean(list(self.inference_times))
                avg_fps = 1000.0 / avg_inf
                self.get_logger().info(
                    f"\nPerformance Summary:\n"
                    f"  Total frames: {self.frame_count}\n"
                    f"  Avg inference: {avg_inf:.1f}ms\n"
                    f"  Avg FPS: {avg_fps:.1f}"
                )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OpenVINOConeDetectionNode()
        node.run()
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
