from flask import Flask, Response, render_template_string, jsonify
import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import numpy as np

app = Flask(__name__)

# Camera configuration - Fixed paths for specific cameras
CAMERA_PATHS = {
    'lenovo': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0',  # USB Port 2
    'logitech': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0',   # USB Port 3
    'black': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0',     # USB Port 6
    'logiblack': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:7.2:1.0-video-index0' 
}

# Camera task/purpose descriptions (optional)
CAMERA_DESCRIPTIONS = {
    'lenovo': 'Lenovo Camera',
    'logitech': 'Logitech Camera',
    'black': 'Black Camera',
    'logiblack' : 'Logitech_C270'
}

# Initialize cameras with stable paths
cameras = {}
camera_names = []

for name, path in CAMERA_PATHS.items():
    try:
        cam = cv2.VideoCapture(path)
        if cam.isOpened():
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cam.set(cv2.CAP_PROP_FPS, 30)
            cameras[name] = cam
            camera_names.append(name)
            print(f"✓ Camera '{name}' initialized: {path}")
        else:
            print(f"✗ Failed to open camera '{name}': {path}")
    except Exception as e:
        print(f"✗ Error initializing camera '{name}': {e}")

# GPS data storage
gps_data = {
    'latitude': float('nan'),
    'longitude': float('nan'),
    'altitude': float('nan'),
    'position_covariance': [float('nan')] * 9,
    'position_covariance_type': 0,
    'status': 'No Data'
}

# IMU data storage
imu_data = {
    'value': 0
}

# Lenovo camera mode (False = normal, True = 3:1 aspect ratio with overlay)
lenovo_overlay_mode = {'enabled': False}

def get_cardinal_direction(yaw_degrees):
    """Convert yaw angle (degrees from north) to cardinal direction"""
    # Normalize yaw to 0-360 range
    yaw = yaw_degrees % 360
    
    # Define direction ranges (22.5 degrees each side of cardinal directions)
    directions = [
        (0, 22.5, "N"),
        (22.5, 67.5, "NE"),
        (67.5, 112.5, "E"),
        (112.5, 157.5, "SE"),
        (157.5, 202.5, "S"),
        (202.5, 247.5, "SW"),
        (247.5, 292.5, "W"),
        (292.5, 337.5, "NW"),
        (337.5, 360, "N")
    ]
    
    for min_angle, max_angle, direction in directions:
        if min_angle <= yaw < max_angle:
            return direction
    
    return "N"  # Default to North

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Int32,
            '/imu_data',
            self.imu_callback,
            10)
        
    def gps_callback(self, msg):
        global gps_data
        gps_data['latitude'] = msg.latitude
        gps_data['longitude'] = msg.longitude
        gps_data['altitude'] = msg.altitude
        gps_data['position_covariance'] = list(msg.position_covariance)
        gps_data['position_covariance_type'] = msg.position_covariance_type
        gps_data['status'] = 'Active'
    
    def imu_callback(self, msg):
        global imu_data
        imu_data['value'] = msg.data

def ros2_spin():
    """Run ROS2 in a separate thread"""
    rclpy.init()
    data_node = DataSubscriber()
    rclpy.spin(data_node)
    data_node.destroy_node()
    rclpy.shutdown()

def add_overlay_text(frame, text, position, font_scale=0.4, thickness=1):
    """Add text with background to frame"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    x, y = position
    # Draw background rectangle
    cv2.rectangle(frame, (x - 5, y - text_size[1] - 5), 
                  (x + text_size[0] + 5, y + 5), (0, 0, 0), -1)
    # Draw text
    cv2.putText(frame, text, (x, y), font, font_scale, (255, 255, 255), thickness)

def generate_frames(camera_name):
    """Generate frames for the specified camera"""
    if camera_name not in cameras:
        return
    
    cam = cameras[camera_name]
    while True:
        success, frame = cam.read()
        if not success:
            break
        
        # Special handling for lenovo camera with overlay mode
        if camera_name == 'lenovo' and lenovo_overlay_mode['enabled']:
            # Crop to 3:1 aspect ratio (width:height = 3:1)
            # Original frame is 640x480
            # For 3:1 ratio, we want width = 3 * height
            # So if we use full width (640), height should be 640/3 ≈ 213
            target_height = 213
            start_y = (frame.shape[0] - target_height) // 2
            frame = frame[start_y:start_y + target_height, :]
            
            # Add overlay text
            lat_val = gps_data['latitude']
            lon_val = gps_data['longitude']
            alt_val = gps_data['altitude']
            imu_val = imu_data['value']
            direction = get_cardinal_direction(imu_val)
            
            # Format values
            lat_str = f"Lat: {lat_val:.6f}" if not (np.isnan(lat_val) or not np.isfinite(lat_val)) else "Lat: nan"
            lon_str = f"Lon: {lon_val:.6f}" if not (np.isnan(lon_val) or not np.isfinite(lon_val)) else "Lon: nan"
            alt_str = f"Alt: {alt_val:.2f}m" if not (np.isnan(alt_val) or not np.isfinite(alt_val)) else "Alt: nan"
            imu_str = f"Yaw: {int(imu_val)} degrees ({direction})"
            
            # Add text overlays
            add_overlay_text(frame, imu_str, (10, 30))
            add_overlay_text(frame, lat_str, (10, 60))
            add_overlay_text(frame, lon_str, (10, 90))
            add_overlay_text(frame, alt_str, (10, 120))
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
        ret, buffer = cv2.imencode('.jpg', frame, encode_param)
        frame = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    html = """
    <html>
    <head>
        <title>Multi-Camera Stream</title>
        <style>
            body { 
                background: #111; 
                color: #fff; 
                font-family: Arial;
                margin: 0;
                padding: 20px;
                display: flex;
                gap: 20px;
            }
            h2 {
                color: #3498db;
                margin-bottom: 30px;
                text-align: center;
            }
            .main-container {
                display: flex;
                gap: 20px;
                width: 100%;
            }
            .cam-section {
                flex: 1;
            }
            .cam-container { 
                display: flex; 
                justify-content: center; 
                flex-wrap: wrap;
                gap: 20px;
            }
            .cam-box { 
                margin: 10px; 
                background: #222; 
                padding: 15px; 
                border-radius: 10px;
                box-shadow: 0 4px 8px rgba(0,0,0,0.3);
            }
            .cam-title {
                color: #3498db;
                margin-bottom: 10px;
                text-transform: capitalize;
                text-align: center;
            }
            .cam-port {
                color: #888;
                font-size: 12px;
                margin-bottom: 10px;
                text-align: center;
            }
            img { 
                border-radius: 8px;
                border: 2px solid #444;
                display: block;
            }
            .normal-cam {
                width: 320px;
                height: 240px;
            }
            .overlay-cam {
                width: 640px;
                height: 213px;
            }
            .sidebar {
                width: 350px;
                flex-shrink: 0;
            }
            .gps-container, .imu-container {
                background: #222;
                padding: 15px;
                border-radius: 10px;
                box-shadow: 0 4px 8px rgba(0,0,0,0.3);
                position: sticky;
                top: 20px;
                margin-bottom: 20px;
            }
            .gps-title, .imu-title {
                color: #3498db;
                margin-bottom: 15px;
                margin-top: 0;
            }
            .gps-data, .imu-data {
                color: #fff;
                font-size: 14px;
                text-align: left;
                line-height: 1.8;
            }
            .gps-label, .imu-label {
                color: #888;
                display: inline-block;
                width: 150px;
            }
            .covariance {
                font-size: 11px;
                color: #aaa;
                margin-top: 5px;
                word-break: break-all;
            }
            .info-text {
                color: #888;
                font-size: 12px;
                text-align: center;
                margin-top: 10px;
                font-style: italic;
            }
            .direction-badge {
                display: inline-block;
                padding: 2px 8px;
                background: #3498db;
                border-radius: 4px;
                font-weight: bold;
                margin-left: 5px;
            }
        </style>
        <script>
            let overlayMode = false;
            
            function formatValue(value) {
                if (isNaN(value) || !isFinite(value)) {
                    return 'nan';
                }
                return value;
            }
            
            function getCardinalDirection(yaw) {
                const normalized = ((yaw % 360) + 360) % 360;
                const directions = [
                    { max: 22.5, dir: 'N' },
                    { max: 67.5, dir: 'NE' },
                    { max: 112.5, dir: 'E' },
                    { max: 157.5, dir: 'SE' },
                    { max: 202.5, dir: 'S' },
                    { max: 247.5, dir: 'SW' },
                    { max: 292.5, dir: 'W' },
                    { max: 337.5, dir: 'NW' },
                    { max: 360, dir: 'N' }
                ];
                
                for (let i = 0; i < directions.length; i++) {
                    if (normalized < directions[i].max) {
                        return directions[i].dir;
                    }
                }
                return 'N';
            }
            
            function updateData() {
                fetch('/gps_data')
                    .then(response => response.json())
                    .then(data => {
                        let lat = formatValue(data.latitude);
                        let lon = formatValue(data.longitude);
                        let alt = formatValue(data.altitude);
                        
                        document.getElementById('latitude').textContent = 
                            lat === 'nan' ? 'nan' : lat.toFixed(6);
                        document.getElementById('longitude').textContent = 
                            lon === 'nan' ? 'nan' : lon.toFixed(6);
                        document.getElementById('altitude').textContent = 
                            alt === 'nan' ? 'nan' : alt.toFixed(2);
                        document.getElementById('status').textContent = data.status;
                        document.getElementById('cov_type').textContent = data.position_covariance_type;
                        
                        let covStr = data.position_covariance.map(v => {
                            if (isNaN(v) || !isFinite(v)) return 'nan';
                            return v.toFixed(3);
                        }).join(', ');
                        document.getElementById('covariance').textContent = '[' + covStr + ']';
                    })
                    .catch(error => console.error('Error:', error));
                
                fetch('/imu_data')
                    .then(response => response.json())
                    .then(data => {
                        let yawValue = data.value;
                        let direction = getCardinalDirection(yawValue);
                        document.getElementById('imu_value').textContent = yawValue + '°';
                        document.getElementById('imu_direction').textContent = direction;
                    })
                    .catch(error => console.error('Error:', error));
            }
            
            function toggleOverlayMode() {
                overlayMode = !overlayMode;
                fetch('/toggle_overlay', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({enabled: overlayMode})
                }).then(() => {
                    // Update image class
                    let lenovoImg = document.querySelector('.lenovo-cam');
                    if (overlayMode) {
                        lenovoImg.classList.remove('normal-cam');
                        lenovoImg.classList.add('overlay-cam');
                    } else {
                        lenovoImg.classList.remove('overlay-cam');
                        lenovoImg.classList.add('normal-cam');
                    }
                    // Force reload the image
                    lenovoImg.src = '/video/lenovo?' + new Date().getTime();
                });
            }
            
            document.addEventListener('keydown', function(event) {
                if (event.key === 'w' || event.key === 'W') {
                    toggleOverlayMode();
                }
            });
            
            setInterval(updateData, 1000);
            window.onload = updateData;
        </script>
    </head>
    <body>
        <div class="main-container">
            <div class="cam-section">
                <h2>Multi-Camera Stream</h2>
                <p class="info-text">Press 'W' to toggle Lenovo camera overlay mode (3:1 aspect ratio)</p>
                <div class="cam-container">
                    {% for cam_name in cam_names %}
                        <div class="cam-box">
                            <h3 class="cam-title">{{ descriptions[cam_name] }}</h3>
                            <p class="cam-port">{{ cam_name }}</p>
                            <img src="/video/{{ cam_name }}" class="{{ cam_name }}-cam normal-cam">
                        </div>
                    {% endfor %}
                </div>
            </div>
            <div class="sidebar">
                <div class="imu-container">
                    <h3 class="imu-title">IMU Data (Yaw from North)</h3>
                    <div class="imu-data">
                        <div><span class="imu-label">Value:</span> <span id="imu_value">0°</span></div>
                        <div><span class="imu-label">Direction:</span> <span id="imu_direction" class="direction-badge">N</span></div>
                    </div>
                </div>
                <div class="gps-container">
                    <h3 class="gps-title">GPS Data</h3>
                    <div class="gps-data">
                        <div><span class="gps-label">Latitude:</span> <span id="latitude">nan</span>°</div>
                        <div><span class="gps-label">Longitude:</span> <span id="longitude">nan</span>°</div>
                        <div><span class="gps-label">Altitude:</span> <span id="altitude">nan</span> m</div>
                        <div><span class="gps-label">Status:</span> <span id="status">No Data</span></div>
                        <div><span class="gps-label">Covariance Type:</span> <span id="cov_type">0</span></div>
                        <div><span class="gps-label">Position Covariance:</span></div>
                        <div class="covariance" id="covariance">[nan, nan, nan, nan, nan, nan, nan, nan, nan]</div>
                    </div>
                </div>
            </div>
        </div>
    </body>
    </html>
    """
    return render_template_string(html, cam_names=camera_names, descriptions=CAMERA_DESCRIPTIONS)

@app.route('/video/<camera_name>')
def video(camera_name):
    """Video streaming route"""
    if camera_name not in cameras:
        return "Camera not found", 404
    return Response(generate_frames(camera_name),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/gps_data')
def get_gps_data():
    """Return current GPS data as JSON"""
    return jsonify(gps_data)

@app.route('/imu_data')
def get_imu_data():
    """Return current IMU data as JSON"""
    return jsonify(imu_data)

@app.route('/toggle_overlay', methods=['POST'])
def toggle_overlay():
    """Toggle overlay mode for lenovo camera"""
    from flask import request
    data = request.get_json()
    lenovo_overlay_mode['enabled'] = data.get('enabled', False)
    return jsonify({'success': True, 'enabled': lenovo_overlay_mode['enabled']})

def cleanup_cameras():
    """Release camera resources"""
    for cam in cameras.values():
        cam.release()

def main():
    print("Starting Flask multi-camera stream server...")
    print(f"Initialized cameras: {', '.join(camera_names)}")
    
    # Start ROS2 subscriber in separate thread
    ros_thread = threading.Thread(target=ros2_spin, daemon=True)
    ros_thread.start()
    print("GPS subscriber started on topic /fix")
    print("IMU subscriber started on topic /imu_data")
    
    print("Access at: http://0.0.0.0:5000")
    print("Press 'W' key on the webpage to toggle Lenovo camera overlay mode (3:1 aspect ratio)")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        cleanup_cameras()