from flask import Flask, Response, render_template_string, jsonify
import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

app = Flask(__name__)

# Camera configuration - Fixed paths for specific cameras
CAMERA_PATHS = {
    'lenovo': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0',  # USB Port 2
    #'logitech': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0',   # USB Port 
    'black': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0' ,    # USB Port 6
    'logiblack': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:7.2:1.0-video-index0' 
}

# Camera task/purpose descriptions (optional)
CAMERA_DESCRIPTIONS = {
    'lenovo': 'Lenovo Camera',
    'black': 'Black Camera',
    'logiblack': 'logiBlack C270',

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

class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10)
        
    def gps_callback(self, msg):
        global gps_data
        gps_data['latitude'] = msg.latitude
        gps_data['longitude'] = msg.longitude
        gps_data['altitude'] = msg.altitude
        gps_data['position_covariance'] = list(msg.position_covariance)
        gps_data['position_covariance_type'] = msg.position_covariance_type
        gps_data['status'] = 'Active'

def ros2_spin():
    """Run ROS2 in a separate thread"""
    rclpy.init()
    gps_node = GPSSubscriber()
    rclpy.spin(gps_node)
    gps_node.destroy_node()
    rclpy.shutdown()

def generate_frames(camera_name):
    """Generate frames for the specified camera"""
    if camera_name not in cameras:
        return
    
    cam = cameras[camera_name]
    while True:
        success, frame = cam.read()
        if not success:
            break
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
                width: 320px;
                height: 240px;
                border-radius: 8px;
                border: 2px solid #444;
                display: block;
            }
            .sidebar {
                width: 400px;
                flex-shrink: 0;
            }
            .gps-container {
                background: #222;
                padding: 20px;
                border-radius: 10px;
                box-shadow: 0 4px 8px rgba(0,0,0,0.3);
                position: sticky;
                top: 20px;
            }
            .gps-title {
                color: #3498db;
                margin-bottom: 20px;
                margin-top: 0;
                font-size: 20px;
            }
            .gps-data {
                color: #fff;
                font-size: 16px;
                text-align: left;
                line-height: 2.2;
            }
            .gps-label {
                color: #888;
                display: inline-block;
                width: 170px;
                font-weight: bold;
            }
            .gps-value {
                color: #fff;
            }
            .covariance-section {
                margin-top: 15px;
                padding-top: 15px;
                border-top: 1px solid #444;
            }
            .covariance {
                font-size: 13px;
                color: #aaa;
                margin-top: 8px;
                word-break: break-all;
                line-height: 1.6;
            }
        </style>
        <script>
            function formatValue(value) {
                if (isNaN(value) || !isFinite(value)) {
                    return 'nan';
                }
                return value;
            }
            
            function updateGPS() {
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
            }
            setInterval(updateGPS, 1000);
            window.onload = updateGPS;
        </script>
    </head>
    <body>
        <div class="main-container">
            <div class="cam-section">
                <h2>Multi-Camera Stream </h2>
                <div class="cam-container">
                    {% for cam_name in cam_names %}
                        <div class="cam-box">
                            <h3 class="cam-title">{{ descriptions[cam_name] }}</h3>
                            <p class="cam-port">{{ cam_name }}</p>
                            <img src="/video/{{ cam_name }}">
                        </div>
                    {% endfor %}
                </div>
            </div>
            <div class="sidebar">
                <div class="gps-container">
                    <h3 class="gps-title">GPS Data</h3>
                    <div class="gps-data">
                        <div><span class="gps-label">Latitude:</span> <span class="gps-value" id="latitude">nan</span>°</div>
                        <div><span class="gps-label">Longitude:</span> <span class="gps-value" id="longitude">nan</span>°</div>
                        <div><span class="gps-label">Altitude:</span> <span class="gps-value" id="altitude">nan</span> m</div>
                        <div><span class="gps-label">Status:</span> <span class="gps-value" id="status">No Data</span></div>
                        <div class="covariance-section">
                            <div><span class="gps-label">Covariance Type:</span> <span class="gps-value" id="cov_type">0</span></div>
                            <div><span class="gps-label">Position Covariance:</span></div>
                            <div class="covariance" id="covariance">[nan, nan, nan, nan, nan, nan, nan, nan, nan]</div>
                        </div>
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

def cleanup_cameras():
    """Release camera resources"""
    for cam in cameras.values():
        cam.release()

def main():
    print("Starting Flask multi-camera stream server...")
    print(f"Initialized cameras: {', '.join(camera_names)}")
    
    # Start ROS2 GPS subscriber in separate thread
    ros_thread = threading.Thread(target=ros2_spin, daemon=True)
    ros_thread.start()
    print("GPS subscriber started on topic /fix")
    
    print("Access at: http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        cleanup_cameras()