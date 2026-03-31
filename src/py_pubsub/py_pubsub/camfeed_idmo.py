from flask import Flask, Response, render_template_string
import cv2

app = Flask(__name__)

# Camera configuration - Fixed paths for specific cameras
CAMERA_PATHS = {
    'lenovo': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:2:1.0-video-index0',  # USB Port 2
    'logitech': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0',   # USB Port 3
    'black': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:6:1.0-video-index0' ,    # USB Port 6
    'logiblack': '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:7.2:1.0-video-index0' ,
}

# Camera task/purpose descriptions (optional)
CAMERA_DESCRIPTIONS = {
    'lenovo': 'Lenovo Camera',
    'logitech': 'Logitech Camera',
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
                text-align: center; 
                background: #111; 
                color: #fff; 
                font-family: Arial;
                margin: 0;
                padding: 20px;
            }
            h2 {
                color: #3498db;
                margin-bottom: 30px;
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
            }
            .cam-port {
                color: #888;
                font-size: 12px;
                margin-bottom: 10px;
            }
            img { 
                width: 320px;
                height: 240px;
                border-radius: 8px;
                border: 2px solid #444;
            }
        </style>
    </head>
    <body>
        <h2>Multi-Camera Stream</h2>
        <div class="cam-container">
            {% for cam_name in cam_names %}
                <div class="cam-box">
                    <h3 class="cam-title">{{ descriptions[cam_name] }}</h3>
                    <p class="cam-port">{{ cam_name }}</p>
                    <img src="/video/{{ cam_name }}">
                </div>
            {% endfor %}
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

def cleanup_cameras():
    """Release camera resources"""
    for cam in cameras.values():
        cam.release()

def main():
    print("Starting Flask multi-camera stream server...")
    print(f"Initialized cameras: {', '.join(camera_names)}")
    print("Access at: http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        cleanup_cameras()