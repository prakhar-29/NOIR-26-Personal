import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import json
from datetime import datetime
from flask import Flask, Response, render_template, jsonify, request, render_template_string
import time

app = Flask(__name__)

# Global sensor data dictionary - removed TCS34725
sensor_data = {
    'mq135': 0.0,          # Raw analog value
    'temperature': 0.0,
    'humidity': 0.0,
    'pressure': 0.0,
    
    # NPK Sensor Data
    'soil_moisture': 0.0,      # %
    'soil_temperature': 0.0,   # °C
    'soil_conductivity': 0.0,  # us/cm
    'soil_ph': 0.0,           # pH
    'nitrogen': 0.0,          # mg/kg
    'phosphorus': 0.0,        # mg/kg
    'potassium': 0.0,         # mg/kg
    
    # Sensor status (0=off, 1=on)
    'mq135_status': 0,
    'bme280_status': 0,
    'npk_status': 0,
    
    'last_update': datetime.now().isoformat(),
    'connected': False,
    
    # Calculated values
    'co_ppm': 0.0,
    'co2_ppm': 0.0,
    'nh3_ppm': 0.0,
    'nox_ppm': 0.0,
    'altitude': 0.0
}

class MicroROSNode(Node):
    def __init__(self):
        super().__init__('flask_sensor_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'sensor_data_live',
            self.sensor_callback,
            10
        )
        self.get_logger().info('MicroROS Flask Subscriber Started')
        
    def sensor_callback(self, msg):
        """Callback for MicroROS sensor data - Removed TCS34725"""
        global sensor_data
        
        # Update timestamp
        sensor_data['last_update'] = datetime.now().isoformat()
        sensor_data['connected'] = True
        
        # Extract data from Float32MultiArray - 11 values expected
        if len(msg.data) >= 11:
            sensor_data['mq135'] = float(msg.data[0])
            sensor_data['temperature'] = float(msg.data[1])
            sensor_data['humidity'] = float(msg.data[2])
            sensor_data['pressure'] = float(msg.data[3])
            sensor_data['soil_moisture'] = float(msg.data[4])
            sensor_data['soil_temperature'] = float(msg.data[5])
            sensor_data['soil_conductivity'] = float(msg.data[6])
            sensor_data['soil_ph'] = float(msg.data[7])
            sensor_data['nitrogen'] = float(msg.data[8])
            sensor_data['phosphorus'] = float(msg.data[9])
            sensor_data['potassium'] = float(msg.data[10])
            
            # Update sensor status
            self.update_sensor_status()
            
            # Calculate derived values
            self.calculate_derived_values()
            
            # Print for debugging
            self.get_logger().info(f"Received: Temp={sensor_data['temperature']:.1f}°C, "
                                  f"NPK: N={sensor_data['nitrogen']} P={sensor_data['phosphorus']} K={sensor_data['potassium']}")
    
    def update_sensor_status(self):
        """Update sensor status based on readings"""
        global sensor_data
        
        sensor_data['mq135_status'] = 1 if sensor_data['mq135'] > 50 else 0
        sensor_data['bme280_status'] = 1 if sensor_data['temperature'] != 0 else 0
        
        # NPK sensor status
        npk_active = (
            sensor_data['soil_moisture'] > 0 or
            sensor_data['nitrogen'] > 0 or
            sensor_data['phosphorus'] > 0 or
            sensor_data['potassium'] > 0
        )
        sensor_data['npk_status'] = 1 if npk_active else 0
    
    def calculate_derived_values(self):
        """Calculate derived sensor values"""
        global sensor_data
        
        # Convert MQ135 raw value to gas concentrations
        raw_value = sensor_data['mq135']
        
        # Example calculations
        sensor_data['co_ppm'] = raw_value * 0.1
        sensor_data['co2_ppm'] = raw_value * 0.05
        sensor_data['nh3_ppm'] = raw_value * 0.08
        sensor_data['nox_ppm'] = raw_value * 0.06
        
        # Calculate altitude from pressure
        if sensor_data['pressure'] > 0:
            sea_level_pressure = 1013.25
            temperature_k = sensor_data['temperature'] + 273.15
            #the following formula requires the temp to be kelvin!!
            sensor_data['altitude'] = ((pow(sea_level_pressure / sensor_data['pressure'], 1/5.257) - 1) * temperature_k) / 0.0065

# Global ROS2 node
ros_node = None
ros_thread = None

def init_ros():
    """Initialize ROS2 node in a separate thread"""
    global ros_node
    
    rclpy.init()
    ros_node = MicroROSNode()
    
    # Spin ROS node
    rclpy.spin(ros_node)
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()

# Flask Routes
@app.route('/')
def index():
    """Render the main dashboard"""
    return render_template('index.html')

@app.route('/api/data')
def get_sensor_data():
    """API endpoint to get current sensor data"""
    return jsonify(sensor_data)

@app.route('/api/bme280')
def get_bme280_data():
    """API endpoint for BME280 sensor data"""
    bme_data = {
        'temperature': sensor_data['temperature'],
        'humidity': sensor_data['humidity'],
        'pressure': sensor_data['pressure'],
        'altitude': sensor_data['altitude'],
        'bme280_status': sensor_data['bme280_status'],
        'last_update': sensor_data['last_update'],
        'connected': sensor_data['connected']
    }
    return jsonify(bme_data)

@app.route('/api/mq135')
def get_mq135_data():
    """API endpoint for MQ135 sensor data"""
    mq135_data = {
        'raw_value': sensor_data['mq135'],
        'co_ppm': sensor_data['co_ppm'],
        'co2_ppm': sensor_data['co2_ppm'],
        'nh3_ppm': sensor_data['nh3_ppm'],
        'nox_ppm': sensor_data['nox_ppm'],
        'mq135_status': sensor_data['mq135_status'],
        'last_update': sensor_data['last_update'],
        'connected': sensor_data['connected']
    }
    return jsonify(mq135_data)

@app.route('/api/npk')
def get_npk_data():
    """API endpoint for NPK sensor data"""
    npk_data = {
        'soil_moisture': sensor_data['soil_moisture'],
        'soil_temperature': sensor_data['soil_temperature'],
        'soil_conductivity': sensor_data['soil_conductivity'],
        'soil_ph': sensor_data['soil_ph'],
        'nitrogen': sensor_data['nitrogen'],
        'phosphorus': sensor_data['phosphorus'],
        'potassium': sensor_data['potassium'],
        'npk_status': sensor_data['npk_status'],
        'last_update': sensor_data['last_update'],
        'connected': sensor_data['connected']
    }
    return jsonify(npk_data)

@app.route('/api/status')
def get_status():
    """API endpoint for overall system status"""
    status = {
        'connected': sensor_data['connected'],
        'last_update': sensor_data['last_update'],
        'sensor_status': {
            'mq135': sensor_data['mq135_status'],
            'bme280': sensor_data['bme280_status'],
            'npk': sensor_data['npk_status']
        },
        'ros_status': ros_node is not None
    }
    return jsonify(status)

@app.route('/api/raw')
def get_raw_data():
    """API endpoint for raw sensor data"""
    raw_data = {
        'mq135_raw': sensor_data['mq135'],
        'temperature': sensor_data['temperature'],
        'humidity': sensor_data['humidity'],
        'pressure': sensor_data['pressure'],
        'soil_moisture': sensor_data['soil_moisture'],
        'soil_temperature': sensor_data['soil_temperature'],
        'soil_conductivity': sensor_data['soil_conductivity'],
        'soil_ph': sensor_data['soil_ph'],
        'nitrogen': sensor_data['nitrogen'],
        'phosphorus': sensor_data['phosphorus'],
        'potassium': sensor_data['potassium']
    }
    return jsonify(raw_data)

@app.route('/api/all_sensors')
def get_all_sensors():
    """API endpoint for comprehensive sensor data"""
    all_data = {
        'environment': {
            'temperature': sensor_data['temperature'],
            'humidity': sensor_data['humidity'],
            'pressure': sensor_data['pressure'],
            'altitude': sensor_data['altitude']
        },
        'air_quality': {
            'co_ppm': sensor_data['co_ppm'],
            'co2_ppm': sensor_data['co2_ppm'],
            'nh3_ppm': sensor_data['nh3_ppm'],
            'nox_ppm': sensor_data['nox_ppm']
        },
        'soil_analysis': {
            'moisture': sensor_data['soil_moisture'],
            'temperature': sensor_data['soil_temperature'],
            'conductivity': sensor_data['soil_conductivity'],
            'ph': sensor_data['soil_ph'],
            'nitrogen': sensor_data['nitrogen'],
            'phosphorus': sensor_data['phosphorus'],
            'potassium': sensor_data['potassium']
        },
        'status': {
            'connected': sensor_data['connected'],
            'last_update': sensor_data['last_update'],
            'mq135': sensor_data['mq135_status'],
            'bme280': sensor_data['bme280_status'],
            'npk': sensor_data['npk_status']
        }
    }
    return jsonify(all_data)

if __name__ == '__main__':
    # Start ROS2 in a separate thread
    ros_thread = threading.Thread(target=init_ros)
    ros_thread.daemon = True
    ros_thread.start()
    
    print("ROS2 subscriber started in background thread")
    print("Starting Flask web server...")
    print("Sensor Dashboard available at: http://localhost:5002")
    print("Available API endpoints:")
    print("  /api/data        - All sensor data")
    print("  /api/npk         - NPK sensor data")
    print("  /api/all_sensors - Comprehensive data")
    print("  /api/status      - System status")
    print("  /api/bme280      - BME280 data")
    print("  /api/mq135       - MQ135 data")
    
    # Run Flask app
    app.run(host='0.0.0.0', port=5002, debug=True, threaded=True, use_reloader=False)