#!/usr/bin/env python3
"""
Rocket Telemetry Sensor Node
Reads real sensors and publishes telemetry via secure MQTT
"""

import json
import time
import ssl
import logging
from datetime import datetime, timezone
from typing import Dict, Any, Optional
import paho.mqtt.client as mqtt
from sensor_manager import SensorManager
import signal
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/rocket_telemetry.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

class RocketTelemetry:
    def __init__(self, config_file: str = "config.json"):
        """Initialize rocket telemetry system"""
        self.config = self._load_config(config_file)
        self.sensor_manager = SensorManager()
        self.mqtt_client = None
        self.running = False
        self.flight_state = "pre_flight"
        self.mission_time = 0.0
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        logger.info("Rocket telemetry system initialized")
    
    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """Load configuration from JSON file"""
        try:
            with open(config_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
            return self._default_config()
    
    def _default_config(self) -> Dict[str, Any]:
        """Default configuration"""
        return {
            "mqtt": {
                "broker": "localhost",
                "port": 8883,
                "username": "rocket_node",
                "password": "secure_password",
                "topic_prefix": "rocket/telemetry",
                "ca_cert": "/etc/ssl/certs/ca.crt",
                "cert_file": "/etc/ssl/certs/client.crt",
                "key_file": "/etc/ssl/private/client.key"
            },
            "sensors": {
                "sample_rate": 10,  # Hz
                "altitude_offset": 0.0,
                "imu_calibration": {
                    "accel_bias": [0.0, 0.0, 0.0],
                    "gyro_bias": [0.0, 0.0, 0.0]
                }
            },
            "flight": {
                "launch_accel_threshold": 2.0,  # g
                "apogee_velocity_threshold": 1.0,  # m/s
                "landing_accel_threshold": 0.5   # g
            }
        }
    
    def _setup_mqtt(self):
        """Setup secure MQTT connection"""
        self.mqtt_client = mqtt.Client(client_id=f"rocket_node_{int(time.time())}")
        
        # Setup TLS
        context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
        context.load_verify_locations(self.config["mqtt"]["ca_cert"])
        context.load_cert_chain(
            self.config["mqtt"]["cert_file"],
            self.config["mqtt"]["key_file"]
        )
        
        self.mqtt_client.tls_set_context(context)
        self.mqtt_client.username_pw_set(
            self.config["mqtt"]["username"],
            self.config["mqtt"]["password"]
        )
        
        # Setup callbacks
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        try:
            self.mqtt_client.connect(
                self.config["mqtt"]["broker"],
                self.config["mqtt"]["port"],
                60
            )
            self.mqtt_client.loop_start()
            logger.info("MQTT client connected")
        except Exception as e:
            logger.error(f"Failed to connect to MQTT broker: {e}")
            raise
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            # Subscribe to control topics
            control_topic = f"{self.config['mqtt']['topic_prefix']}/control/+"
            client.subscribe(control_topic)
            logger.info(f"Subscribed to {control_topic}")
        else:
            logger.error(f"Failed to connect to MQTT broker: {rc}")
    
    def _on_mqtt_disconnect(self, client, userdata, rc):
        """MQTT disconnection callback"""
        logger.warning(f"Disconnected from MQTT broker: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            topic_parts = msg.topic.split('/')
            if len(topic_parts) >= 3 and topic_parts[-2] == "control":
                command = topic_parts[-1]
                payload = json.loads(msg.payload.decode())
                self._handle_control_command(command, payload)
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")
    
    def _handle_control_command(self, command: str, payload: Dict[str, Any]):
        """Handle control commands from ground station"""
        logger.info(f"Received control command: {command}")
        
        if command == "mode":
            mode = payload.get("mode", "live")
            if mode in ["live", "sim", "test"]:
                self.sensor_manager.set_mode(mode)
                logger.info(f"Switched to {mode} mode")
        
        elif command == "calibrate":
            self.sensor_manager.calibrate_sensors()
            logger.info("Sensor calibration initiated")
        
        elif command == "reset":
            self.mission_time = 0.0
            self.flight_state = "pre_flight"
            logger.info("Mission reset")
    
    def _collect_telemetry(self) -> Dict[str, Any]:
        """Collect telemetry data from sensors"""
        try:
            sensor_data = self.sensor_manager.read_all_sensors()
            
            # Calculate derived values
            altitude = sensor_data["pressure"]["altitude"] - self.config["sensors"]["altitude_offset"]
            velocity = self._calculate_velocity(sensor_data["imu"]["accel_z"])
            
            # Update flight state
            self._update_flight_state(sensor_data["imu"]["accel_total"], velocity)
            
            telemetry = {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "mission_time": self.mission_time,
                "flight_state": self.flight_state,
                "altitude": round(altitude, 2),
                "velocity": round(velocity, 2),
                "acceleration": {
                    "x": round(sensor_data["imu"]["accel_x"], 3),
                    "y": round(sensor_data["imu"]["accel_y"], 3),
                    "z": round(sensor_data["imu"]["accel_z"], 3),
                    "total": round(sensor_data["imu"]["accel_total"], 3)
                },
                "orientation": {
                    "roll": round(sensor_data["imu"]["roll"], 2),
                    "pitch": round(sensor_data["imu"]["pitch"], 2),
                    "yaw": round(sensor_data["imu"]["yaw"], 2)
                },
                "environmental": {
                    "pressure": round(sensor_data["pressure"]["pressure"], 2),
                    "temperature": round(sensor_data["pressure"]["temperature"], 2),
                    "humidity": round(sensor_data["pressure"]["humidity"], 2)
                },
                "system": {
                    "battery_voltage": sensor_data["system"]["battery_voltage"],
                    "cpu_temp": sensor_data["system"]["cpu_temp"],
                    "memory_usage": sensor_data["system"]["memory_usage"]
                }
            }
            
            return telemetry
            
        except Exception as e:
            logger.error(f"Error collecting telemetry: {e}")
            return self._create_error_telemetry(str(e))
    
    def _calculate_velocity(self, accel_z: float) -> float:
        """Calculate velocity from acceleration (simple integration)"""
        # This is a simplified calculation - in real application,
        # you'd want more sophisticated filtering and integration
        dt = 1.0 / self.config["sensors"]["sample_rate"]
        if not hasattr(self, '_last_velocity'):
            self._last_velocity = 0.0
        
        self._last_velocity += accel_z * dt * 9.81  # Convert g to m/s²
        return self._last_velocity
    
    def _update_flight_state(self, accel_total: float, velocity: float):
        """Update flight state based on sensor data"""
        if self.flight_state == "pre_flight":
            if accel_total > self.config["flight"]["launch_accel_threshold"]:
                self.flight_state = "boost"
                logger.info("Launch detected - entering boost phase")
        
        elif self.flight_state == "boost":
            if accel_total < 1.2:  # Low acceleration indicates motor burnout
                self.flight_state = "coast"
                logger.info("Motor burnout - entering coast phase")
        
        elif self.flight_state == "coast":
            if abs(velocity) < self.config["flight"]["apogee_velocity_threshold"]:
                self.flight_state = "apogee"
                logger.info("Apogee detected")
        
        elif self.flight_state == "apogee":
            if velocity < -2.0:  # Significant downward velocity
                self.flight_state = "descent"
                logger.info("Descent phase started")
        
        elif self.flight_state == "descent":
            if (accel_total < self.config["flight"]["landing_accel_threshold"] and 
                abs(velocity) < 1.0):
                self.flight_state = "landed"
                logger.info("Landing detected")
    
    def _create_error_telemetry(self, error_msg: str) -> Dict[str, Any]:
        """Create error telemetry packet"""
        return {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "mission_time": self.mission_time,
            "flight_state": "error",
            "error": error_msg,
            "altitude": 0.0,
            "velocity": 0.0,
            "acceleration": {"x": 0, "y": 0, "z": 0, "total": 0},
            "orientation": {"roll": 0, "pitch": 0, "yaw": 0},
            "environmental": {"pressure": 0, "temperature": 0, "humidity": 0},
            "system": {"battery_voltage": 0, "cpu_temp": 0, "memory_usage": 0}
        }
    
    def _publish_telemetry(self, telemetry: Dict[str, Any]):
        """Publish telemetry data via MQTT"""
        if self.mqtt_client and self.mqtt_client.is_connected():
            topic = f"{self.config['mqtt']['topic_prefix']}/data"
            payload = json.dumps(telemetry)
            
            result = self.mqtt_client.publish(topic, payload, qos=1)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                logger.error(f"Failed to publish telemetry: {result.rc}")
        else:
            logger.warning("MQTT client not connected - telemetry not published")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        logger.info(f"Received signal {signum} - shutting down")
        self.stop()
    
    def start(self):
        """Start telemetry collection and transmission"""
        logger.info("Starting rocket telemetry system")
        
        try:
            # Initialize sensors
            self.sensor_manager.initialize()
            
            # Setup MQTT
            self._setup_mqtt()
            
            # Main telemetry loop
            self.running = True
            sample_interval = 1.0 / self.config["sensors"]["sample_rate"]
            
            while self.running:
                start_time = time.time()
                
                # Collect and publish telemetry
                telemetry = self._collect_telemetry()
                self._publish_telemetry(telemetry)
                
                # Update mission time
                self.mission_time = time.time() - getattr(self, '_start_time', time.time())
                if not hasattr(self, '_start_time'):
                    self._start_time = time.time()
                
                # Maintain sample rate
                elapsed = time.time() - start_time
                if elapsed < sample_interval:
                    time.sleep(sample_interval - elapsed)
                
        except Exception as e:
            logger.error(f"Error in telemetry loop: {e}")
            raise
        finally:
            self.stop()
    
    def stop(self):
        """Stop telemetry system"""
        logger.info("Stopping rocket telemetry system")
        self.running = False
        
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        self.sensor_manager.cleanup()

if __name__ == "__main__":
    try:
        telemetry = RocketTelemetry()
        telemetry.start()
    except KeyboardInterrupt:
        logger.info("Telemetry system stopped by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)
