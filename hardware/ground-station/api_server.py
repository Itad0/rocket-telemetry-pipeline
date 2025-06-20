#!/usr/bin/env python3
"""
Ground Station API Server
Provides REST API for controlling telemetry system and fault injection
"""

import json
import time
import ssl
import logging
import sys
import signal
import threading
from datetime import datetime, timezone
from typing import Dict, Any, List, Optional
from flask import Flask, request, jsonify, Response
from flask_httpauth import HTTPBasicAuth
import paho.mqtt.client as mqtt
from prometheus_client import Counter, Histogram, Gauge, generate_latest
from telemetry_processor import TelemetryProcessor
from hil_simulator import IntegratedHILSimulator
import config

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/api_server.log'),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

# Prometheus metrics
telemetry_packets_total = Counter('telemetry_packets_total', 'Total telemetry packets processed')
api_requests_total = Counter('api_requests_total', 'Total API requests', ['method', 'endpoint'])
telemetry_altitude = Gauge('telemetry_altitude_meters', 'Current altitude in meters')
telemetry_velocity = Gauge('telemetry_velocity_ms', 'Current velocity in m/s')
telemetry_acceleration = Gauge('telemetry_acceleration_g', 'Current acceleration in g')
active_faults = Gauge('active_faults_count', 'Number of active faults')
api_response_time = Histogram('api_response_time_seconds', 'API response time')
system_uptime = Gauge('system_uptime_seconds', 'System uptime in seconds')

app = Flask(__name__)
auth = HTTPBasicAuth()

class GroundStationAPI:
    def __init__(self):
        """Initialize Ground Station API"""
        self.config = config.load_config()
        self.mqtt_client = None
        self.telemetry_processor = TelemetryProcessor()
        self.hil_simulator = None
        self.current_mode = "sim"
        self.system_status = "idle"
        self.latest_telemetry = {}
        self.fault_status = {}
        self.running = False
        self.start_time = time.time()
        
        # Setup Flask routes
        self._setup_routes()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        logger.info("Ground Station API initialized")
    
    def _setup_routes(self):
        """Setup Flask API routes"""
        
        @auth.verify_password
        def verify_password(username, password):
            users = self.config.get("api", {}).get("users", {"admin": "admin"})
            return users.get(username) == password
        
        @app.route('/health', methods=['GET'])
        def health_check():
            """Health check endpoint"""
            return jsonify({
                "status": "healthy",
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "version": "1.0.0",
                "uptime": time.time() - self.start_time
            })
        
        @app.route('/metrics', methods=['GET'])
        def metrics():
            """Prometheus metrics endpoint"""
            system_uptime.set(time.time() - self.start_time)
            return Response(generate_latest(), mimetype='text/plain')
        
        @app.route('/status', methods=['GET'])
        @auth.login_required
        def get_status():
            """Get system status"""
            with api_response_time.time():
                api_requests_total.labels(method='GET', endpoint='/status').inc()
                
                return jsonify({
                    "timestamp": datetime.now(timezone.utc).isoformat(),
                    "mode": self.current_mode,
                    "system_status": self.system_status,
                    "latest_telemetry": self.latest_telemetry,
                    "fault_status": self.fault_status,
                    "mqtt_connected": self.mqtt_client.is_connected() if self.mqtt_client else False,
                    "uptime": time.time() - self.start_time,
                    "services": {
                        "api_server": "running",
                        "mqtt_broker": "connected" if self.mqtt_client and self.mqtt_client.is_connected() else "disconnected",
                        "hil_simulator": "running" if self.hil_simulator and self.hil_simulator.running else "stopped",
                        "telemetry_processor": "active"
                    }
                })
        
        @app.route('/mode', methods=['POST'])
        @auth.login_required
        def set_mode():
            """Set telemetry mode"""
            with api_response_time.time():
                api_requests_total.labels(method='POST', endpoint='/mode').inc()
                
                data = request.get_json()
                if not data or 'mode' not in data:
                    return jsonify({"error": "Mode parameter required"}), 400
                
                mode = data['mode']
                if mode not in ['live', 'sim', 'hybrid', 'test']:
                    return jsonify({"error": "Invalid mode"}), 400
                
                # Send control command to sensor nodes
                self._send_control_command("rocket/telemetry/control/mode", {"mode": mode})
                
                # Control HIL simulator
                if mode == "sim":
                    if self.hil_simulator:
                        self.hil_simulator.start_simulation()
                elif mode == "live":
                    if self.hil_simulator:
                        self.hil_simulator.stop_simulation()
                
                self.current_mode = mode
                
                logger.info(f"Mode changed to: {mode}")
                return jsonify({
                    "status": "success",
                    "mode": mode,
                    "timestamp": datetime.now(timezone.utc).isoformat()
                })
        
        @app.route('/simulation/start', methods=['POST'])
        @auth.login_required
        def start_simulation():
            """Start HIL simulation"""
            with api_response_time.time():
                api_requests_total.labels(method='POST', endpoint='/simulation/start').inc()
                
                if self.hil_simulator:
                    self.hil_simulator.start_simulation()
                    return jsonify({
                        "status": "success",
                        "message": "HIL simulation started",
                        "timestamp": datetime.now(timezone.utc).isoformat()
                    })
                else:
                    return jsonify({"error": "HIL simulator not available"}), 503
        
        @app.route('/simulation/stop', methods=['POST'])
        @auth.login_required
        def stop_simulation():
            """Stop HIL simulation"""
            with api_response_time.time():
                api_requests_total.labels(method='POST', endpoint='/simulation/stop').inc()
                
                if self.hil_simulator:
                    self.hil_simulator.stop_simulation()
                    return jsonify({
                        "status": "success",
                        "message": "HIL simulation stopped",
                        "timestamp": datetime.now(timezone.utc).isoformat()
                    })
                else:
                    return jsonify({"error": "HIL simulator not available"}), 503
        
        @app.route('/fault', methods=['POST'])
        @auth.login_required
        def inject_fault():
            """Inject a fault"""
            with api_response_time.time():
                api_requests_total.labels(method='POST', endpoint='/fault').inc()
                
                data = request.get_json()
                if not data or 'type' not in data:
                    return jsonify({"error": "Fault type required"}), 400
                
                fault_type = data['type']
                duration = data.get('duration', 5.0)
                params = data.get('params', {})
                
                command = {
                    "command": "inject_fault",
                    "fault_type": fault_type,
                    "duration": duration,
                    "params": params
                }
                
                self._send_control_command("rocket/fault_control", command)
                
                logger.info(f"Fault injection requested: {fault_type}")
                return jsonify({
                    "status": "success",
                    "fault_type": fault_type,
                    "duration": duration,
                    "timestamp": datetime.now(timezone.utc).isoformat()
                })
        
        @app.route('/telemetry/latest', methods=['GET'])
        @auth.login_required
        def get_latest_telemetry():
            """Get latest telemetry data"""
            with api_response_time.time():
                api_requests_total.labels(method='GET', endpoint='/telemetry/latest').inc()
                
                return jsonify(self.latest_telemetry)
    
    def _setup_mqtt(self):
        """Setup secure MQTT connection"""
        self.mqtt_client = mqtt.Client(client_id=f"ground_station_{int(time.time())}")
        
        mqtt_config = self.config["mqtt"]
        
        # Setup TLS if enabled
        if mqtt_config.get("use_tls", False):
            context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
            context.load_verify_locations(mqtt_config["ca_cert"])
            context.load_cert_chain(mqtt_config["cert_file"], mqtt_config["key_file"])
            self.mqtt_client.tls_set_context(context)
        
        self.mqtt_client.username_pw_set(mqtt_config["username"], mqtt_config["password"])
        
        # Setup callbacks
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        try:
            self.mqtt_client.connect(mqtt_config["broker"], mqtt_config["port"], 60)
            self.mqtt_client.loop_start()
            logger.info("MQTT client connected")
        except Exception as e:
            logger.error(f"Failed to connect to MQTT broker: {e}")
            raise
    
    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            logger.info("Connected to MQTT broker")
            # Subscribe to telemetry topics
            client.subscribe("rocket/telemetry/data")
            client.subscribe("rocket/simulation/data")
            client.subscribe("rocket/faults/status")
            client.subscribe("rocket/faults/events")
            logger.info("Subscribed to telemetry topics")
        else:
            logger.error(f"Failed to connect to MQTT broker: {rc}")
    
    def _on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            
            if topic in ["rocket/telemetry/data", "rocket/simulation/data"]:
                self._process_telemetry(payload)
            elif topic == "rocket/faults/status":
                self.fault_status = payload
                active_faults.set(payload.get("fault_count", 0))
            elif topic == "rocket/faults/events":
                logger.info(f"Fault event: {payload.get('event_type', 'unknown')}")
                
        except Exception as e:
            logger.error(f"Error processing MQTT message: {e}")
    
    def _process_telemetry(self, telemetry: Dict[str, Any]):
        """Process incoming telemetry data"""
        try:
            # Update latest telemetry
            self.latest_telemetry = telemetry
            
            # Update Prometheus metrics
            telemetry_packets_total.inc()
            
            if "altitude" in telemetry:
                telemetry_altitude.set(telemetry["altitude"])
            if "velocity" in telemetry:
                telemetry_velocity.set(telemetry["velocity"])
            if "acceleration" in telemetry and "total" in telemetry["acceleration"]:
                telemetry_acceleration.set(telemetry["acceleration"]["total"])
            
            # Process through telemetry processor
            self.telemetry_processor.process(telemetry)
            
            # Update system status based on flight state
            flight_state = telemetry.get("flight_state", "unknown")
            if flight_state in ["boost", "coast", "descent"]:
                self.system_status = "active_flight"
            elif flight_state == "landed":
                self.system_status = "mission_complete"
            else:
                self.system_status = "standby"
                
        except Exception as e:
            logger.error(f"Error processing telemetry: {e}")
    
    def _send_control_command(self, topic: str, command: Dict[str, Any]):
        """Send control command via MQTT"""
        if self.mqtt_client and self.mqtt_client.is_connected():
            payload = json.dumps(command)
            result = self.mqtt_client.publish(topic, payload, qos=1)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                logger.error(f"Failed to send control command to {topic}")
        else:
            logger.warning("MQTT client not connected - command not sent")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        logger.info(f"Received signal {signum} - shutting down API server")
        self.stop()
    
    def start(self):
        """Start the ground station API server"""
        logger.info("Starting Ground Station API server")
        
        try:
            # Setup MQTT
            self._setup_mqtt()
            
            # Initialize HIL simulator
            self.hil_simulator = IntegratedHILSimulator(self.mqtt_client, self.config)
            
            # Start in simulation mode by default
            if self.current_mode == "sim":
                self.hil_simulator.start_simulation()
            
            # Start Flask app
            self.running = True
            api_config = self.config.get("api", {})
            
            ssl_context = None
            if api_config.get("use_ssl", False):
                ssl_context = (
                    api_config.get("ssl_cert", "/etc/ssl/certs/server.crt"),
                    api_config.get("ssl_key", "/etc/ssl/private/server.key")
                )
            
            app.run(
                host=api_config.get("host", "0.0.0.0"),
                port=api_config.get("port", 5000),
                debug=api_config.get("debug", False),
                ssl_context=ssl_context,
                threaded=True
            )
            
        except Exception as e:
            logger.error(f"Error starting API server: {e}")
            raise
        finally:
            self.stop()
    
    def stop(self):
        """Stop the API server"""
        logger.info("Stopping Ground Station API server")
        self.running = False
        
        if self.hil_simulator:
            self.hil_simulator.stop_simulation()
        
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

if __name__ == "__main__":
    try:
        api_server = GroundStationAPI()
        api_server.start()
    except KeyboardInterrupt:
        logger.info("API server stopped by user")
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        sys.exit(1)
