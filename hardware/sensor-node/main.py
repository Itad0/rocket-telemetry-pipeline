#!/usr/bin/env python3
"""
Rocket Telemetry Sensor Node - Raspberry Pi Pico W
Main telemetry collection and transmission system
"""

import time
import json
import math
import random
import gc
from machine import Pin, I2C, ADC, reset
import network
import urequests
from umqtt.simple import MQTTClient
import ubinascii
import machine

# Configuration
CONFIG = {
    "wifi": {
        "ssid": "YourWiFiNetwork",
        "password": "YourWiFiPassword"
    },
    "mqtt": {
        "broker": "192.168.1.100",  # Ground station IP
        "port": 1883,
        "client_id": None,  # Will be generated
        "topic_prefix": "rocket/telemetry",
        "username": "rocket_node",
        "password": "secure_password"
    },
    "sensors": {
        "sample_rate": 10,  # Hz
        "altitude_offset": 0.0,
        "enable_simulation": False
    },
    "system": {
        "status_led_pin": 15,
        "battery_adc_pin": 26,
        "i2c_sda_pin": 4,
        "i2c_scl_pin": 5
    }
}

class StatusLED:
    """Status LED controller"""
    def __init__(self, pin_num):
        self.led = Pin(pin_num, Pin.OUT)
        self.state = False
    
    def on(self):
        self.led.on()
        self.state = True
    
    def off(self):
        self.led.off()
        self.state = False
    
    def toggle(self):
        if self.state:
            self.off()
        else:
            self.on()
    
    def blink(self, times=3, delay=0.2):
        for _ in range(times):
            self.on()
            time.sleep(delay)
            self.off()
            time.sleep(delay)

class SensorManager:
    """Sensor interface for BME688 and MPU6050"""
    def __init__(self, i2c):
        self.i2c = i2c
        self.bme688_addr = 0x77
        self.mpu6050_addr = 0x68
        self.baseline_pressure = 1013.25  # hPa
        self.simulation_time = 0.0
        
    def initialize(self):
        """Initialize sensors"""
        try:
            # Scan for I2C devices
            devices = self.i2c.scan()
            print(f"I2C devices found: {[hex(d) for d in devices]}")
            
            # Check for BME688
            if self.bme688_addr in devices:
                print("BME688 found")
                self._init_bme688()
            else:
                print("BME688 not found - using simulation")
                
            # Check for MPU6050
            if self.mpu6050_addr in devices:
                print("MPU6050 found")
                self._init_mpu6050()
            else:
                print("MPU6050 not found - using simulation")
                
            return True
        except Exception as e:
            print(f"Sensor initialization error: {e}")
            return False
    
    def _init_bme688(self):
        """Initialize BME688 sensor"""
        try:
            # Basic BME688 initialization
            # Write to ctrl_meas register (0x74)
            self.i2c.writeto_mem(self.bme688_addr, 0x74, bytes([0x27]))
            time.sleep(0.1)
        except Exception as e:
            print(f"BME688 init error: {e}")
    
    def _init_mpu6050(self):
        """Initialize MPU6050 sensor"""
        try:
            # Wake up MPU6050 (exit sleep mode)
            self.i2c.writeto_mem(self.mpu6050_addr, 0x6B, bytes([0x00]))
            time.sleep(0.1)
            
            # Set sample rate to 100Hz
            self.i2c.writeto_mem(self.mpu6050_addr, 0x19, bytes([0x07]))
            
            # Set accelerometer range to ±8g
            self.i2c.writeto_mem(self.mpu6050_addr, 0x1C, bytes([0x10]))
            
            # Set gyroscope range to ±1000°/s
            self.i2c.writeto_mem(self.mpu6050_addr, 0x1B, bytes([0x10]))
            
        except Exception as e:
            print(f"MPU6050 init error: {e}")
    
    def read_sensors(self):
        """Read all sensor data"""
        try:
            if CONFIG["sensors"]["enable_simulation"]:
                return self._read_simulated_data()
            else:
                return self._read_hardware_sensors()
        except Exception as e:
            print(f"Sensor read error: {e}")
            return self._create_error_data()
    
    def _read_hardware_sensors(self):
        """Read actual hardware sensors"""
        try:
            # Read BME688 (simplified)
            pressure = self._read_pressure()
            temperature = self._read_temperature()
            humidity = 45.0  # Placeholder
            altitude = self._pressure_to_altitude(pressure)
            
            # Read MPU6050
            accel_data = self._read_accelerometer()
            gyro_data = self._read_gyroscope()
            
            # Calculate orientation (simplified)
            roll = math.atan2(accel_data[1], accel_data[2]) * 180 / math.pi
            pitch = math.atan2(-accel_data[0], 
                             math.sqrt(accel_data[1]**2 + accel_data[2]**2)) * 180 / math.pi
            
            return {
                "pressure": pressure,
                "temperature": temperature,
                "humidity": humidity,
                "altitude": altitude - CONFIG["sensors"]["altitude_offset"],
                "accel_x": accel_data[0],
                "accel_y": accel_data[1],
                "accel_z": accel_data[2],
                "accel_total": math.sqrt(sum(x**2 for x in accel_data)),
                "gyro_x": gyro_data[0],
                "gyro_y": gyro_data[1],
                "gyro_z": gyro_data[2],
                "roll": roll,
                "pitch": pitch,
                "yaw": 0.0,  # Requires magnetometer
                "is_simulated": False
            }
            
        except Exception as e:
            print(f"Hardware sensor read error: {e}")
            return self._read_simulated_data()
    
    def _read_simulated_data(self):
        """Generate simulated sensor data"""
        self.simulation_time += 0.1
        
        # Simple flight simulation
        if self.simulation_time < 2.0:  # Pre-flight
            altitude = 0.0
            accel_z = 1.0  # 1g (gravity)
            accel_total = 1.0
        elif self.simulation_time < 5.0:  # Boost phase
            t = self.simulation_time - 2.0
            altitude = 0.5 * 20 * t**2  # 20 m/s² acceleration
            accel_z = -3.0  # 3g upward acceleration
            accel_total = 3.2
        elif self.simulation_time < 15.0:  # Coast phase
            t = self.simulation_time - 5.0
            altitude = 150 + 60*t - 0.5*9.81*t**2  # Ballistic trajectory
            accel_z = 1.0  # Just gravity
            accel_total = 1.0
        else:  # Descent
            t = self.simulation_time - 15.0
            altitude = max(0, 300 - 5*t)  # Parachute descent
            accel_z = 0.5  # Reduced acceleration due to parachute
            accel_total = 0.7
        
        # Add noise
        noise = lambda: random.uniform(-0.02, 0.02)
        
        return {
            "pressure": 1013.25 * ((1.0 - altitude / 44330.0) ** (1.0 / 0.1903)),
            "temperature": 20.0 + noise() * 5,
            "humidity": 45.0 + noise() * 10,
            "altitude": altitude + noise() * 0.5,
            "accel_x": noise() * 0.1,
            "accel_y": noise() * 0.1,
            "accel_z": accel_z + noise() * 0.05,
            "accel_total": accel_total + noise() * 0.05,
            "gyro_x": noise() * 5,
            "gyro_y": noise() * 5,
            "gyro_z": noise() * 2,
            "roll": noise() * 10,
            "pitch": 85.0 + noise() * 10 if 2.0 <= self.simulation_time <= 15.0 else noise() * 10,
            "yaw": random.uniform(-180, 180),
            "is_simulated": True
        }
    
    def _read_pressure(self):
        """Read pressure from BME688"""
        try:
            # Simplified BME688 pressure reading
            raw_data = self.i2c.readfrom_mem(self.bme688_addr, 0x1F, 3)
            raw_pressure = (raw_data[0] << 16) | (raw_data[1] << 8) | raw_data[2]
            # Apply calibration (simplified)
            return 1013.25 + random.uniform(-5, 5)  # Placeholder
        except:
            return 1013.25
    
    def _read_temperature(self):
        """Read temperature from BME688"""
        try:
            return 20.0 + random.uniform(-2, 2)  # Placeholder
        except:
            return 20.0
    
    def _read_accelerometer(self):
        """Read accelerometer data from MPU6050"""
        try:
            # Read 6 bytes starting from register 0x3B
            data = self.i2c.readfrom_mem(self.mpu6050_addr, 0x3B, 6)
            
            # Combine high and low bytes
            ax = self._combine_bytes(data[0], data[1]) / 4096.0  # ±8g range
            ay = self._combine_bytes(data[2], data[3]) / 4096.0
            az = self._combine_bytes(data[4], data[5]) / 4096.0
            
            return [ax, ay, az]
        except:
            return [0.0, 0.0, 1.0]  # Default gravity
    
    def _read_gyroscope(self):
        """Read gyroscope data from MPU6050"""
        try:
            # Read 6 bytes starting from register 0x43
            data = self.i2c.readfrom_mem(self.mpu6050_addr, 0x43, 6)
            
            # Combine high and low bytes
            gx = self._combine_bytes(data[0], data[1]) / 32.8  # ±1000°/s range
            gy = self._combine_bytes(data[2], data[3]) / 32.8
            gz = self._combine_bytes(data[4], data[5]) / 32.8
            
            return [gx, gy, gz]
        except:
            return [0.0, 0.0, 0.0]
    
    def _combine_bytes(self, high, low):
        """Combine high and low bytes into signed 16-bit value"""
        value = (high << 8) | low
        if value >= 32768:
            value -= 65536
        return value
    
    def _pressure_to_altitude(self, pressure_hpa):
        """Convert pressure to altitude using barometric formula"""
        return 44330.0 * (1.0 - (pressure_hpa / self.baseline_pressure) ** 0.1903)
    
    def _create_error_data(self):
        """Create error data when sensors fail"""
        return {
            "pressure": 0.0,
            "temperature": 0.0,
            "humidity": 0.0,
            "altitude": 0.0,
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 0.0,
            "accel_total": 0.0,
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "is_simulated": False,
            "error": True
        }

class TelemetryNode:
    """Main telemetry node controller"""
    def __init__(self):
        self.status_led = StatusLED(CONFIG["system"]["status_led_pin"])
        self.i2c = I2C(0, sda=Pin(CONFIG["system"]["i2c_sda_pin"]), 
                          scl=Pin(CONFIG["system"]["i2c_scl_pin"]), freq=400000)
        self.sensor_manager = SensorManager(self.i2c)
        self.battery_adc = ADC(Pin(CONFIG["system"]["battery_adc_pin"]))
        self.mqtt_client = None
        self.wifi = network.WLAN(network.STA_IF)
        
        self.mission_time = 0.0
        self.flight_state = "pre_flight"
        self.running = False
        
        # Generate unique client ID
        CONFIG["mqtt"]["client_id"] = f"rocket_node_{ubinascii.hexlify(machine.unique_id()).decode()}"
    
    def initialize(self):
        """Initialize the telemetry node"""
        print("🚀 Initializing Rocket Telemetry Node...")
        
        # LED startup sequence
        self.status_led.blink(5, 0.1)
        
        # Initialize sensors
        if not self.sensor_manager.initialize():
            print("⚠️ Sensor initialization failed - using simulation mode")
            CONFIG["sensors"]["enable_simulation"] = True
        
        # Connect to WiFi
        if not self._connect_wifi():
            print("❌ WiFi connection failed")
            self._error_state()
            return False
        
        # Connect to MQTT
        if not self._connect_mqtt():
            print("❌ MQTT connection failed")
            self._error_state()
            return False
        
        print("✅ Telemetry node initialized successfully")
        self.status_led.on()
        return True
    
    def _connect_wifi(self):
        """Connect to WiFi network"""
        print(f"📶 Connecting to WiFi: {CONFIG['wifi']['ssid']}")
        
        self.wifi.active(True)
        self.wifi.connect(CONFIG['wifi']['ssid'], CONFIG['wifi']['password'])
        
        # Wait for connection
        timeout = 10
        while timeout > 0 and not self.wifi.isconnected():
            self.status_led.toggle()
            time.sleep(1)
            timeout -= 1
        
        if self.wifi.isconnected():
            ip_info = self.wifi.ifconfig()
            print(f"✅ WiFi connected: {ip_info[0]}")
            return True
        else:
            print("❌ WiFi connection timeout")
            return False
    
    def _connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            print(f"📡 Connecting to MQTT broker: {CONFIG['mqtt']['broker']}")
            
            self.mqtt_client = MQTTClient(
                CONFIG["mqtt"]["client_id"],
                CONFIG["mqtt"]["broker"],
                port=CONFIG["mqtt"]["port"],
                user=CONFIG["mqtt"]["username"],
                password=CONFIG["mqtt"]["password"]
            )
            
            self.mqtt_client.set_callback(self._mqtt_callback)
            self.mqtt_client.connect()
            
            # Subscribe to control topics
            control_topic = f"{CONFIG['mqtt']['topic_prefix']}/control/+"
            self.mqtt_client.subscribe(control_topic)
            
            print("✅ MQTT connected")
            return True
            
        except Exception as e:
            print(f"❌ MQTT connection error: {e}")
            return False
    
    def _mqtt_callback(self, topic, msg):
        """Handle incoming MQTT messages"""
        try:
            topic_str = topic.decode()
            msg_str = msg.decode()
            print(f"📨 MQTT message: {topic_str} = {msg_str}")
            
            # Parse control commands
            if "/control/" in topic_str:
                command = topic_str.split("/")[-1]
                try:
                    payload = json.loads(msg_str)
                    self._handle_control_command(command, payload)
                except:
                    # Simple command without JSON
                    self._handle_control_command(command, {})
                    
        except Exception as e:
            print(f"MQTT callback error: {e}")
    
    def _handle_control_command(self, command, payload):
        """Handle control commands from ground station"""
        print(f"🎮 Control command: {command}")
        
        if command == "mode":
            mode = payload.get("mode", "live")
            if mode == "sim":
                CONFIG["sensors"]["enable_simulation"] = True
                print("Switched to simulation mode")
            else:
                CONFIG["sensors"]["enable_simulation"] = False
                print("Switched to live sensor mode")
        
        elif command == "reset":
            self.mission_time = 0.0
            self.flight_state = "pre_flight"
            print("Mission reset")
        
        elif command == "calibrate":
            # Perform sensor calibration
            self._calibrate_sensors()
        
        elif command == "reboot":
            print("Rebooting in 3 seconds...")
            time.sleep(3)
            reset()
    
    def _calibrate_sensors(self):
        """Calibrate sensors (establish baseline)"""
        print("🔧 Calibrating sensors...")
        self.status_led.blink(3, 0.5)
        
        # Take several pressure readings for baseline
        pressures = []
        for _ in range(10):
            data = self.sensor_manager.read_sensors()
            pressures.append(data.get("pressure", 1013.25))
            time.sleep(0.1)
        
        avg_pressure = sum(pressures) / len(pressures)
        baseline_altitude = self.sensor_manager._pressure_to_altitude(avg_pressure)
        CONFIG["sensors"]["altitude_offset"] = baseline_altitude
        
        print(f"✅ Calibration complete - baseline altitude: {baseline_altitude:.2f}m")
    
    def _read_battery_voltage(self):
        """Read battery voltage via ADC"""
        try:
            # Voltage divider: 3.3V ADC, 16-bit resolution
            adc_reading = self.battery_adc.read_u16()
            voltage = (adc_reading / 65535.0) * 3.3 * 2  # Assuming 2:1 voltage divider
            return voltage
        except:
            return 3.7  # Default value
    
    def _get_system_info(self):
        """Get system information"""
        return {
            "battery_voltage": self._read_battery_voltage(),
            "memory_free": gc.mem_free(),
            "wifi_rssi": self.wifi.status('rssi') if self.wifi.isconnected() else 0,
            "uptime": time.ticks_ms() / 1000.0
        }
    
    def _update_flight_state(self, sensor_data):
        """Update flight state based on sensor data"""
        accel_total = sensor_data.get("accel_total", 1.0)
        altitude = sensor_data.get("altitude", 0.0)
        
        if self.flight_state == "pre_flight":
            if accel_total > 2.0:  # 2g threshold for launch detection
                self.flight_state = "boost"
                print("🚀 Launch detected!")
        
        elif self.flight_state == "boost":
            if accel_total < 1.2:  # Motor burnout
                self.flight_state = "coast"
                print("🔥 Motor burnout - coasting")
        
        elif self.flight_state == "coast":
            if altitude > 50 and accel_total < 0.8:  # Approaching apogee
                self.flight_state = "apogee"
                print("🔝 Apogee reached")
        
        elif self.flight_state == "apogee":
            if altitude < 50:  # Significant descent
                self.flight_state = "descent"
                print("🪂 Descent phase")
        
        elif self.flight_state == "descent":
            if altitude < 5 and accel_total < 0.5:  # Near ground with low acceleration
                self.flight_state = "landed"
                print("🎯 Landing detected")
    
    def _create_telemetry_packet(self, sensor_data, system_info):
        """Create telemetry packet"""
        return {
            "timestamp": time.time(),
            "mission_time": self.mission_time,
            "flight_state": self.flight_state,
            "altitude": sensor_data.get("altitude", 0.0),
            "velocity": 0.0,  # Would need integration or GPS
            "acceleration": {
                "x": sensor_data.get("accel_x", 0.0),
                "y": sensor_data.get("accel_y", 0.0),
                "z": sensor_data.get("accel_z", 0.0),
                "total": sensor_data.get("accel_total", 0.0)
            },
            "orientation": {
                "roll": sensor_data.get("roll", 0.0),
                "pitch": sensor_data.get("pitch", 0.0),
                "yaw": sensor_data.get("yaw", 0.0)
            },
            "angular_velocity": {
                "x": sensor_data.get("gyro_x", 0.0),
                "y": sensor_data.get("gyro_y", 0.0),
                "z": sensor_data.get("gyro_z", 0.0)
            },
            "environmental": {
                "pressure": sensor_data.get("pressure", 0.0),
                "temperature": sensor_data.get("temperature", 0.0),
        "humidity": sensor_data.get("humidity", 0.0)
           },
           "system": {
               "battery_voltage": system_info.get("battery_voltage", 0.0),
               "memory_free": system_info.get("memory_free", 0),
               "wifi_rssi": system_info.get("wifi_rssi", 0),
               "uptime": system_info.get("uptime", 0.0)
           },
           "device": {
               "client_id": CONFIG["mqtt"]["client_id"],
               "is_simulated": sensor_data.get("is_simulated", False),
               "has_error": sensor_data.get("error", False)
           }
       }
   
   def _publish_telemetry(self, telemetry):
       """Publish telemetry via MQTT"""
       try:
           topic = f"{CONFIG['mqtt']['topic_prefix']}/data"
           payload = json.dumps(telemetry)
           
           self.mqtt_client.publish(topic, payload)
           
           # Blink LED to indicate transmission
           self.status_led.toggle()
           
       except Exception as e:
           print(f"MQTT publish error: {e}")
           # Try to reconnect
           self._reconnect_mqtt()
   
   def _reconnect_mqtt(self):
       """Attempt to reconnect to MQTT"""
       try:
           print("🔄 Attempting MQTT reconnection...")
           self.mqtt_client.disconnect()
           time.sleep(1)
           self.mqtt_client.connect()
           print("✅ MQTT reconnected")
       except:
           print("❌ MQTT reconnection failed")
   
   def _error_state(self):
       """Enter error state with blinking LED"""
       print("💥 Entering error state")
       for _ in range(10):
           self.status_led.blink(1, 0.1)
           time.sleep(0.5)
   
   def run(self):
       """Main telemetry loop"""
       if not self.initialize():
           return
       
       print("🎯 Starting telemetry transmission...")
       self.running = True
       
       sample_interval = 1.0 / CONFIG["sensors"]["sample_rate"]
       start_time = time.ticks_ms()
       
       while self.running:
           loop_start = time.ticks_ms()
           
           try:
               # Update mission time
               self.mission_time = (time.ticks_ms() - start_time) / 1000.0
               
               # Read sensors
               sensor_data = self.sensor_manager.read_sensors()
               system_info = self._get_system_info()
               
               # Update flight state
               self._update_flight_state(sensor_data)
               
               # Create and publish telemetry
               telemetry = self._create_telemetry_packet(sensor_data, system_info)
               self._publish_telemetry(telemetry)
               
               # Check for incoming MQTT messages
               self.mqtt_client.check_msg()
               
               # Memory management
               if self.mission_time % 10 == 0:
                   gc.collect()
               
               # Maintain sample rate
               loop_time = time.ticks_diff(time.ticks_ms(), loop_start) / 1000.0
               if loop_time < sample_interval:
                   time.sleep(sample_interval - loop_time)
               
           except KeyboardInterrupt:
               print("🛑 Stopping telemetry node...")
               self.running = False
               break
           except Exception as e:
               print(f"❌ Error in main loop: {e}")
               self.status_led.blink(2, 0.2)
               time.sleep(1)
       
       # Cleanup
       if self.mqtt_client:
           try:
               self.mqtt_client.disconnect()
           except:
               pass
       
       self.status_led.off()
       print("✅ Telemetry node stopped")

# Main execution
if __name__ == "__main__":
   try:
       node = TelemetryNode()
       node.run()
   except Exception as e:
       print(f"💥 Fatal error: {e}")
       # Emergency LED pattern
       led = StatusLED(15)
       for _ in range(5):
           led.blink(3, 0.1)
           time.sleep(1)
