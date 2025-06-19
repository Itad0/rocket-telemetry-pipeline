#!/usr/bin/env python3
"""
Sensor Manager for Rocket Telemetry
Handles BME688 (pressure) and MPU6050 (IMU) sensors
"""

import time
import math
import logging
from typing import Dict, Any, Optional
import json

logger = logging.getLogger(__name__)

try:
    import board
    import busio
    import adafruit_bme680
    import adafruit_mpu6050
    HARDWARE_AVAILABLE = True
except ImportError:
    logger.warning("Hardware libraries not available - using simulation mode")
    HARDWARE_AVAILABLE = False

class SensorManager:
    def __init__(self):
        self.i2c = None
        self.bme = None
        self.mpu = None
        self.mode = "live"  # live, sim, test
        self.sea_level_pressure = 1013.25  # hPa
        self.baseline_altitude = 0.0
        self.initialized = False
        
        # Simulation state
        self.sim_time = 0.0
        self.sim_altitude = 0.0
        self.sim_velocity = 0.0
        
    def initialize(self):
        """Initialize sensors"""
        if HARDWARE_AVAILABLE and self.mode == "live":
            try:
                self.i2c = busio.I2C(board.SCL, board.SDA)
                
                # Initialize BME688 (pressure/temperature/humidity)
                self.bme = adafruit_bme680.Adafruit_BME680_I2C(self.i2c)
                self.bme.sea_level_pressure = self.sea_level_pressure
                
                # Initialize MPU6050 (IMU)
                self.mpu = adafruit_mpu6050.MPU6050(self.i2c)
                
                # Calibrate baseline altitude
                pressure_readings = []
                for _ in range(10):
                    pressure_readings.append(self.bme.pressure)
                    time.sleep(0.1)
                
                avg_pressure = sum(pressure_readings) / len(pressure_readings)
                self.baseline_altitude = self._pressure_to_altitude(avg_pressure)
                
                logger.info(f"Sensors initialized - baseline altitude: {self.baseline_altitude:.2f}m")
                self.initialized = True
                
            except Exception as e:
                logger.error(f"Failed to initialize sensors: {e}")
                logger.info("Falling back to simulation mode")
                self.mode = "sim"
                self.initialized = True
        else:
            logger.info("Using simulation mode")
            self.mode = "sim"
            self.initialized = True
    
    def set_mode(self, mode: str):
        """Set sensor mode (live/sim/test)"""
        if mode in ["live", "sim", "test"]:
            self.mode = mode
            logger.info(f"Sensor mode set to: {mode}")
    
    def calibrate_sensors(self):
        """Calibrate sensors (re-establish baseline)"""
        if self.mode == "live" and self.bme:
            try:
                pressure_readings = []
                for _ in range(20):
                    pressure_readings.append(self.bme.pressure)
                    time.sleep(0.05)
                
                avg_pressure = sum(pressure_readings) / len(pressure_readings)
                self.baseline_altitude = self._pressure_to_altitude(avg_pressure)
                logger.info(f"Sensors recalibrated - new baseline: {self.baseline_altitude:.2f}m")
            except Exception as e:
                logger.error(f"Calibration failed: {e}")
    
    def read_all_sensors(self) -> Dict[str, Any]:
        """Read all sensor data"""
        if not self.initialized:
            raise RuntimeError("Sensors not initialized")
        
        if self.mode == "live":
            return self._read_hardware_sensors()
        elif self.mode == "sim":
            return self._read_simulated_sensors()
        else:  # test mode
            return self._read_test_sensors()
    
    def _read_hardware_sensors(self) -> Dict[str, Any]:
        """Read actual hardware sensors"""
        try:
            # Read pressure sensor (BME688)
            pressure = self.bme.pressure
            temperature = self.bme.temperature
            humidity = self.bme.relative_humidity
            altitude = self._pressure_to_altitude(pressure) - self.baseline_altitude
            
            # Read IMU (MPU6050)
            accel_x, accel_y, accel_z = self.mpu.acceleration
            gyro_x, gyro_y, gyro_z = self.mpu.gyro
            
            # Convert acceleration from m/s² to g
            accel_x_g = accel_x / 9.81
            accel_y_g = accel_y / 9.81
            accel_z_g = accel_z / 9.81
            accel_total = math.sqrt(accel_x_g**2 + accel_y_g**2 + accel_z_g**2)
            
            # Calculate orientation (simplified - for demonstration)
            roll = math.atan2(accel_y_g, accel_z_g) * 180 / math.pi
            pitch = math.atan2(-accel_x_g, math.sqrt(accel_y_g**2 + accel_z_g**2)) * 180 / math.pi
            yaw = 0.0  # Would need magnetometer for proper yaw
            
            # System information
            battery_voltage = self._read_battery_voltage()
            cpu_temp = self._read_cpu_temperature()
            memory_usage = self._get_memory_usage()
            
            return {
                "pressure": {
                    "pressure": pressure,
                    "temperature": temperature,
                    "humidity": humidity,
                    "altitude": altitude
                },
                "imu": {
                    "accel_x": accel_x_g,
                    "accel_y": accel_y_g,
                    "accel_z": accel_z_g,
                    "accel_total": accel_total,
                    "gyro_x": gyro_x,
                    "gyro_y": gyro_y,
                    "gyro_z": gyro_z,
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw
                },
                "system": {
                    "battery_voltage": battery_voltage,
                    "cpu_temp": cpu_temp,
                    "memory_usage": memory_usage
                }
            }
            
        except Exception as e:
            logger.error(f"Error reading hardware sensors: {e}")
            raise
    
    def _read_simulated_sensors(self) -> Dict[str, Any]:
        """Generate simulated sensor data"""
        self.sim_time += 0.1
        
        # Simple flight simulation
        if self.sim_time < 2.0:  # Pre-flight
            altitude = 0.0
            accel_z = 1.0  # 1g (gravity)
            accel_total = 1.0
        elif self.sim_time < 5.0:  # Boost phase
            t = self.sim_time - 2.0
            altitude = 0.5 * 20 * t**2  # 20 m/s² acceleration
            accel_z = -3.0  # 3g upward acceleration
            accel_total = 3.2
        elif self.sim_time < 15.0:  # Coast phase
            t = self.sim_time - 5.0
            altitude = 150 + 60*t - 0.5*9.81*t**2  # Ballistic trajectory
            accel_z = 1.0  # Just gravity
            accel_total = 1.0
        else:  # Descent
            t = self.sim_time - 15.0
            altitude = max(0, 300 - 5*t)  # Parachute descent
            accel_z = 0.5  # Reduced acceleration due to parachute
            accel_total = 0.7
        
        # Add some noise
        import random
        altitude += random.uniform(-0.5, 0.5)
        accel_z += random.uniform(-0.05, 0.05)
        
        # Calculate pressure from altitude
        pressure = self._altitude_to_pressure(altitude + self.baseline_altitude)
        
        return {
            "pressure": {
                "pressure": pressure,
                "temperature": 20.0 + random.uniform(-2, 2),
                "humidity": 45.0 + random.uniform(-5, 5),
                "altitude": altitude
            },
            "imu": {
                "accel_x": random.uniform(-0.1, 0.1),
                "accel_y": random.uniform(-0.1, 0.1),
                "accel_z": accel_z,
                "accel_total": accel_total,
                "gyro_x": random.uniform(-5, 5),
                "gyro_y": random.uniform(-5, 5),
                "gyro_z": random.uniform(-2, 2),
                "roll": random.uniform(-10, 10),
                "pitch": random.uniform(-5, 5),
                "yaw": random.uniform(-180, 180)
            },
            "system": {
                "battery_voltage": 3.7 - (self.sim_time * 0.001),
                "cpu_temp": 45.0 + random.uniform(-3, 3),
                "memory_usage": 25.0 + random.uniform(-2, 2)
            }
        }
    
    def _read_test_sensors(self) -> Dict[str, Any]:
        """Generate test sensor data (static values)"""
        return {
            "pressure": {
                "pressure": 1013.25,
                "temperature": 22.0,
                "humidity": 50.0,
                "altitude": 0.0
            },
            "imu": {
                "accel_x": 0.0,
                "accel_y": 0.0,
                "accel_z": 1.0,
                "accel_total": 1.0,
                "gyro_x": 0.0,
                "gyro_y": 0.0,
                "gyro_z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0
            },
            "system": {
                "battery_voltage": 3.7,
                "cpu_temp": 45.0,
                "memory_usage": 25.0
            }
        }
    
    def _pressure_to_altitude(self, pressure_hpa: float) -> float:
        """Convert pressure to altitude using barometric formula"""
        return 44330.0 * (1.0 - (pressure_hpa / self.sea_level_pressure) ** 0.1903)
    
    def _altitude_to_pressure(self, altitude_m: float) -> float:
        """Convert altitude to pressure"""
        return self.sea_level_pressure * ((1.0 - altitude_m / 44330.0) ** (1.0 / 0.1903))
    
    def _read_battery_voltage(self) -> float:
        """Read battery voltage (platform-specific implementation)"""
        try:
            # This would be platform-specific - example for Raspberry Pi
            with open('/sys/class/power_supply/BAT0/voltage_now', 'r') as f:
                voltage_uv = int(f.read().strip())
                return voltage_uv / 1000000.0  # Convert µV to V
        except:
            return 3.7  # Default value
    
    def _read_cpu_temperature(self) -> float:
        """Read CPU temperature"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_millicelsius = int(f.read().strip())
                return temp_millicelsius / 1000.0
        except:
            return 45.0  # Default value
    
    def _get_memory_usage(self) -> float:
        """Get memory usage percentage"""
        try:
            with open('/proc/meminfo', 'r') as f:
                lines = f.readlines()
                mem_total = int(lines[0].split()[1])
                mem_available = int(lines[2].split()[1])
                return ((mem_total - mem_available) / mem_total) * 100
        except:
            return 25.0  # Default value
    
    def cleanup(self):
        """Cleanup sensor resources"""
        if self.i2c:
            self.i2c.deinit()
        logger.info("Sensor manager cleaned up")
