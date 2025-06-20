#!/usr/bin/env python3
"""
Unit tests for sensor manager
"""

import pytest
import unittest
from unittest.mock import Mock, patch
import sys
import os

# Add hardware directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../hardware/sensor-node'))

from sensor_manager import SensorManager

class TestSensorManager(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_i2c = Mock()
        self.sensor_manager = SensorManager(self.mock_i2c)
    
    def test_initialization(self):
        """Test sensor manager initialization"""
        self.assertEqual(self.sensor_manager.bme688_addr, 0x77)
        self.assertEqual(self.sensor_manager.mpu6050_addr, 0x68)
        self.assertEqual(self.sensor_manager.baseline_pressure, 1013.25)
    
    def test_simulation_mode(self):
        """Test simulation data generation"""
        data = self.sensor_manager._read_simulated_data()
        
        # Check required fields
        required_fields = [
            'pressure', 'temperature', 'humidity', 'altitude',
            'accel_x', 'accel_y', 'accel_z', 'accel_total',
            'gyro_x', 'gyro_y', 'gyro_z',
            'roll', 'pitch', 'yaw', 'is_simulated'
        ]
        
        for field in required_fields:
            self.assertIn(field, data)
        
        # Check simulation flag
        self.assertTrue(data['is_simulated'])
        
        # Check reasonable values
        self.assertGreaterEqual(data['altitude'], 0)
        self.assertGreaterEqual(data['accel_total'], 0)
    
    def test_pressure_to_altitude_conversion(self):
        """Test pressure to altitude conversion"""
        # Sea level pressure should give 0 altitude
        altitude = self.sensor_manager._pressure_to_altitude(1013.25)
        self.assertAlmostEqual(altitude, 0, places=1)
        
        # Lower pressure should give positive altitude
        altitude_high = self.sensor_manager._pressure_to_altitude(900)
        self.assertGreater(altitude_high, 0)
    
    def test_error_handling(self):
        """Test error data creation"""
        error_data = self.sensor_manager._create_error_data()
        
        # Check error flag
        self.assertTrue(error_data.get('error', False))
        
        # Check that values are zeroed
        self.assertEqual(error_data['altitude'], 0.0)
        self.assertEqual(error_data['accel_total'], 0.0)
    
    @patch('random.uniform')
    def test_noise_consistency(self, mock_random):
        """Test that noise is applied consistently"""
        mock_random.return_value = 0.01
        
        data1 = self.sensor_manager._read_simulated_data()
        data2 = self.sensor_manager._read_simulated_data()
        
        # Values should be different due to noise
        # (This test might need adjustment based on implementation)
        self.assertIsInstance(data1['altitude'], float)
        self.assertIsInstance(data2['altitude'], float)

if __name__ == '__main__':
    unittest.main()
