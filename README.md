## Documentation

### Main README (`docs/README.md`)

```markdown
# Secure Rocket Telemetry Pipeline

A comprehensive, production-ready telemetry system for model rockets featuring
Hardware-in-the-Loop (HIL) simulation, fault injection, real-time monitoring, and security controls.

## Features

###  Core Telemetry
- Real-time sensor data collection (BME688 pressure, MPU6050 IMU)
- Secure MQTT transmission with TLS encryption
- Flight phase detection and state management
- Hardware abstraction supporting Raspberry Pi Pico W/Zero 2 W

###  HIL Simulation
- Realistic flight physics engine with atmospheric modeling
- Motor thrust curves and parachute deployment simulation
- Configurable rocket parameters and environmental conditions
- Real-time vs accelerated time simulation modes

###  Fault Injection
- Comprehensive fault types: sensor spikes, dropouts, communication loss
- Scheduled and on-demand fault injection
- Remote control via REST API and MQTT
- Fault event logging and analysis

###  Monitoring & Observability
- Grafana dashboards for real-time visualization
- Prometheus metrics collection and alerting
- Loki log aggregation across all components
- System health monitoring and anomaly detection

###  Security
- TLS encryption for all communications
- Certificate-based MQTT authentication
- API authentication with role-based access
- Input validation and schema enforcement

## Quick Start

1. Environment Setup
```bash
git clone <repository-url>
cd rocket-telemetry-pipeline
chmod +x scripts/*.sh
./scripts/setup_environment.sh
```

2. Generate Certificates
```bash
./scripts/generate_certs.sh
```

3. Deploy Infrastructure
```bash
./scripts/deploy.sh
```

4. Start Telemetry Nodes
```bash
# Terminal 1 - HIL Simulator
cd hil_simulation
source venv/bin/activate
python sim_telemetry.py

# Terminal 2 - Ground Station API
cd ground_station
source venv/bin/activate
python api_server.py

# Terminal 3 - Fault Injector (optional)
cd fault_injector
source venv/bin/activate
python fault_injector.py
```

5. Access Dashboards
- Grafana: https://localhost:3000 (admin/admin)
- Prometheus: http://localhost:9090
- **Ground Station API**: https://localhost:5000

## Architecture

### System Components

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Node   │    │  HIL Simulator  │    │ Fault Injector  │
│ (Rocket/Pi)     │    │  (Physics Sim)  │    │ (Test Faults)   │
└─────────┬───────┘    └─────────┬───────┘    └─────────┬───────┘
          │                      │                      │
          └──────────────────────┼──────────────────────┘
                                 │
                    ┌─────────────┴─────────────┐
                    │      MQTT Broker          │
                    │     (TLS Secured)         │
                    └─────────────┬─────────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    │   Ground Station API      │
                    │  (Control & Processing)   │
                    └─────────────┬─────────────┘
                                  │
          ┌───────────────────────┼───────────────────────┐
          │                       │                       │
    ┌─────▼─────┐         ┌───────▼───────┐       ┌───────▼───────┐
    │ Prometheus│         │    Grafana    │       │     Loki      │
    │ (Metrics) │         │ (Dashboards)  │       │    (Logs)     │
    └───────────┘         └───────────────┘       └───────────────┘
```

### Data Flow

1. Collection: Sensor node or HIL simulator generates telemetry
2. Transmission: Data sent via secure MQTT to ground station
3. Processing: Ground station processes, stores, and analyzes data
4. Monitoring: Prometheus scrapes metrics, Loki collects logs
5. Visualization: Grafana displays real-time dashboards
6. Control: API endpoints allow remote system control

## Hardware Setup

### Rocket Sensor Node
- Microcontroller: Raspberry Pi Pico W or Pi Zero 2 W
- Sensors: 
  - BME688 (pressure, temperature, humidity, gas)
  - MPU6050 (6-axis IMU: accelerometer + gyroscope)
- Optional: PiCam, status LED, GPS module
- Power: LiPo battery (3.7V) with voltage regulator
- Communication: Wi-Fi (onboard) or LoRa module

### Ground Station
- Computer: Raspberry Pi 4/5 or laptop with Docker
- Network: Wi-Fi or Ethernet for MQTT broker
- Storage: 32GB+ MicroSD card or SSD for data logging
- Display: Monitor for real-time dashboard viewing

### Wiring Diagram
```
Raspberry Pi Pico W Connections:
├── BME688 (I2C)
│   ├── VCC → 3.3V
│   ├── GND → GND
│   ├── SDA → GPIO 4 (Pin 6)
│   └── SCL → GPIO 5 (Pin 7)
├── MPU6050 (I2C)
│   ├── VCC → 3.3V
│   ├── GND → GND
│   ├── SDA → GPIO 4 (Pin 6)
│   └── SCL → GPIO 5 (Pin 7)
├── Status LED
│   ├── Anode → GPIO 15 (Pin 20)
│   └── Cathode → GND (via 220Ω resistor)
└── Battery Monitor
    └── VSYS → ADC0 (Pin 31) via voltage divider
```

## Configuration

### Environment Variables
```bash
# MQTT Configuration
export MQTT_BROKER="localhost"
export MQTT_PORT="8883"
export MQTT_USERNAME="rocket_node"
export MQTT_PASSWORD="secure_password"

# API Configuration
export API_HOST="0.0.0.0"
export API_PORT="5000"
export API_ADMIN_PASSWORD="your_secure_password"

# Database Configuration
export DB_PATH="/var/lib/rocket_telemetry.db"
```

### Component Configuration Files
- `sensor_node/config.json` - Sensor calibration and MQTT settings
- `hil_simulation/sim_config.json` - Physics simulation parameters
- `fault_injector/fault_config.json` - Fault injection rules
- `ground_station/config.py` - API and processing configuration

## API Reference

### REST Endpoints

#### System Control
```http
GET  /health                    # Health check
GET  /status                    # System status
POST /mode                      # Set telemetry mode
POST /control/calibrate         # Calibrate sensors
POST /control/reset             # Reset mission data
```

#### Fault Injection
```http
POST /fault                     # Inject fault
POST /fault/clear               # Clear all faults
GET  /fault/status              # Get fault status
```

#### Telemetry Data
```http
GET  /telemetry/latest          # Latest telemetry packet
GET  /telemetry/history         # Historical data
GET  /metrics                   # Prometheus metrics
```

### Example API Usage

Set Simulation Mode
```bash
curl -X POST https://localhost:5000/mode \
  -H "Content-Type: application/json" \
  -u admin:admin \
  -d '{"mode": "sim"}'
```

Inject Sensor Fault
```bash
curl -X POST https://localhost:5000/fault \
  -H "Content-Type: application/json" \
  -u admin:admin \
  -d '{
    "type": "sensor_spike",
    "duration": 10.0,
    "params": {
      "sensor": "altitude",
      "magnitude": 100.0
    }
  }'
```

## Development

### Adding New Sensors
1. Extend `SensorManager` class in `sensor_node/sensor_manager.py`
2. Add sensor initialization in `initialize()` method
3. Update `_read_hardware_sensors()` to include new data
4. Modify telemetry schema in related components

### Creating Custom Faults
1. Add new fault type to `fault_injector/fault_types.py`
2. Implement fault logic in `FaultGenerator.apply_fault()`
3. Update configuration schema in fault_config.json
4. Add fault to dashboard visualizations

### Custom Dashboards
1. Create JSON dashboard definition in `dashboards/`
2. Import via Grafana UI or API
3. Use Prometheus metrics for data sources
4. Add alerting rules for critical conditions

## Troubleshooting

### Common Issues

MQTT Connection Failed
- Check certificate paths and permissions
- Verify MQTT broker is running: `docker-compose ps`
- Test connectivity: `mosquitto_pub -h localhost -p 8883 -t test -m hello`

**Sensor Initialization Error
- Ensure I2C is enabled on Raspberry Pi
- Check wiring connections and power supply
- Verify sensor addresses with `i2cdetect -y 1`

**Dashboard Not Loading**
- Check Grafana service status
- Verify Prometheus is scraping metrics
- Review logs: `docker logs rocket-grafana`

**API Authentication Issues
- Check username/password configuration
- Verify TLS certificates are valid
- Review API server logs for details

### Log Locations
- Application logs: `/var/log/rocket_*.log`
- Docker logs: `docker logs <container_name>`
- System logs: `journalctl -u rocket-telemetry`

## Security Considerations

### Production Deployment
- Change default passwords and certificates
- Use strong authentication mechanisms
- Enable firewall rules for necessary ports only
- Regular security updates and monitoring
- Implement backup and recovery procedures

### Network Security
- Use VPN for remote access
- Implement network segmentation
- Monitor for unauthorized access attempts
- Regular certificate rotation

## Performance Tuning

### Telemetry Rate Optimization
- Adjust sample rates based on flight requirements
- Use data compression for bandwidth-limited links
- Implement adaptive sample rates based on flight phase
- Buffer data during communication outages

### Resource Management
- Monitor CPU and memory usage
- Implement log rotation and cleanup
- Use efficient data storage formats
- Optimize database queries and indexing

## Contributing

1. Fork the repository
2. Create a feature branch
3. Implement changes with tests
4. Update documentation
5. Submit pull request

### Development Setup
```bash
# Install development dependencies
pip install -r requirements-dev.txt

# Run tests
python -m pytest tests/

# Check code quality
flake8 --config .flake8
black --check .
```
