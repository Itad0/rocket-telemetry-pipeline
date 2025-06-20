# Complete Setup Guide

## Prerequisites

### Hardware Requirements
- **Development Machine**: Laptop/desktop with 8GB+ RAM, 50GB free space
- **Rocket Platform**: Raspberry Pi Pico W or Pi Zero 2 W
- **Sensors**: BME688, MPU6050 
- **Ground Station**: Raspberry Pi 4/5 or x86 computer
- **Network**: Wi-Fi router with internet access

### Software Requirements
- Ubuntu 20.04+ or Raspberry Pi OS
- Python 3.8+
- Docker and Docker Compose
- Git

## Step-by-Step Installation

### 1. System Preparation

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install base packages
sudo apt install -y git curl wget python3 python3-pip python3-venv \
    build-essential libssl-dev sqlite3 mosquitto-clients

# Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in for group changes
```

### 2. Project Setup

```bash
# Clone repository
git clone https://github.com/your-org/rocket-telemetry-pipeline.git
cd rocket-telemetry-pipeline

# Make scripts executable
chmod +x scripts/*.sh

# Run environment setup
./scripts/setup/setup-development.sh
```

### 3. Certificate Generation

```bash
# Generate TLS certificates
./scripts/security/generate-certificates.sh

# Verify certificates
openssl x509 -in infrastructure/mosquitto/certs/server.crt -text -noout
```

### 4. Infrastructure Development

```bash
# Start all services
cd infrastructure
docker-compose up -d

# Check service status
docker-compose ps
docker-compose logs grafana
```

### 5. Component Testing

```bash
# HIL Simulator

cd hardware/ground-station
source venv/bin/activate
python hil_simulator.py
# Should see telemetry messages in MQTT and Grafana
```

# Ground Station API

```bash
cd hardware/ground-station
source venv/bin/activate
python api_server.py
# Test API: curl -k https://localhost:5000/health
```

### Hardware Configuration

```bash
# Sensor Node Setup

# Flash MicroPython to Pico W
# Copy hardware/sensor-node/ files via Thonny
# Configure WiFi in config.json
```

# Network Setup

```bash
# Configure Pi WiFi
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

network={
    ssid="YourNetworkName"
    psk="YourPassword"
}
```

# Validation Testing
```bash
# Run tests
python -m pytest tests/ -v

# End-to-end telemetry flow
./scripts/testing/run-integration-tests.sh
```

# Troubleshooting

### Common Issues

Docker permission denied: sudo usermod -aG docker $USER
Certificate errors: Regenerate with ./scripts/security/generate-certificates.sh
Port conflicts: Check with sudo netstat -tulpn

For detailed troubleshooting, see TROUBLESHOOTING.md
