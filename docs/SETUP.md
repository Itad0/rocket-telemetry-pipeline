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

### 2. System Preparation 

# Clone repository
git clone https://github.com/your-org/rocket-telemetry-pipeline.git
cd rocket-telemetry-pipeline

# Make scripts executable
chmod +x scripts/*.sh

# Run environment setup
./scripts/setup/setup-development.sh
