#!/bin/bash
# Setup development environment for rocket telemetry system

set -e

echo "🚀 Setting up Rocket Telemetry Pipeline development environment..."

# Check if running as root for system-level installs
if [[ $EUID -eq 0 ]]; then
   echo "Don't run this script as root for safety"
   exit 1
fi

# Update system packages
echo "Updating system packages..."
sudo apt-get update

# Install required system packages
echo "Installing system dependencies..."
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    docker.io \
    docker-compose \
    git \
    curl \
    wget \
    openssl \
    sqlite3 \
    mosquitto-clients \
    net-tools \
    htop \
    tree

# Install Docker if not present
if ! command -v docker &> /dev/null; then
    echo "Installing Docker..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    rm get-docker.sh
fi

# Install Docker Compose if not present
if ! command -v docker-compose &> /dev/null; then
    echo "Installing Docker Compose..."
    sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
fi

# Create project directories
echo "Creating project directories..."
mkdir -p {hardware/sensor-node,hardware/ground-station,hardware/fault-injector}
mkdir -p {infrastructure/mosquitto/certs,infrastructure/nginx/{certs,private}}
mkdir -p {dashboards,docs,scripts,tests}
mkdir -p logs/{sensor_node,hil_simulation,fault_injector,ground_station}

# Setup Python virtual environments
echo "Setting up Python virtual environments..."

for component in "hardware/sensor-node" "hardware/ground-station" "hardware/fault-injector"; do
    echo "Setting up $component environment..."
    cd $component
    python3 -m venv venv
    source venv/bin/activate
    pip install --upgrade pip
    
    # Install component-specific requirements
    if [ -f requirements.txt ]; then
        pip install -r requirements.txt
    fi
    
    deactivate
    cd - > /dev/null
done

# Create systemd service files for automatic startup
echo "Creating systemd service files..."

sudo tee /etc/systemd/system/rocket-telemetry.service > /dev/null <<EOF
[Unit]
Description=Rocket Telemetry System
After=network.target docker.service
Requires=docker.service

[Service]
Type=forking
User=$USER
Group=$USER
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/scripts/start_services.sh
ExecStop=$(pwd)/scripts/stop_services.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Set up log rotation
echo "Configuring log rotation..."
sudo tee /
sudo tee /etc/logrotate.d/rocket-telemetry > /dev/null <<EOF
/var/log/rocket_*.log {
   daily
   missingok
   rotate 30
   compress
   delaycompress
   notifempty
   sharedscripts
   postrotate
       systemctl reload rocket-telemetry || true
   endscript
}
EOF

# Create SSL directory and set permissions
sudo mkdir -p /etc/ssl/{certs,private}
sudo chown -R $USER:$USER /etc/ssl/{certs,private}

# Create data directories
sudo mkdir -p /var/lib/rocket-telemetry
sudo mkdir -p /var/log/rocket-telemetry
sudo chown -R $USER:$USER /var/lib/rocket-telemetry /var/log/rocket-telemetry

echo "Environment setup complete!"
echo ""
echo "Next steps:"
echo "1. Run ./scripts/security/generate-certificates.sh to create TLS certificates"
echo "2. Run docker-compose up -d in the infrastructure directory"
echo "3. Start individual telemetry nodes as needed"
echo ""
echo "Note: You may need to log out and back in for Docker group changes to take effect"
