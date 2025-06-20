#!/bin/bash
# Deploy ground station to Raspberry Pi

set -e

if [ $# -eq 0 ]; then
    echo "Usage: $0 <pi-username@pi-ip>"
    echo "Example: $0 pi@192.168.1.100"
    exit 1
fi

PI_HOST=$1
PROJECT_DIR="rocket-telemetry-pipeline"

echo "🚀 Deploying Ground Station to $PI_HOST..."

# Create deployment package
echo "📦 Creating deployment package..."
tar -czf ground-station-deploy.tar.gz \
    hardware/ground-station/ \
    infrastructure/ \
    scripts/ \
    dashboards/ \
    --exclude="*/venv/*" \
    --exclude="*/__pycache__/*" \
    --exclude="*.pyc"

# Transfer to Pi
echo "📡 Transferring files to Raspberry Pi..."
scp ground-station-deploy.tar.gz $PI_HOST:~/

# Execute deployment on Pi
echo "🔧 Installing on Raspberry Pi..."
ssh $PI_HOST << 'EOF'
    # Extract files
    tar -xzf ground-station-deploy.tar.gz
    
    # Update system
    sudo apt update
    
    # Install Docker if needed
    if ! command -v docker &> /dev/null; then
        curl -fsSL https://get.docker.com -o get-docker.sh
        sudo sh get-docker.sh
        sudo usermod -aG docker pi
    fi
    
    # Install Docker Compose
    if ! command -v docker-compose &> /dev/null; then
        sudo pip3 install docker-compose
    fi
    
    # Setup Python environment
    cd hardware/ground-station
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
    deactivate
    cd ../..
    
    # Start infrastructure
    cd infrastructure
    docker-compose up -d
    cd ..
    
    # Create systemd service
    sudo tee /etc/systemd/system/ground-station.service > /dev/null <<SERVICE_EOF
[Unit]
Description=Rocket Telemetry Ground Station
After=network.target docker.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/hardware/ground-station
ExecStart=/home/pi/hardware/ground-station/venv/bin/python api_server.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
SERVICE_EOF
    
    # Enable and start service
    sudo systemctl enable ground-station
    sudo systemctl start ground-station
    
    echo "✅ Ground station deployment complete!"
    echo "🌐 Access Grafana at: http://$(hostname -I | awk '{print $1}'):3000"
    echo "🔧 API available at: http://$(hostname -I | awk '{print $1}'):5000"
EOF

# Cleanup
rm ground-station-deploy.tar.gz

echo "🎉 Deployment completed successfully!"
