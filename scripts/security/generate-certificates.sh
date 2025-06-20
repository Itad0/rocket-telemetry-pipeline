#!/bin/bash
# Generate TLS certificates for secure MQTT and API communication

set -e

CERT_DIR="infrastructure/mosquitto/certs"
NGINX_CERT_DIR="infrastructure/nginx/certs"
NGINX_KEY_DIR="infrastructure/nginx/private"

# Create directories
mkdir -p "$CERT_DIR"
mkdir -p "$NGINX_CERT_DIR"
mkdir -p "$NGINX_KEY_DIR"

echo "Generating CA certificate..."

# Generate CA private key
openssl genrsa -out "$CERT_DIR/ca.key" 4096

# Generate CA certificate
openssl req -new -x509 -days 365 -key "$CERT_DIR/ca.key" -out "$CERT_DIR/ca.crt" \
    -subj "/C=US/ST=NY/L=Dunkirk/O=Rocket Telemetry/OU=CA/CN=rocket-ca"

echo "Generating server certificate..."

# Generate server private key
openssl genrsa -out "$CERT_DIR/server.key" 2048

# Generate server certificate signing request
openssl req -new -key "$CERT_DIR/server.key" -out "$CERT_DIR/server.csr" \
    -subj "/C=US/ST=NY/L=Dunkirk/O=Rocket Telemetry/OU=Server/CN=localhost"

# Generate server certificate
openssl x509 -req -in "$CERT_DIR/server.csr" -CA "$CERT_DIR/ca.crt" -CAkey "$CERT_DIR/ca.key" \
    -CAcreateserial -out "$CERT_DIR/server.crt" -days 365 \
    -extensions v3_req -extfile <(cat <<EOF
[v3_req]
keyUsage = keyEncipherment, dataEncipherment
extendedKeyUsage = serverAuth
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
DNS.2 = rocket.local
DNS.3 = grafana.rocket.local
DNS.4 = prometheus.rocket.local
IP.1 = 127.0.0.1
IP.2 = 192.168.1.100
EOF
)

echo "Generating client certificates..."

# Generate client private key
openssl genrsa -out "$CERT_DIR/client.key" 2048

# Generate client certificate signing request
openssl req -new -key "$CERT_DIR/client.key" -out "$CERT_DIR/client.csr" \
    -subj "/C=US/ST=NY/L=Dunkirk/O=Rocket Telemetry/OU=Client/CN=rocket-client"

# Generate client certificate
openssl x509 -req -in "$CERT_DIR/client.csr" -CA "$CERT_DIR/ca.crt" -CAkey "$CERT_DIR/ca.key" \
    -CAcreateserial -out "$CERT_DIR/client.crt" -days 365

# Copy certificates for Nginx
cp "$CERT_DIR/server.crt" "$NGINX_CERT_DIR/"
cp "$CERT_DIR/server.key" "$NGINX_KEY_DIR/"
cp "$CERT_DIR/ca.crt" "$NGINX_CERT_DIR/"

# Set appropriate permissions
chmod 600 "$CERT_DIR"/*.key "$NGINX_KEY_DIR"/*.key
chmod 644 "$CERT_DIR"/*.crt "$NGINX_CERT_DIR"/*.crt

echo "Certificates generated successfully!"
echo "CA certificate: $CERT_DIR/ca.crt"
echo "Server certificate: $CERT_DIR/server.crt"
echo "Client certificate: $CERT_DIR/client.crt"

# Generate MQTT password file
echo "Generating MQTT password file..."
docker run --rm -v "$(pwd)/infrastructure/mosquitto:/mosquitto/config" eclipse-mosquitto:2.0 \
    mosquitto_passwd -c -b /mosquitto/config/passwd rocket_node secure_password

docker run --rm -v "$(pwd)/infrastructure/mosquitto:/mosquitto/config" eclipse-mosquitto:2.0 \
    mosquitto_passwd -b /mosquitto/config/passwd hil_simulator secure_password

docker run --rm -v "$(pwd)/infrastructure/mosquitto:/mosquitto/config" eclipse-mosquitto:2.0 \
    mosquitto_passwd -b /mosquitto/config/passwd fault_injector secure_password

docker run --rm -v "$(pwd)/infrastructure/mosquitto:/mosquitto/config" eclipse-mosquitto:2.0 \
    mosquitto_passwd -b /mosquitto/config/passwd ground_station secure_password

echo "MQTT authentication configured!"
