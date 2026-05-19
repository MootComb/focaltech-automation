#!/bin/bash

# Fingerprint Smart Button - Setup Script
SRC_DIR="src"
INSTALL_DIR="/opt/smarthome"
SERVICE_NAME="fingerprint-automation"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo ./setup.sh)"
    exit 1
fi

# Create install directory
mkdir -p "$INSTALL_DIR"

# Copy files
echo "Installing to $INSTALL_DIR..."
cp "$SRC_DIR/fingerprint" "$INSTALL_DIR/"
cp "$SRC_DIR/.env" "$INSTALL_DIR/"
chmod +x "$INSTALL_DIR/fingerprint"
echo "Done."

# Systemd service
if [ -f "$SERVICE_FILE" ]; then
    read -p "Service exists. Remove systemd service? [y/N]: " remove_service
    if [ "$remove_service" = "y" ] || [ "$remove_service" = "Y" ]; then
        systemctl stop "$SERVICE_NAME"
        systemctl disable "$SERVICE_NAME"
        rm "$SERVICE_FILE"
        systemctl daemon-reload
        echo "Service removed."
    fi
else
    read -p "Create systemd service (auto-start on boot)? [y/N]: " create_service
    if [ "$create_service" = "y" ] || [ "$create_service" = "Y" ]; then
        cat > "$SERVICE_FILE" << EOF
[Unit]
Description=Fingerprint Smart Button Automation
After=network.target mosquitto.service
Wants=network.target mosquitto.service

[Service]
Type=simple
User=root
WorkingDirectory=$INSTALL_DIR
ExecStart=$INSTALL_DIR/fingerprint
Restart=always
RestartSec=10
EnvironmentFile=$INSTALL_DIR/.env

[Install]
WantedBy=multi-user.target
EOF
        systemctl daemon-reload
        systemctl enable "$SERVICE_NAME"
        systemctl start "$SERVICE_NAME"
        echo "Service created and started."
    fi
fi

echo "Setup complete."
