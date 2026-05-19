# FocalTech Automation

Turn a FocalTech 2808:c652 USB fingerprint sensor into a multi-tap smart home controller via MQTT.

## Features

- Multi-tap detection (1-5 taps)
- MQTT integration

## Hardware

- FocalTech 2808:c652 USB fingerprint sensor

## Dependencies

**Debian:**
```bash
sudo apt install libusb-1.0-0-dev libmosquitto-dev libcurl4-openssl-dev
```

**Arch Linux:**
```bash
sudo pacman -S libusb mosquitto curl
```

**Fedora:**
```bash
sudo dnf install libusb1-devel mosquitto-devel libcurl-devel
```

## Quick Start

### 1. Clone
```bash
git clone https://github.com/YOUR_USER/focaltech-automation.git
cd focaltech-automation
```

### 2. Configure
```bash
cd src
nano .env
```

Edit these lines:

```c
CAPTURE_INTERVAL=50000
BASELINE_SAMPLES=10
FINGER_THRESHOLD=15.0
MULTI_TAP_TIMEOUT=0.5
SMOOTHING_FRAMES=1

MQTT_HOST=localhost
MQTT_PORT=1883
MQTT_USERNAME=your_mqtt_username
MQTT_PASSWORD=your_mqtt_password
MQTT_TOPIC=fingerprint/action
MQTT_KEEPALIVE=60
MQTT_RECONNECT_DELAY=5
```

### 3. Build
```bash
cd ..
make
```

### 4. Run
```bash
sudo ./fingerprint
```

### 5. Setup autostart
```bash
sudo ./setup.sh
```
