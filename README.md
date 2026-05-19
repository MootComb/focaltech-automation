# FocalTech Automation

Turn a FocalTech 2808:c652 USB fingerprint sensor into a multi-tap smart home controller via MQTT or HTTP.

## Features

- 👆 Multi-tap detection (1-5 taps)
- 🔌 Homeassistant integration

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
nano fingerprint.c
```

Edit these lines:

```c
#define CONNECTION_TYPE 0             // 0 = MQTT, 1 = HTTP

#define MQTT_HOST "localhost"
#define MQTT_PORT 1883
#define MQTT_USERNAME "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"
#define MQTT_TOPIC "fingerprint/action"

#define HTTP_HOST "localhost"
#define HTTP_PORT 8123
#define HTTP_TOKEN "your_long_lived_token_here"
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
