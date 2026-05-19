#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>
#include <mosquitto.h>
#include <curl/curl.h>

// Performance tuning
#define CAPTURE_INTERVAL 50000        // 50ms = 20 FPS - Sensor polling rate in microseconds. 
                                      // Lower = more responsive but higher CPU usage.
                                      // 50000µs = 20 captures/second. Min: 10000, Max: 100000.

#define BASELINE_SAMPLES 10           // Calibration sample count at startup.
                                      // More samples = more accurate baseline brightness.
                                      // Used to establish "no finger" reference level.
                                      // Range: 5-50. Higher = longer calibration time.

#define FINGER_THRESHOLD 15.0         // Brightness change threshold to detect touch.
                                      // |current - baseline| > 15 = finger detected.
                                      // 0-255 scale (8-bit pixel values).
                                      // Lower = more sensitive, may cause false triggers.
                                      // Higher = requires firmer press. Range: 5-50.

#define MULTI_TAP_TIMEOUT 0.5         // Max delay between taps in seconds.
                                      // If next tap doesn't occur within 0.5s,
                                      // current tap sequence is processed.
                                      // 0.3-0.7s typical. Lower = faster response but
                                      // harder to perform multi-taps.

#define SMOOTHING_FRAMES 1            // Number of frames to average for noise reduction.
                                      // 1 = no smoothing (raw values).
                                      // 2-5 = smoother but adds slight latency.
                                      // Each frame = +50ms delay (based on CAPTURE_INTERVAL).
                                      // Trade-off: smoothness vs responsiveness.
#define DEBUG 1

// Connection config
#define MQTT_HOST "localhost"
#define MQTT_PORT 1883
#define MQTT_USERNAME "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"
#define MQTT_TOPIC "fingerprint/action"

#define HTTP_HOST "localhost"
#define HTTP_PORT 8123
#define HTTP_TOKEN "your_long_lived_token_here"
#define CONNECTION_TYPE 0             // 0 = MQTT, 1 = HTTP

// USB IDs
#define VENDOR_ID  0x2808
#define PRODUCT_ID 0xc652
#define OUT_ENDPOINT 0x01
#define IN_ENDPOINT 0x82

// Image parameters
#define IMAGE_WIDTH 64
#define IMAGE_HEIGHT 80
#define HEADER_OFFSET 4
#define PIXEL_STRIDE 2
#define RESPONSE_ACK_LENGTH 7
#define IMAGE_DATA_LENGTH 10246
#define NUM_PIXELS (IMAGE_WIDTH * IMAGE_HEIGHT)
#define MAX_TAP_SEQUENCE 10

// FocalTech commands
static const unsigned char CMD_STATUS[]      = {0x37, 0x01, 0x01, 0x01};
static const unsigned char CMD_PREPARE[]     = {0x80, 0x02, 0x01};
static const unsigned char CMD_CAPTURE[]     = {0x82, 0x73, 0x01};
static const unsigned char CMD_IMAGE[]       = {0x81};

// Globals
static struct mosquitto *mosq = NULL;
static int mqtt_connected = 0;
static CURL *curl = NULL;
static double baseline_brightness = 0.0;
static int is_touching = 0;
static double tap_times[MAX_TAP_SEQUENCE];
static int tap_count = 0;
static double brightness_history[SMOOTHING_FRAMES];
static int history_index = 0;
static int history_filled = 0;

// Debug macro
#define DEBUG_PRINT(fmt, ...) do { if (DEBUG) printf(fmt, ##__VA_ARGS__); } while(0)

static inline double get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static size_t write_callback(void *contents, size_t size, size_t nmemb, void *userp) {
    (void)contents; (void)userp;
    return size * nmemb;
}

static inline unsigned char checksum(const unsigned char *data, int len) {
    unsigned char cs = (unsigned char)len;
    for (int i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

static int send_cmd(libusb_device_handle *h, const unsigned char *payload, int len) {
    unsigned char buf[64];
    int transferred;
    buf[0] = 0x02; buf[1] = 0x00; buf[2] = (unsigned char)len;
    memcpy(buf + 3, payload, len);
    buf[3 + len] = checksum(payload, len);
    return libusb_bulk_transfer(h, OUT_ENDPOINT, buf, 3 + len + 1, &transferred, 1000);
}

static int read_resp(libusb_device_handle *h, unsigned char *data, int len) {
    int transferred;
    return libusb_bulk_transfer(h, IN_ENDPOINT, data, len, &transferred, 500) < 0 ? -1 : transferred;
}

static int capture_frame(libusb_device_handle *h, unsigned char *pixels) {
    unsigned char resp[IMAGE_DATA_LENGTH];
    if (send_cmd(h, CMD_STATUS, sizeof(CMD_STATUS)) < 0) return -1;
    if (read_resp(h, resp, RESPONSE_ACK_LENGTH) < 0) return -1;
    if (send_cmd(h, CMD_PREPARE, sizeof(CMD_PREPARE)) < 0) return -1;
    if (read_resp(h, resp, RESPONSE_ACK_LENGTH) < 0) return -1;
    if (send_cmd(h, CMD_CAPTURE, sizeof(CMD_CAPTURE)) < 0) return -1;
    if (read_resp(h, resp, RESPONSE_ACK_LENGTH) < 0) return -1;
    if (send_cmd(h, CMD_IMAGE, sizeof(CMD_IMAGE)) < 0) return -1;
    int bytes = read_resp(h, resp, IMAGE_DATA_LENGTH);
    if (bytes < 0) return -1;
    for (int i = 0; i < NUM_PIXELS; i++) {
        int off = HEADER_OFFSET + i * PIXEL_STRIDE;
        pixels[i] = (off + 1 < bytes) ? resp[off] : 0;
    }
    return 0;
}

static inline double avg_brightness(unsigned char *pixels) {
    double sum = 0.0;
    for (int i = 0; i < NUM_PIXELS; i++) sum += pixels[i];
    return sum / NUM_PIXELS;
}

static inline double smooth(double current) {
    brightness_history[history_index] = current;
    history_index = (history_index + 1) % SMOOTHING_FRAMES;
    if (!history_filled && history_index == 0) history_filled = 1;
    int count = history_filled ? SMOOTHING_FRAMES : history_index;
    double sum = 0.0;
    for (int i = 0; i < count; i++) sum += brightness_history[i];
    return sum / count;
}

static const char* tap_to_string(int count) {
    switch(count) {
        case 1: return "one";
        case 2: return "two";
        case 3: return "three";
        case 4: return "four";
        case 5: return "five";
        default: return NULL;
    }
}

static int init_http(void) {
    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if (!curl) { fprintf(stderr, "HTTP init failed\n"); return -1; }
    DEBUG_PRINT("HTTP client initialized\n");
    return 0;
}

static void http_post(const char *payload) {
    if (!curl) return;
    char url[512];
    struct curl_slist *headers = NULL;
    snprintf(url, sizeof(url), "http://%s:%d/api/webhook/%s", HTTP_HOST, HTTP_PORT, payload);
    snprintf(url, sizeof(url), "http://%s:%d/api/webhook/fingerprint_%s", HTTP_HOST, HTTP_PORT, payload);
    headers = curl_slist_append(headers, "Content-Type: application/json");
    char auth[256];
    snprintf(auth, sizeof(auth), "Authorization: Bearer %s", HTTP_TOKEN);
    headers = curl_slist_append(headers, auth);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "{}");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
    CURLcode res = curl_easy_perform(curl);
    DEBUG_PRINT("  HTTP: %s [%s]\n", payload, res == CURLE_OK ? "OK" : curl_easy_strerror(res));
    curl_slist_free_all(headers);
}

static int init_mqtt(void) {
    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) { fprintf(stderr, "MQTT create failed\n"); return -1; }
    if (strlen(MQTT_USERNAME) > 0) mosquitto_username_pw_set(mosq, MQTT_USERNAME, MQTT_PASSWORD);
    if (mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, 60)) { fprintf(stderr, "MQTT connect failed\n"); return -1; }
    mqtt_connected = 1;
    DEBUG_PRINT("MQTT connected to %s:%d\n", MQTT_HOST, MQTT_PORT);
    return 0;
}

static void mqtt_publish(const char *payload) {
    if (!mqtt_connected || !mosq) return;
    mosquitto_publish(mosq, NULL, MQTT_TOPIC, strlen(payload), payload, 0, false);
    DEBUG_PRINT("  MQTT: %s -> %s\n", MQTT_TOPIC, payload);
}

static void send_action(const char *payload) {
    if (CONNECTION_TYPE == 0) mqtt_publish(payload);
    else http_post(payload);
}

static void process_taps(void) {
    if (tap_count == 0) return;
    const char *action = tap_to_string(tap_count);
    if (action) {
        DEBUG_PRINT("\n    Processing %d tap(s) -> %s\n", tap_count, action);
        send_action(action);
    } else {
        DEBUG_PRINT("\n    No action for %d taps\n", tap_count);
    }
    tap_count = 0;
}

static int calibrate(libusb_device_handle *h) {
    unsigned char pixels[NUM_PIXELS];
    double sum = 0.0;
    int samples = 0;
    DEBUG_PRINT("Calibrating baseline (%d samples)...\n", BASELINE_SAMPLES);
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
        if (capture_frame(h, pixels) == 0) {
            double b = avg_brightness(pixels);
            sum += b;
            samples++;
            DEBUG_PRINT("  Sample %d: %.1f\n", i + 1, b);
        }
        struct timespec ts = {0, 100000000}; nanosleep(&ts, NULL);
    }
    if (!samples) { fprintf(stderr, "No calibration samples!\n"); return -1; }
    baseline_brightness = sum / samples;
    DEBUG_PRINT("Baseline: %.1f\n\n", baseline_brightness);
    return 0;
}

static void *sensor_thread(void *arg) {
    libusb_device_handle *h = (libusb_device_handle *)arg;
    unsigned char pixels[NUM_PIXELS];
    DEBUG_PRINT("Monitoring started. Threshold: %.1f, Multi-tap timeout: %.1fs\n\n", FINGER_THRESHOLD, MULTI_TAP_TIMEOUT);
    
    while (1) {
        if (capture_frame(h, pixels) < 0) {
            struct timespec ts = {0, CAPTURE_INTERVAL * 1000}; nanosleep(&ts, NULL);
            continue;
        }
        
        double brightness = smooth(avg_brightness(pixels));
        double delta = fabs(brightness - baseline_brightness);
        double now = get_time();
        
        if (tap_count > 0 && !is_touching && now - tap_times[tap_count - 1] > MULTI_TAP_TIMEOUT)
            process_taps();
        
        if (!is_touching && delta > FINGER_THRESHOLD) {
            is_touching = 1;
            DEBUG_PRINT("[TOUCH] Tap #%d | Brightness: %.1f (Δ=%.1f)\n", tap_count + 1, brightness, delta);
        }
        
        if (is_touching && delta <= FINGER_THRESHOLD) {
            is_touching = 0;
            if (tap_count < MAX_TAP_SEQUENCE) {
                tap_times[tap_count] = now;
                tap_count++;
                DEBUG_PRINT("         Released -> Tap #%d\n", tap_count);
            }
        }
        
        struct timespec ts = {0, CAPTURE_INTERVAL * 1000}; nanosleep(&ts, NULL);
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    (void)argc; (void)argv;
    libusb_device_handle *handle = NULL;
    
    printf("\nFingerprint Sensor\n");
    printf("Device: %04x:%04x | Protocol: %s | Debug: %s\n\n", 
           VENDOR_ID, PRODUCT_ID, CONNECTION_TYPE == 0 ? "MQTT" : "HTTP", DEBUG ? "ON" : "OFF");
    
    if (libusb_init(NULL) < 0) { fprintf(stderr, "libusb init failed\n"); return 1; }
    handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!handle) { fprintf(stderr, "Device not found\n"); libusb_exit(NULL); return 1; }
    DEBUG_PRINT("Device found\n");
    
    if (libusb_kernel_driver_active(handle, 0)) libusb_detach_kernel_driver(handle, 0);
    if (libusb_claim_interface(handle, 0) < 0) { fprintf(stderr, "Interface claim failed\n"); libusb_close(handle); libusb_exit(NULL); return 1; }
    libusb_reset_device(handle);
    
    if (CONNECTION_TYPE == 0) init_mqtt(); else init_http();
    if (calibrate(handle) < 0) { libusb_release_interface(handle, 0); libusb_close(handle); libusb_exit(NULL); return 1; }
    
    pthread_t thread;
    pthread_create(&thread, NULL, sensor_thread, handle);
    pthread_join(thread, NULL);
    
    if (mqtt_connected) { mosquitto_disconnect(mosq); mosquitto_destroy(mosq); mosquitto_lib_cleanup(); }
    if (curl) { curl_easy_cleanup(curl); curl_global_cleanup(); }
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(NULL);
    return 0;
}
