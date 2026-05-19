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

#define DEBUG 1

#define DEFAULT_CAPTURE_INTERVAL 50000
#define DEFAULT_BASELINE_SAMPLES 10
#define DEFAULT_FINGER_THRESHOLD 15.0
#define DEFAULT_MULTI_TAP_TIMEOUT 0.5
#define DEFAULT_SMOOTHING_FRAMES 1
#define DEFAULT_MQTT_HOST "localhost"
#define DEFAULT_MQTT_PORT 1883
#define DEFAULT_MQTT_USERNAME ""
#define DEFAULT_MQTT_PASSWORD ""
#define DEFAULT_MQTT_TOPIC "fingerprint/action"
#define DEFAULT_MQTT_KEEPALIVE 60
#define DEFAULT_MQTT_RECONNECT_DELAY 5

static int CAPTURE_INTERVAL = DEFAULT_CAPTURE_INTERVAL;
static int BASELINE_SAMPLES = DEFAULT_BASELINE_SAMPLES;
static double FINGER_THRESHOLD = DEFAULT_FINGER_THRESHOLD;
static double MULTI_TAP_TIMEOUT = DEFAULT_MULTI_TAP_TIMEOUT;
static int SMOOTHING_FRAMES = DEFAULT_SMOOTHING_FRAMES;
static char MQTT_HOST[256] = DEFAULT_MQTT_HOST;
static int MQTT_PORT = DEFAULT_MQTT_PORT;
static char MQTT_USERNAME[128] = DEFAULT_MQTT_USERNAME;
static char MQTT_PASSWORD[128] = DEFAULT_MQTT_PASSWORD;
static char MQTT_TOPIC[256] = DEFAULT_MQTT_TOPIC;
static int MQTT_KEEPALIVE = DEFAULT_MQTT_KEEPALIVE;
static int MQTT_RECONNECT_DELAY = DEFAULT_MQTT_RECONNECT_DELAY;

#define VENDOR_ID  0x2808
#define PRODUCT_ID 0xc652
#define OUT_ENDPOINT 0x01
#define IN_ENDPOINT 0x82

#define IMAGE_WIDTH 64
#define IMAGE_HEIGHT 80
#define HEADER_OFFSET 4
#define PIXEL_STRIDE 2
#define RESPONSE_ACK_LENGTH 7
#define IMAGE_DATA_LENGTH 10246
#define NUM_PIXELS (IMAGE_WIDTH * IMAGE_HEIGHT)
#define MAX_TAP_SEQUENCE 10

static const unsigned char CMD_STATUS[]  = {0x37, 0x01, 0x01, 0x01};
static const unsigned char CMD_PREPARE[] = {0x80, 0x02, 0x01};
static const unsigned char CMD_CAPTURE[] = {0x82, 0x73, 0x01};
static const unsigned char CMD_IMAGE[]   = {0x81};

static struct mosquitto *mosq = NULL;
static volatile int mqtt_connected = 0;
static volatile int running = 1;
static double baseline_brightness = 0.0;
static int is_touching = 0;
static double tap_times[MAX_TAP_SEQUENCE];
static int tap_count = 0;
static double *brightness_history = NULL;
static int history_index = 0;
static int history_filled = 0;
static pthread_mutex_t mqtt_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t tap_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DEBUG_PRINT(fmt, ...) do { if (DEBUG) printf(fmt, ##__VA_ARGS__); } while(0)

static char* trim(char *str) {
    while (*str == ' ' || *str == '\t') str++;
    char *end = str + strlen(str) - 1;
    while (end > str && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
        *end-- = '\0';
    }
    return str;
}

static int load_env(const char *filename) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        DEBUG_PRINT("No .env file found at %s, using defaults\n", filename);
        return 0;
    }
    
    char line[512];
    int loaded = 0;
    
    while (fgets(line, sizeof(line), file)) {
        char *trimmed = trim(line);
        if (trimmed[0] == '#' || trimmed[0] == '\0') continue;
        
        char *equals = strchr(trimmed, '=');
        if (!equals) continue;
        
        *equals = '\0';
        char *key = trim(trimmed);
        char *value = trim(equals + 1);
        
        if (value[0] == '"' || value[0] == '\'') {
            value++;
            value[strlen(value) - 1] = '\0';
        }
        
        if (strcmp(key, "CAPTURE_INTERVAL") == 0) {
            CAPTURE_INTERVAL = atoi(value);
            loaded++;
        }
        else if (strcmp(key, "BASELINE_SAMPLES") == 0) {
            BASELINE_SAMPLES = atoi(value);
            loaded++;
        }
        else if (strcmp(key, "FINGER_THRESHOLD") == 0) {
            FINGER_THRESHOLD = atof(value);
            loaded++;
        }
        else if (strcmp(key, "MULTI_TAP_TIMEOUT") == 0) {
            MULTI_TAP_TIMEOUT = atof(value);
            loaded++;
        }
        else if (strcmp(key, "SMOOTHING_FRAMES") == 0) {
            SMOOTHING_FRAMES = atoi(value);
            loaded++;
        }
        else if (strcmp(key, "MQTT_HOST") == 0) {
            strncpy(MQTT_HOST, value, sizeof(MQTT_HOST) - 1);
            MQTT_HOST[sizeof(MQTT_HOST) - 1] = '\0';
            loaded++;
        }
        else if (strcmp(key, "MQTT_PORT") == 0) {
            MQTT_PORT = atoi(value);
            loaded++;
        }
        else if (strcmp(key, "MQTT_USERNAME") == 0) {
            strncpy(MQTT_USERNAME, value, sizeof(MQTT_USERNAME) - 1);
            MQTT_USERNAME[sizeof(MQTT_USERNAME) - 1] = '\0';
            loaded++;
        }
        else if (strcmp(key, "MQTT_PASSWORD") == 0) {
            strncpy(MQTT_PASSWORD, value, sizeof(MQTT_PASSWORD) - 1);
            MQTT_PASSWORD[sizeof(MQTT_PASSWORD) - 1] = '\0';
            loaded++;
        }
        else if (strcmp(key, "MQTT_TOPIC") == 0) {
            strncpy(MQTT_TOPIC, value, sizeof(MQTT_TOPIC) - 1);
            MQTT_TOPIC[sizeof(MQTT_TOPIC) - 1] = '\0';
            loaded++;
        }
        else if (strcmp(key, "MQTT_KEEPALIVE") == 0) {
            MQTT_KEEPALIVE = atoi(value);
            loaded++;
        }
        else if (strcmp(key, "MQTT_RECONNECT_DELAY") == 0) {
            MQTT_RECONNECT_DELAY = atoi(value);
            loaded++;
        }
    }
    
    fclose(file);
    DEBUG_PRINT("Loaded %d settings from %s\n", loaded, filename);
    return loaded;
}

static inline double get_time(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
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

static void mqtt_publish(const char *payload) {
    pthread_mutex_lock(&mqtt_mutex);
    if (!mqtt_connected || !mosq) {
        pthread_mutex_unlock(&mqtt_mutex);
        return;
    }
    int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC, strlen(payload), payload, 0, false);
    if (ret == MOSQ_ERR_SUCCESS) {
        DEBUG_PRINT("  MQTT: %s -> %s\n", MQTT_TOPIC, payload);
    } else if (ret == MOSQ_ERR_NO_CONN) {
        DEBUG_PRINT("  MQTT publish failed: no connection\n");
        mqtt_connected = 0;
    } else {
        DEBUG_PRINT("  MQTT publish error: %d\n", ret);
    }
    pthread_mutex_unlock(&mqtt_mutex);
}

static void process_taps(void) {
    pthread_mutex_lock(&tap_mutex);
    if (tap_count == 0) {
        pthread_mutex_unlock(&tap_mutex);
        return;
    }
    const char *action = tap_to_string(tap_count);
    if (action) {
        DEBUG_PRINT("\n    Processing %d tap(s) -> %s\n", tap_count, action);
        mqtt_publish(action);
    } else {
        DEBUG_PRINT("\n    No action for %d taps\n", tap_count);
    }
    tap_count = 0;
    pthread_mutex_unlock(&tap_mutex);
}

static int calibrate(libusb_device_handle *h) {
    unsigned char *pixels = malloc(NUM_PIXELS);
    if (!pixels) return -1;
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
    free(pixels);
    if (!samples) { fprintf(stderr, "No calibration samples!\n"); return -1; }
    baseline_brightness = sum / samples;
    DEBUG_PRINT("Baseline: %.1f\n\n", baseline_brightness);
    return 0;
}

static void mqtt_connect_callback(struct mosquitto *m, void *obj, int result) {
    (void)m; (void)obj;
    pthread_mutex_lock(&mqtt_mutex);
    if (result == 0) {
        mqtt_connected = 1;
        DEBUG_PRINT("MQTT connected successfully\n");
    } else {
        mqtt_connected = 0;
        DEBUG_PRINT("MQTT connection failed: %d\n", result);
    }
    pthread_mutex_unlock(&mqtt_mutex);
}

static void mqtt_disconnect_callback(struct mosquitto *m, void *obj, int rc) {
    (void)m; (void)obj;
    pthread_mutex_lock(&mqtt_mutex);
    mqtt_connected = 0;
    DEBUG_PRINT("MQTT disconnected: %d\n", rc);
    pthread_mutex_unlock(&mqtt_mutex);
}

static void *mqtt_thread(void *arg) {
    (void)arg;
    int reconnect_delay = 1;
    
    while (running) {
        pthread_mutex_lock(&mqtt_mutex);
        if (mosq) {
            mosquitto_destroy(mosq);
            mosq = NULL;
        }
        mqtt_connected = 0;
        pthread_mutex_unlock(&mqtt_mutex);
        
        mosq = mosquitto_new(NULL, true, NULL);
        if (!mosq) {
            DEBUG_PRINT("MQTT: failed to create client, retrying in %ds\n", MQTT_RECONNECT_DELAY);
            sleep(MQTT_RECONNECT_DELAY);
            continue;
        }
        
        mosquitto_connect_callback_set(mosq, mqtt_connect_callback);
        mosquitto_disconnect_callback_set(mosq, mqtt_disconnect_callback);
        
        if (strlen(MQTT_USERNAME) > 0) {
            mosquitto_username_pw_set(mosq, MQTT_USERNAME, MQTT_PASSWORD);
        }
        
        int ret = mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE);
        if (ret != MOSQ_ERR_SUCCESS) {
            DEBUG_PRINT("MQTT: connection failed (%d), retrying in %ds\n", ret, MQTT_RECONNECT_DELAY);
            mosquitto_destroy(mosq);
            mosq = NULL;
            sleep(MQTT_RECONNECT_DELAY);
            continue;
        }
        
        DEBUG_PRINT("MQTT connecting to %s:%d\n", MQTT_HOST, MQTT_PORT);
        reconnect_delay = 1;
        
        while (running) {
            ret = mosquitto_loop(mosq, 1000, 1);
            if (ret == MOSQ_ERR_NO_CONN || ret == MOSQ_ERR_CONN_LOST || ret == MOSQ_ERR_CONN_REFUSED) {
                pthread_mutex_lock(&mqtt_mutex);
                mqtt_connected = 0;
                pthread_mutex_unlock(&mqtt_mutex);
                DEBUG_PRINT("MQTT connection lost, reconnecting in %ds\n", reconnect_delay);
                sleep(reconnect_delay);
                if (reconnect_delay < MQTT_RECONNECT_DELAY) reconnect_delay *= 2;
                if (reconnect_delay > MQTT_RECONNECT_DELAY) reconnect_delay = MQTT_RECONNECT_DELAY;
                break;
            }
            if (ret != MOSQ_ERR_SUCCESS) {
                DEBUG_PRINT("MQTT loop error: %d\n", ret);
            }
        }
    }
    
    pthread_mutex_lock(&mqtt_mutex);
    if (mosq) {
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = NULL;
    }
    mqtt_connected = 0;
    pthread_mutex_unlock(&mqtt_mutex);
    
    return NULL;
}

static void *sensor_thread(void *arg) {
    libusb_device_handle *h = (libusb_device_handle *)arg;
    unsigned char *pixels = malloc(NUM_PIXELS);
    if (!pixels) return NULL;
    DEBUG_PRINT("Monitoring started. Threshold: %.1f, Multi-tap timeout: %.1fs\n\n", FINGER_THRESHOLD, MULTI_TAP_TIMEOUT);
    
    while (running) {
        if (capture_frame(h, pixels) < 0) {
            struct timespec ts = {0, CAPTURE_INTERVAL * 1000}; nanosleep(&ts, NULL);
            continue;
        }
        
        double brightness = smooth(avg_brightness(pixels));
        double delta = fabs(brightness - baseline_brightness);
        double now = get_time();
        
        pthread_mutex_lock(&tap_mutex);
        if (tap_count > 0 && !is_touching && now - tap_times[tap_count - 1] > MULTI_TAP_TIMEOUT) {
            pthread_mutex_unlock(&tap_mutex);
            process_taps();
        } else {
            pthread_mutex_unlock(&tap_mutex);
        }
        
        if (!is_touching && delta > FINGER_THRESHOLD) {
            is_touching = 1;
            pthread_mutex_lock(&tap_mutex);
            DEBUG_PRINT("[TOUCH] Tap #%d | Brightness: %.1f (Δ=%.1f)\n", tap_count + 1, brightness, delta);
            pthread_mutex_unlock(&tap_mutex);
        }
        
        if (is_touching && delta <= FINGER_THRESHOLD) {
            is_touching = 0;
            pthread_mutex_lock(&tap_mutex);
            if (tap_count < MAX_TAP_SEQUENCE) {
                tap_times[tap_count] = now;
                tap_count++;
                DEBUG_PRINT("         Released -> Tap #%d\n", tap_count);
            }
            pthread_mutex_unlock(&tap_mutex);
        }
        
        struct timespec ts = {0, CAPTURE_INTERVAL * 1000}; nanosleep(&ts, NULL);
    }
    free(pixels);
    return NULL;
}

int main(int argc, char *argv[]) {
    (void)argc; (void)argv;
    libusb_device_handle *handle = NULL;
    
    if (load_env(".env") == 0) {
        DEBUG_PRINT("No .env file found, using default settings\n");
    }
    
    brightness_history = malloc(SMOOTHING_FRAMES * sizeof(double));
    if (!brightness_history) {
        fprintf(stderr, "Failed to allocate memory\n");
        return 1;
    }
    
    printf("\nFingerprint Sensor\n");
    printf("Device: %04x:%04x | Protocol: MQTT | Debug: %s\n", 
           VENDOR_ID, PRODUCT_ID, DEBUG ? "ON" : "OFF");
    printf("MQTT: %s:%d | Topic: %s\n\n", MQTT_HOST, MQTT_PORT, MQTT_TOPIC);
    
    if (libusb_init(NULL) < 0) { fprintf(stderr, "libusb init failed\n"); free(brightness_history); return 1; }
    handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!handle) { fprintf(stderr, "Device not found\n"); libusb_exit(NULL); free(brightness_history); return 1; }
    DEBUG_PRINT("Device found\n");
    
    if (libusb_kernel_driver_active(handle, 0)) libusb_detach_kernel_driver(handle, 0);
    if (libusb_claim_interface(handle, 0) < 0) { fprintf(stderr, "Interface claim failed\n"); libusb_close(handle); libusb_exit(NULL); free(brightness_history); return 1; }
    libusb_reset_device(handle);
    
    mosquitto_lib_init();
    
    pthread_t mqtt_tid, sensor_tid;
    pthread_create(&mqtt_tid, NULL, mqtt_thread, NULL);
    
    if (calibrate(handle) < 0) { 
        running = 0;
        pthread_join(mqtt_tid, NULL);
        mosquitto_lib_cleanup();
        libusb_release_interface(handle, 0); 
        libusb_close(handle); 
        libusb_exit(NULL);
        free(brightness_history);
        return 1; 
    }
    
    pthread_create(&sensor_tid, NULL, sensor_thread, handle);
    
    pthread_join(sensor_tid, NULL);
    
    running = 0;
    pthread_join(mqtt_tid, NULL);
    
    mosquitto_lib_cleanup();
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(NULL);
    free(brightness_history);
    return 0;
}
