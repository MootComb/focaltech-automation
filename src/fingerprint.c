#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
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

#define MAX_QUEUE_SIZE 100
#define MAX_MESSAGE_LENGTH 256

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
#define MAX_CMD_PAYLOAD 60

static const unsigned char CMD_STATUS[]  = {0x37, 0x01, 0x01, 0x01};
static const unsigned char CMD_PREPARE[] = {0x80, 0x02, 0x01};
static const unsigned char CMD_CAPTURE[] = {0x82, 0x73, 0x01};
static const unsigned char CMD_IMAGE[]   = {0x81};

typedef struct {
    char messages[MAX_QUEUE_SIZE][MAX_MESSAGE_LENGTH];
    atomic_int head;
    atomic_int tail;
    atomic_int count;
} MessageQueue;

static MessageQueue mqtt_queue;
static atomic_int running = 1;
static atomic_int is_touching = 0;
static double baseline_brightness = 0.0;
static double *brightness_history = NULL;
static atomic_int history_index = 0;
static atomic_int history_filled = 0;

#define DEBUG_PRINT(fmt, ...) do { if (DEBUG) printf(fmt, ##__VA_ARGS__); } while(0)

static char* trim(char *str) {
    while (*str == ' ' || *str == '\t') str++;
    char *end = str + strlen(str) - 1;
    while (end > str && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
        *end-- = '\0';
    }
    return str;
}

static int parse_int_safe(const char *str, int *result) {
    char *endptr;
    errno = 0;
    long val = strtol(str, &endptr, 10);
    if (errno != 0 || endptr == str || *endptr != '\0' || val < INT32_MIN || val > INT32_MAX) {
        return -1;
    }
    *result = (int)val;
    return 0;
}

static int parse_double_safe(const char *str, double *result) {
    char *endptr;
    errno = 0;
    double val = strtod(str, &endptr);
    if (errno != 0 || endptr == str || *endptr != '\0') {
        return -1;
    }
    *result = val;
    return 0;
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
        
        size_t value_len = strlen(value);
        if (value_len >= 2 && (value[0] == '"' || value[0] == '\'')) {
            if (value[value_len - 1] == value[0]) {
                value[value_len - 1] = '\0';
                value++;
            }
        }
        
        int int_val;
        double double_val;
        
        if (strcmp(key, "CAPTURE_INTERVAL") == 0 && parse_int_safe(value, &int_val) == 0) {
            if (int_val > 0) { CAPTURE_INTERVAL = int_val; loaded++; }
        }
        else if (strcmp(key, "BASELINE_SAMPLES") == 0 && parse_int_safe(value, &int_val) == 0) {
            if (int_val > 0) { BASELINE_SAMPLES = int_val; loaded++; }
        }
        else if (strcmp(key, "FINGER_THRESHOLD") == 0 && parse_double_safe(value, &double_val) == 0) {
            if (double_val > 0) { FINGER_THRESHOLD = double_val; loaded++; }
        }
        else if (strcmp(key, "MULTI_TAP_TIMEOUT") == 0 && parse_double_safe(value, &double_val) == 0) {
            if (double_val > 0) { MULTI_TAP_TIMEOUT = double_val; loaded++; }
        }
        else if (strcmp(key, "SMOOTHING_FRAMES") == 0 && parse_int_safe(value, &int_val) == 0) {
            if (int_val > 0) { SMOOTHING_FRAMES = int_val; loaded++; }
        }
        else if (strcmp(key, "MQTT_HOST") == 0 && strlen(value) > 0) {
            strncpy(MQTT_HOST, value, sizeof(MQTT_HOST) - 1);
            MQTT_HOST[sizeof(MQTT_HOST) - 1] = '\0';
            loaded++;
        }
        else if (strcmp(key, "MQTT_PORT") == 0 && parse_int_safe(value, &int_val) == 0) {
            if (int_val > 0 && int_val <= 65535) { MQTT_PORT = int_val; loaded++; }
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
        else if (strcmp(key, "MQTT_TOPIC") == 0 && strlen(value) > 0) {
            strncpy(MQTT_TOPIC, value, sizeof(MQTT_TOPIC) - 1);
            MQTT_TOPIC[sizeof(MQTT_TOPIC) - 1] = '\0';
            loaded++;
        }
        else if (strcmp(key, "MQTT_KEEPALIVE") == 0 && parse_int_safe(value, &int_val) == 0) {
            if (int_val > 0) { MQTT_KEEPALIVE = int_val; loaded++; }
        }
        else if (strcmp(key, "MQTT_RECONNECT_DELAY") == 0 && parse_int_safe(value, &int_val) == 0) {
            if (int_val > 0) { MQTT_RECONNECT_DELAY = int_val; loaded++; }
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
    if (len > MAX_CMD_PAYLOAD) {
        DEBUG_PRINT("Command payload too large: %d bytes\n", len);
        return -1;
    }
    
    unsigned char buf[64];
    int transferred;
    buf[0] = 0x02;
    buf[1] = 0x00;
    buf[2] = (unsigned char)len;
    memcpy(buf + 3, payload, len);
    buf[3 + len] = checksum(payload, len);
    
    int ret = libusb_bulk_transfer(h, OUT_ENDPOINT, buf, 3 + len + 1, &transferred, 1000);
    if (ret < 0) {
        DEBUG_PRINT("Send command failed: %s\n", libusb_error_name(ret));
        return -1;
    }
    return 0;
}

static int read_resp(libusb_device_handle *h, unsigned char *data, int len) {
    int transferred;
    int ret = libusb_bulk_transfer(h, IN_ENDPOINT, data, len, &transferred, 500);
    if (ret < 0) {
        DEBUG_PRINT("Read response failed: %s\n", libusb_error_name(ret));
        return -1;
    }
    return transferred;
}

static int capture_frame(libusb_device_handle *h, unsigned char *pixels) {
    unsigned char resp[IMAGE_DATA_LENGTH];
    int bytes;
    
    if (send_cmd(h, CMD_STATUS, sizeof(CMD_STATUS)) < 0) return -1;
    if (read_resp(h, resp, RESPONSE_ACK_LENGTH) < 0) return -1;
    
    if (send_cmd(h, CMD_PREPARE, sizeof(CMD_PREPARE)) < 0) return -1;
    if (read_resp(h, resp, RESPONSE_ACK_LENGTH) < 0) return -1;
    
    if (send_cmd(h, CMD_CAPTURE, sizeof(CMD_CAPTURE)) < 0) return -1;
    if (read_resp(h, resp, RESPONSE_ACK_LENGTH) < 0) return -1;
    
    if (send_cmd(h, CMD_IMAGE, sizeof(CMD_IMAGE)) < 0) return -1;
    bytes = read_resp(h, resp, IMAGE_DATA_LENGTH);
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
    int idx = atomic_load(&history_index);
    brightness_history[idx] = current;
    idx = (idx + 1) % SMOOTHING_FRAMES;
    atomic_store(&history_index, idx);
    if (!atomic_load(&history_filled) && idx == 0) atomic_store(&history_filled, 1);
    
    int filled = atomic_load(&history_filled);
    int count = filled ? SMOOTHING_FRAMES : idx;
    double sum = 0.0;
    for (int i = 0; i < count; i++) {
        int read_idx = (idx - 1 - i + SMOOTHING_FRAMES) % SMOOTHING_FRAMES;
        sum += brightness_history[read_idx];
    }
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

static int queue_push(MessageQueue *q, const char *message) {
    if (!message) return -1;
    
    int tail = atomic_load(&q->tail);
    int count = atomic_load(&q->count);
    
    if (count >= MAX_QUEUE_SIZE) {
        DEBUG_PRINT("MQTT Queue full, dropping message: %s\n", message);
        return -1;
    }
    
    strncpy(q->messages[tail], message, MAX_MESSAGE_LENGTH - 1);
    q->messages[tail][MAX_MESSAGE_LENGTH - 1] = '\0';
    
    int new_tail = (tail + 1) % MAX_QUEUE_SIZE;
    atomic_store(&q->tail, new_tail);
    atomic_fetch_add(&q->count, 1);
    
    return 0;
}

static int queue_pop(MessageQueue *q, char *message, size_t max_len) {
    if (!message || max_len == 0) return -1;
    
    int head = atomic_load(&q->head);
    int count = atomic_load(&q->count);
    
    if (count == 0) return 0;
    
    strncpy(message, q->messages[head], max_len - 1);
    message[max_len - 1] = '\0';
    
    int new_head = (head + 1) % MAX_QUEUE_SIZE;
    atomic_store(&q->head, new_head);
    atomic_fetch_sub(&q->count, 1);
    
    return 1;
}

static int calibrate(libusb_device_handle *h) {
    unsigned char *pixels = malloc(NUM_PIXELS);
    if (!pixels) {
        fprintf(stderr, "Failed to allocate pixel buffer\n");
        return -1;
    }
    
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
        struct timespec ts = {0, 100000000};
        nanosleep(&ts, NULL);
    }
    
    free(pixels);
    
    if (!samples) {
        fprintf(stderr, "No calibration samples!\n");
        return -1;
    }
    
    baseline_brightness = sum / samples;
    DEBUG_PRINT("Baseline: %.1f\n\n", baseline_brightness);
    return 0;
}

static void *mqtt_thread(void *arg) {
    (void)arg;
    struct mosquitto *mosq = NULL;
    int mqtt_connected = 0;
    int reconnect_delay = 1;
    
    while (atomic_load(&running)) {
        if (!mqtt_connected) {
            if (mosq) {
                mosquitto_destroy(mosq);
                mosq = NULL;
            }
            
            mosquitto_lib_init();
            mosq = mosquitto_new(NULL, true, NULL);
            if (!mosq) {
                DEBUG_PRINT("MQTT: failed to create client, retrying in %ds\n", MQTT_RECONNECT_DELAY);
                sleep(MQTT_RECONNECT_DELAY);
                continue;
            }
            
            if (strlen(MQTT_USERNAME) > 0) {
                mosquitto_username_pw_set(mosq, MQTT_USERNAME, MQTT_PASSWORD);
            }
            
            DEBUG_PRINT("MQTT connecting to %s:%d\n", MQTT_HOST, MQTT_PORT);
            int ret = mosquitto_connect(mosq, MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE);
            
            if (ret == MOSQ_ERR_SUCCESS) {
                mqtt_connected = 1;
                reconnect_delay = 1;
                DEBUG_PRINT("MQTT connected successfully\n");
            } else {
                DEBUG_PRINT("MQTT connection failed: %s, retrying in %ds\n",
                           mosquitto_strerror(ret), MQTT_RECONNECT_DELAY);
                mosquitto_destroy(mosq);
                mosq = NULL;
                sleep(MQTT_RECONNECT_DELAY);
                continue;
            }
        }
        
        while (mqtt_connected && atomic_load(&running)) {
            char message[MAX_MESSAGE_LENGTH];
            int has_message = queue_pop(&mqtt_queue, message, sizeof(message));
            
            if (has_message == 1) {
                int ret = mosquitto_publish(mosq, NULL, MQTT_TOPIC,
                                           (int)strlen(message), message, 0, false);
                if (ret == MOSQ_ERR_SUCCESS) {
                    DEBUG_PRINT("MQTT sent: %s -> %s\n", MQTT_TOPIC, message);
                } else if (ret == MOSQ_ERR_NO_CONN || ret == MOSQ_ERR_CONN_LOST) {
                    DEBUG_PRINT("MQTT publish failed: connection lost\n");
                    mqtt_connected = 0;
                    queue_push(&mqtt_queue, message);
                    break;
                } else {
                    DEBUG_PRINT("MQTT publish error: %s\n", mosquitto_strerror(ret));
                }
            }
            
            int ret = mosquitto_loop(mosq, 0, 1);
            
            if (ret == MOSQ_ERR_NO_CONN || ret == MOSQ_ERR_CONN_LOST) {
                DEBUG_PRINT("MQTT connection lost\n");
                mqtt_connected = 0;
            } else if (ret != MOSQ_ERR_SUCCESS) {
                DEBUG_PRINT("MQTT loop error: %s\n", mosquitto_strerror(ret));
            }
            
            struct timespec ts = {0, 100000000};
            nanosleep(&ts, NULL);
        }
        
        if (!mqtt_connected && atomic_load(&running)) {
            DEBUG_PRINT("MQTT reconnecting in %ds\n", reconnect_delay);
            sleep(reconnect_delay);
            reconnect_delay = (reconnect_delay * 2 > MQTT_RECONNECT_DELAY) ?
                            MQTT_RECONNECT_DELAY : reconnect_delay * 2;
        }
    }
    
    if (mosq) {
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
    }
    mosquitto_lib_cleanup();
    
    return NULL;
}

static void *sensor_thread(void *arg) {
    libusb_device_handle *h = (libusb_device_handle *)arg;
    unsigned char *pixels = malloc(NUM_PIXELS);
    if (!pixels) {
        fprintf(stderr, "Failed to allocate pixel buffer\n");
        return NULL;
    }
    
    DEBUG_PRINT("Sensor thread started. Threshold: %.1f, Multi-tap timeout: %.1fs\n\n",
                FINGER_THRESHOLD, MULTI_TAP_TIMEOUT);
    DEBUG_PRINT("Monitoring started...\n");
    
    int local_tap_count = 0;
    double local_tap_times[MAX_TAP_SEQUENCE] = {0};
    
    while (atomic_load(&running)) {
        if (capture_frame(h, pixels) < 0) {
            struct timespec ts = {0, CAPTURE_INTERVAL * 1000};
            nanosleep(&ts, NULL);
            continue;
        }
        
        double brightness = smooth(avg_brightness(pixels));
        double delta = fabs(brightness - baseline_brightness);
        double now = get_time();
        
        if (local_tap_count > 0 && !atomic_load(&is_touching) &&
            now - local_tap_times[local_tap_count - 1] > MULTI_TAP_TIMEOUT) {
            const char *action = tap_to_string(local_tap_count);
            if (action) {
                DEBUG_PRINT("\nProcessing %d tap(s) -> %s\n", local_tap_count, action);
                queue_push(&mqtt_queue, action);
            }
            local_tap_count = 0;
        }
        
        if (!atomic_load(&is_touching) && delta > FINGER_THRESHOLD) {
            atomic_store(&is_touching, 1);
            DEBUG_PRINT("[TOUCH] Tap #%d | Brightness: %.1f (delta=%.1f)\n",
                       local_tap_count + 1, brightness, delta);
        }
        
        if (atomic_load(&is_touching) && delta <= FINGER_THRESHOLD) {
            atomic_store(&is_touching, 0);
            if (local_tap_count < MAX_TAP_SEQUENCE) {
                local_tap_times[local_tap_count] = now;
                local_tap_count++;
                DEBUG_PRINT("Released -> Tap #%d\n", local_tap_count);
            }
        }
        
        struct timespec ts = {0, CAPTURE_INTERVAL * 1000};
        nanosleep(&ts, NULL);
    }
    
    free(pixels);
    DEBUG_PRINT("Sensor thread stopped\n");
    return NULL;
}

static void cleanup_resources(libusb_device_handle *handle) {
    if (handle) {
        libusb_release_interface(handle, 0);
        libusb_close(handle);
    }
    libusb_exit(NULL);
    free(brightness_history);
}

static void signal_handler(int sig) {
    (void)sig;
    atomic_store(&running, 0);
}

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    
    libusb_device_handle *handle = NULL;
    pthread_t mqtt_tid, sensor_tid;
    
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    
    if (load_env(".env") == 0) {
        DEBUG_PRINT("No .env file found, using default settings\n");
    }
    
    brightness_history = calloc(SMOOTHING_FRAMES, sizeof(double));
    if (!brightness_history) {
        fprintf(stderr, "Failed to allocate memory\n");
        return 1;
    }
    
    atomic_init(&mqtt_queue.head, 0);
    atomic_init(&mqtt_queue.tail, 0);
    atomic_init(&mqtt_queue.count, 0);
    
    printf("\nFingerprint Sensor\n");
    printf("Device: %04x:%04x | Protocol: MQTT | Debug: %s\n",
           VENDOR_ID, PRODUCT_ID, DEBUG ? "ON" : "OFF");
    printf("MQTT: %s:%d | Topic: %s\n\n", MQTT_HOST, MQTT_PORT, MQTT_TOPIC);
    
    if (libusb_init(NULL) < 0) {
        fprintf(stderr, "libusb init failed\n");
        cleanup_resources(NULL);
        return 1;
    }
    
    handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!handle) {
        fprintf(stderr, "Device not found\n");
        cleanup_resources(NULL);
        return 1;
    }
    DEBUG_PRINT("Device found\n");
    
    if (libusb_kernel_driver_active(handle, 0)) {
        if (libusb_detach_kernel_driver(handle, 0) < 0) {
            fprintf(stderr, "Failed to detach kernel driver\n");
            cleanup_resources(handle);
            return 1;
        }
    }
    
    if (libusb_claim_interface(handle, 0) < 0) {
        fprintf(stderr, "Interface claim failed\n");
        cleanup_resources(handle);
        return 1;
    }
    
    if (libusb_reset_device(handle) < 0) {
        DEBUG_PRINT("Warning: Device reset failed\n");
    }
    
    if (calibrate(handle) < 0) {
        fprintf(stderr, "Calibration failed\n");
        cleanup_resources(handle);
        return 1;
    }
    
    if (pthread_create(&mqtt_tid, NULL, mqtt_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create MQTT thread\n");
        cleanup_resources(handle);
        return 1;
    }
    
    if (pthread_create(&sensor_tid, NULL, sensor_thread, handle) != 0) {
        fprintf(stderr, "Failed to create sensor thread\n");
        atomic_store(&running, 0);
        pthread_join(mqtt_tid, NULL);
        cleanup_resources(handle);
        return 1;
    }
    
    pthread_join(sensor_tid, NULL);
    
    atomic_store(&running, 0);
    pthread_join(mqtt_tid, NULL);
    
    cleanup_resources(handle);
    
    printf("Program terminated\n");
    return 0;
}
