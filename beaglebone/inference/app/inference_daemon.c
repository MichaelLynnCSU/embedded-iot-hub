/******************************************************************************
 * \file inference_daemon.c
 * \brief PIR-triggered person detection using TFLite C API.
 *
 * Acts as TCP server on port 9090.
 * ESP32-CAM connects and pushes JPEGs on PIR trigger.
 * Inference runs on each received JPEG.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <stdint.h>
#include <stdarg.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"
#include "tensorflow/lite/c/c_api.h"

/******************************** CONFIG **************************************/
#define MODEL_PATH        "/opt/inference/models/detect.tflite"
#define LABEL_PATH        "/opt/inference/models/labelmap.txt"
#define PENDING_DIR       "/data/pending"
#define LOG_PATH          "/var/log/inference.log"
#define LISTEN_PORT       9090
#define MAX_JPEG_BYTES    500000
#define PERSON_LABEL      "person"
#define CONFIDENCE_THRESH 0.5f
#define MAX_LABELS        100
#define LABEL_LEN         64

/******************************** GLOBALS *************************************/
static volatile int       g_running   = 1;
static FILE              *g_log       = NULL;
static TfLiteModel       *g_model     = NULL;
static TfLiteInterpreter *g_interp    = NULL;
static int                g_server_fd = -1;
static int                g_client_fd = -1;
static char               g_labels[MAX_LABELS][LABEL_LEN];
static int                g_label_count = 0;

/******************************** LOGGING *************************************/
static void log_msg(const char *fmt, ...)
{
    va_list ap;
    time_t  now = time(NULL);
    char    ts[32];
    struct tm tm_buf;

    localtime_r(&now, &tm_buf);
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_buf);

    if (g_log) {
        fprintf(g_log, "[%s] ", ts);
        va_start(ap, fmt);
        vfprintf(g_log, fmt, ap);
        va_end(ap);
        fprintf(g_log, "\n");
        fflush(g_log);
    }

    fprintf(stderr, "[%s] ", ts);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
}

/******************************** SIGNAL **************************************/
static void sig_handler(int sig) { (void)sig; g_running = 0; }

/******************************** LABELS **************************************/
static int load_labels(void)
{
    FILE *f = fopen(LABEL_PATH, "r");
    if (!f) { log_msg("ERROR: cannot open labels: %s", LABEL_PATH); return -1; }
    while (g_label_count < MAX_LABELS &&
           fgets(g_labels[g_label_count], LABEL_LEN, f)) {
        g_labels[g_label_count][strcspn(g_labels[g_label_count], "\r\n")] = 0;
        g_label_count++;
    }
    fclose(f);
    log_msg("Loaded %d labels", g_label_count);
    return 0;
}

static const char *label_for(int idx)
{
    if (idx < 0 || idx >= g_label_count) return "";
    return g_labels[idx];
}

/******************************** TCP SERVER **********************************/
static void tcp_server_init(void)
{
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(LISTEN_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    g_server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(g_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(g_server_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(g_server_fd, 1);
    log_msg("TCP server listening on port %d", LISTEN_PORT);
}

static uint8_t *receive_jpeg(size_t *out_len)
{
    if (g_client_fd < 0) {
        log_msg("Waiting for ESP32-CAM connection...");
        g_client_fd = accept(g_server_fd, NULL, NULL);
        if (g_client_fd < 0) return NULL;
        log_msg("ESP32-CAM connected");

        int keepalive = 1;
        int keepidle  = 30;
        int keepintvl = 5;
        int keepcnt   = 3;
        setsockopt(g_client_fd, SOL_SOCKET,  SO_KEEPALIVE, &keepalive, sizeof(keepalive));
        setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
        setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
        setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));
    }

    uint8_t hdr[4];
    ssize_t n = recv(g_client_fd, hdr, 4, MSG_WAITALL);
    if (n != 4) {
        log_msg("Connection lost, waiting for reconnect");
        close(g_client_fd); g_client_fd = -1; return NULL;
    }

    uint32_t jpeg_len = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) |
                        ((uint32_t)hdr[2] <<  8) |  (uint32_t)hdr[3];

    if (jpeg_len == 0 || jpeg_len > MAX_JPEG_BYTES) {
        log_msg("Bad jpeg_len %u", jpeg_len);
        close(g_client_fd); g_client_fd = -1; return NULL;
    }

    uint8_t *buf = malloc(jpeg_len);
    if (!buf) { log_msg("ERROR: malloc"); return NULL; }

    size_t received = 0;
    while (received < jpeg_len) {
        n = recv(g_client_fd, buf + received, jpeg_len - received, 0);
        if (n <= 0) {
            log_msg("Receive truncated");
            free(buf); close(g_client_fd); g_client_fd = -1; return NULL;
        }
        received += (size_t)n;
    }

    *out_len = jpeg_len;
    log_msg("Received JPEG %zu bytes", jpeg_len);
    return buf;
}

/******************************** INFERENCE ***********************************/
static int run_inference(const uint8_t *jpeg, size_t jpeg_len,
                         int *person_detected, float *confidence)
{
    *person_detected = 0;
    *confidence      = 0.0f;

    TfLiteTensor *input_tensor = TfLiteInterpreterGetInputTensor(g_interp, 0);
    if (!input_tensor) { log_msg("ERROR: no input tensor"); return -1; }

    int input_h = TfLiteTensorDim(input_tensor, 1);
    int input_w = TfLiteTensorDim(input_tensor, 2);

    int img_w = 0, img_h = 0, img_ch = 0;
    uint8_t *img = stbi_load_from_memory(jpeg, (int)jpeg_len,
                                         &img_w, &img_h, &img_ch, 3);
    if (!img) {
        log_msg("ERROR: stbi_load failed: %s", stbi_failure_reason());
        return -1;
    }
    log_msg("Decoded JPEG %dx%d -> model %dx%d", img_w, img_h, input_w, input_h);

    size_t   input_size = (size_t)(input_h * input_w * 3);
    uint8_t *input_buf  = malloc(input_size);
    if (!input_buf) { stbi_image_free(img); return -1; }

    stbir_resize_uint8_linear(img, img_w, img_h, 0,
                              input_buf, input_w, input_h, 0, STBIR_RGB);
    stbi_image_free(img);

    TfLiteStatus status = TfLiteTensorCopyFromBuffer(input_tensor,
                                                     input_buf, input_size);
    free(input_buf);
    if (status != kTfLiteOk) { log_msg("ERROR: CopyFromBuffer"); return -1; }

    if (TfLiteInterpreterInvoke(g_interp) != kTfLiteOk) {
        log_msg("ERROR: Invoke failed"); return -1;
    }

    const TfLiteTensor *scores_tensor  = TfLiteInterpreterGetOutputTensor(g_interp, 2);
    const TfLiteTensor *classes_tensor = TfLiteInterpreterGetOutputTensor(g_interp, 1);
    if (!scores_tensor || !classes_tensor) {
        log_msg("ERROR: missing output tensors"); return -1;
    }

    int          num_det = TfLiteTensorDim(scores_tensor, 1);
    const float *scores  = (const float *)TfLiteTensorData(scores_tensor);
    const float *classes = (const float *)TfLiteTensorData(classes_tensor);

    for (int i = 0; i < num_det; i++) {
        if (scores[i] < CONFIDENCE_THRESH) break;
        int         class_idx = (int)classes[i] + 1;
        const char *label     = label_for(class_idx);
        log_msg("Detection %d: label=%s score=%.2f", i, label, scores[i]);
        if (strcmp(label, PERSON_LABEL) == 0) {
            *person_detected = 1;
            *confidence      = scores[i];
            break;
        }
    }
    return 0;
}

/******************************** SAVE JPEG ***********************************/
static void save_jpeg(const uint8_t *jpeg, size_t len,
                      int detected, float confidence)
{
    char path[256];
    time_t now = time(NULL);
    struct tm tm_buf;
    char ts[32];

    localtime_r(&now, &tm_buf);
    strftime(ts, sizeof(ts), "%Y%m%dT%H%M%SZ", &tm_buf);
    snprintf(path, sizeof(path), "%s/%s_%s_%03d.jpg",
             PENDING_DIR, ts,
             detected ? "person" : "noperson",
             (int)(confidence * 100));

    FILE *f = fopen(path, "wb");
    if (!f) { log_msg("ERROR: save JPEG: %s", strerror(errno)); return; }
    fwrite(jpeg, 1, len, f);
    fclose(f);
    log_msg("Saved %s", path);
}

/******************************** MAIN ****************************************/
int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    g_log = fopen(LOG_PATH, "a");
    log_msg("inference_daemon starting");

    mkdir(PENDING_DIR, 0755);

    if (load_labels() < 0) return 1;

    g_model = TfLiteModelCreateFromFile(MODEL_PATH);
    if (!g_model) { log_msg("ERROR: cannot load model"); return 1; }

    TfLiteInterpreterOptions *opts = TfLiteInterpreterOptionsCreate();
    TfLiteInterpreterOptionsSetNumThreads(opts, 1);
    g_interp = TfLiteInterpreterCreate(g_model, opts);
    TfLiteInterpreterOptionsDelete(opts);

    if (!g_interp) { log_msg("ERROR: cannot create interpreter"); return 1; }
    if (TfLiteInterpreterAllocateTensors(g_interp) != kTfLiteOk) {
        log_msg("ERROR: AllocateTensors failed"); return 1;
    }
    log_msg("Model loaded: %s", MODEL_PATH);

    tcp_server_init();

    while (g_running) {
        size_t   jpeg_len = 0;
        uint8_t *jpeg     = receive_jpeg(&jpeg_len);
        if (!jpeg) { usleep(100000); continue; }

        int   detected   = 0;
        float confidence = 0.0f;
        run_inference(jpeg, jpeg_len, &detected, &confidence);
        log_msg("Result: person=%d confidence=%.2f", detected, confidence);
        save_jpeg(jpeg, jpeg_len, detected, confidence);
        free(jpeg);
    }

    log_msg("inference_daemon shutting down");
    if (g_client_fd >= 0) close(g_client_fd);
    if (g_server_fd >= 0) close(g_server_fd);
    TfLiteInterpreterDelete(g_interp);
    TfLiteModelDelete(g_model);
    if (g_log) fclose(g_log);
    return 0;
}
