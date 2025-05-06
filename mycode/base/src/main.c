#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <math.h>
#include "ibeacon_node_manager.h"


#define POS_PACKET_TYPE       0x02        
#define COMPANY_ID_LSB 0xFF
#define COMPANY_ID_MSB 0xFF
#define NUM_BEACONS 8
#define ULTRASONIC_ADV_DATA_LEN 7
#define ULTRASONIC_PACKET_TYPE 0x01

// Coordinates of beacons
static const double grid_a[NUM_BEACONS][2] = {
    {0.0, 0.0}, {1.5, 0.0}, {3.0, 0.0}, {3.0, 2.0},
    {3.0, 4.0}, {1.5, 4.0}, {0.0, 4.0}, {0.0, 2.0}
};

// // // Per-beacon TX power
// static const float tx_power_dbm[NUM_BEACONS] = {
//     -59, /* A */
//     -65, /* B */
//     -61, /* C */
//     -66, /* D */
//     -66, /* E */
//     -65, /* F */
//     -67, /* G */
//     -47  /* H */
// };

static uint8_t mfg[7];

static const struct bt_data ad[] = {
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg, sizeof(mfg)),
};

// Per-beacon TX power
static const float tx_power_dbm[NUM_BEACONS] = {
    -59, /* A */
    -59, /* B */
    -59, /* B */
    -59, /* B */
    -59, /* B */
    -59, /* B */
    -59, /* B */
    -59, /* B */
};

#define PATH_LOSS_N 2.0

static int8_t rssi_values[NUM_BEACONS] = {0};
static bool rssi_updated[NUM_BEACONS] = {false};
static double smoothed_rssi[NUM_BEACONS] = {0}; // For RSSI smoothing
static uint8_t processed_ids = 0;

// Threads
#define RSSI_THREAD_STACK_SIZE 2048
#define FUSION_THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(rssi_thread_stack, RSSI_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(fusion_thread_stack, FUSION_THREAD_STACK_SIZE);
static struct k_thread rssi_thread_data;
static struct k_thread fusion_thread_data;

// Message queue for position data
#define QUEUE_SIZE 10

struct fused_position_packet {
    double x;
    double y;
    int64_t timestamp_ms;
    uint8_t source; // 0 = RSSI, 1 = ultrasonic
};

struct position_data {
    double x;
    double y;
    uint8_t source; // 0 for RSSI, 1 for ultrasonic
};

K_MSGQ_DEFINE(position_queue, sizeof(struct position_data), QUEUE_SIZE, 4);

// CLAMP macro
#define CLAMP(val, min, max) (MAX(MIN(val, max), min))

static void format_double(double value, char *buf, size_t buf_size) {
    int integer_part = (int)value;
    int fractional_part = (int)(fabs(value * 100.0)) % 100;
    snprintk(buf, buf_size, "%d.%02d", integer_part, fractional_part);
}

// Kalman filter parameters
#define MEAS_NOISE_RSSI 0.5    // Measurement noise for RSSI (meters)
#define MEAS_NOISE_ULTRA 0.05  // Measurement noise for ultrasonic (meters)
#define PROCESS_NOISE 0.1      // Process noise (meters)

struct kalman_state {
    double x[2];        // State: [x, y]
    double cov[2][2];   // Covariance matrix
};

// Initialize Kalman filter
static struct kalman_state kalman = {
    .x = {0.0, 0.0},   // Start at (0, 0)
    .cov = {
        {1.0, 0.0},
        {0.0, 1.0}
    }
};

static double rssi_to_distance(int8_t rssi, int index) {
    double distance = pow(10.0, (tx_power_dbm[index] - rssi) / (10.0 * PATH_LOSS_N));
    return CLAMP(distance, 0.1, 10.0); // Limit to 0.1–10 meters
}

static void update_rssi(int index, int8_t rssi) {
    const double alpha = 0.3; // Smoothing factor
    if (!rssi_updated[index]) {
        smoothed_rssi[index] = rssi;
    } else {
        smoothed_rssi[index] = alpha * rssi + (1 - alpha) * smoothed_rssi[index];
    }
    rssi_values[index] = (int8_t)smoothed_rssi[index];
    rssi_updated[index] = true;
}

static bool invert2x2(double m[2][2], double inv[2][2]) {
    double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    if (fabs(det) < 1e-6) {
        char det_buf[16];
        format_double(det, det_buf, sizeof(det_buf));
        printk("Matrix inversion failed: determinant too small (%s)\n", det_buf);
        return false;
    }
    double inv_det = 1.0 / det;
    inv[0][0] = m[1][1] * inv_det;
    inv[0][1] = -m[0][1] * inv_det;
    inv[1][0] = -m[1][0] * inv_det;
    inv[1][1] = m[0][0] * inv_det;
    return true;
}

static bool multilateration(double *x, double *y) {
    double distances[NUM_BEACONS];
    int valid_indices[NUM_BEACONS];
    int valid_count = 0;

    // Collect valid RSSI measurements
    for (int i = 0; i < NUM_BEACONS; i++) {
        if (rssi_updated[i]) {
            distances[valid_count] = rssi_to_distance(rssi_values[i], i);
            valid_indices[valid_count++] = i;
        }
    }
    // Check for more than 3 beacons
    if (valid_count <= 2) {
        printk("Multilateration failed: %d beacons detected, more than 2 required\n", valid_count);
        return false;
    }
    // Log beacon data
    char dist_buf[16], x_buf[16], y_buf[16];
    for (int i = 0; i < valid_count; i++) {
        int idx = valid_indices[i];
        format_double(distances[i], dist_buf, sizeof(dist_buf));
        format_double(grid_a[idx][0], x_buf, sizeof(x_buf));
        format_double(grid_a[idx][1], y_buf, sizeof(y_buf));
        printk("Beacon %c: RSSI=%d, Distance=%s m, Coord=(%s, %s)\n",
               'A' + idx, rssi_values[idx], dist_buf, x_buf, y_buf);
    }
    // Select reference beacon (strongest RSSI)
    int ref_idx = valid_indices[0];
    int8_t max_rssi = rssi_values[ref_idx];
    for (int i = 1; i < valid_count; i++) {
        int idx = valid_indices[i];
        if (rssi_values[idx] > max_rssi) {
            max_rssi = rssi_values[idx];
            ref_idx = idx;
        }
    }
    double xr = grid_a[ref_idx][0];
    double yr = grid_a[ref_idx][1];
    double dr = 0.0;
    for (int i = 0; i < valid_count; i++) {
        if (valid_indices[i] == ref_idx) {
            dr = distances[i];
            break;
        }
    }

    // Build linear system
    double A[NUM_BEACONS][2], b[NUM_BEACONS];
    int used = valid_count - 1; // Equations = valid_count - 1

    for (int i = 0, j = 0; i < valid_count; i++) {
        if (valid_indices[i] == ref_idx) continue;
        int idx = valid_indices[i];
        double xi = grid_a[idx][0];
        double yi = grid_a[idx][1];
        double di = distances[i];
        A[j][0] = 2.0 * (xi - xr);
        A[j][1] = 2.0 * (yi - yr);
        b[j] = (xi * xi - xr * xr) + (yi * yi - yr * yr) + (dr * dr - di * di);
        j++;
    }
    // Compute normal equations
    double ATA[2][2] = {{0}}, ATb[2] = {0};
    for (int i = 0; i < used; i++) {
        ATA[0][0] += A[i][0] * A[i][0];
        ATA[0][1] += A[i][0] * A[i][1];
        ATA[1][0] += A[i][1] * A[i][0];
        ATA[1][1] += A[i][1] * A[i][1];
        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }
    // Add small regularization for numerical stability
    ATA[0][0] += 1e-4;
    ATA[1][1] += 1e-4;

    // Invert ATA
    double ATA_inv[2][2];
    if (!invert2x2(ATA, ATA_inv)) {
        printk("Multilateration failed: cannot invert ATA\n");
        return false;
    }

    // Solve for position
    *x = ATA_inv[0][0] * ATb[0] + ATA_inv[0][1] * ATb[1] + xr;
    *y = ATA_inv[1][0] * ATb[0] + ATA_inv[1][1] * ATb[1] + yr;

    // Log result
    format_double(*x, x_buf, sizeof(x_buf));
    format_double(*y, y_buf, sizeof(y_buf));
    printk("Multilateration (%d beacons): x=%s, y=%s\n", valid_count, x_buf, y_buf);

    // Clamp to grid
    *x = CLAMP(*x, 0.0, 3.0);
    *y = CLAMP(*y, 0.0, 4.0);
    return true;
}

K_MUTEX_DEFINE(adv_mutex); // Mutex for protecting mfg array

// 2D Kalman filter functions
static void kalman_predict(struct kalman_state *state) {
    // State transition matrix A (identity, assuming static position)
    double A[2][2] = {
        {1.0, 0.0},
        {0.0, 1.0}
    };

    // Process noise covariance Q
    double Q[2][2] = {
        {PROCESS_NOISE, 0.0},
        {0.0, PROCESS_NOISE}
    };

    // Predict state: x = A * x
    double x_temp[2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            x_temp[i] += A[i][j] * state->x[j];
        }
    }
    memcpy(state->x, x_temp, sizeof(x_temp));

    // Predict covariance: cov = A * cov * A^T + Q
    double temp[2][2] = {0}, cov_temp[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                temp[i][j] += A[i][k] * state->cov[k][j];
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                cov_temp[i][j] += temp[i][k] * A[j][k];
            }
            cov_temp[i][j] += Q[i][j];
        }
    }
    memcpy(state->cov, cov_temp, sizeof(cov_temp));
}

static void kalman_update(struct kalman_state *state, double obs[2], uint8_t source) {
    // Observation matrix H (identity, observing x, y directly)
    double H[2][2] = {
        {1.0, 0.0},
        {0.0, 1.0}
    };

    // Measurement noise covariance R
    double R[2][2] = {
        {source == 0 ? MEAS_NOISE_RSSI : MEAS_NOISE_ULTRA, 0.0},
        {0.0, source == 0 ? MEAS_NOISE_RSSI : MEAS_NOISE_ULTRA}
    };

    // Measurement residual: error_x = obs - H * x
    double error_x[2] = {0};
    for (int i = 0; i < 2; i++) {
        error_x[i] = obs[i];
        for (int j = 0; j < 2; j++) {
            error_x[i] -= H[i][j] * state->x[j];
        }
    }

    // Residual covariance: error_cov = H * cov * H^T + R
    double temp[2][2] = {0}, error_cov[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                temp[i][j] += H[i][k] * state->cov[k][j];
            }
        }
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                error_cov[i][j] += temp[i][k] * H[j][k]; // H^T = H
            }
            error_cov[i][j] += R[i][j];
        }
    }

    // Kalman gain: K = cov * H^T * error_cov^-1
    double K[2][2] = {0}, error_cov_inv[2][2];
    if (!invert2x2(error_cov, error_cov_inv)) {
        printk("Kalman update failed: cannot invert error covariance\n");
        return;
    }
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                K[i][j] += state->cov[i][k] * H[j][k] * error_cov_inv[k][j];
            }
        }
    }

    // Update state: x = x + K * error_x
    double x_temp[2] = {0};
    for (int i = 0; i < 2; i++) {
        x_temp[i] = state->x[i];
        for (int j = 0; j < 2; j++) {
            x_temp[i] += K[i][j] * error_x[j];
        }
    }
    memcpy(state->x, x_temp, sizeof(x_temp));

    // Update covariance: cov = (I - K * H) * cov
    double KH[2][2] = {0}, I_KH[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        }
    }
    double cov_temp[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                cov_temp[i][j] += I_KH[i][k] * state->cov[k][j];
            }
        }
    }
    memcpy(state->cov, cov_temp, sizeof(cov_temp));
}

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf) {
    if (!info || !buf || buf->len == 0) return;

    uint8_t *data = buf->data;
    size_t len = buf->len;
    size_t i = 0;

    while (i + 1 < len) {
        uint8_t field_len = data[i];
        if (field_len == 0 || i + field_len >= len) break;

        uint8_t field_type = data[i + 1];
        if (field_type == BT_DATA_MANUFACTURER_DATA) {
            uint8_t *m = &data[i + 2];
            size_t mlen = field_len - 1;

            if (mlen >= 4 && m[0] == COMPANY_ID_LSB && m[1] == COMPANY_ID_MSB) {
                // printk("[DEBUG] Matched iBeacon packet header (Company ID)\n");
                     

                int8_t rssi = (int8_t)m[2];
                char id = (char)m[8];

                if (is_ibeacon_node(id, NULL)) {
            
                if (id >= 'A' && id <= 'H') {
                    int index = id - 'A';
            
                    if (!(processed_ids & (1 << index))) {
                        update_rssi(index, rssi);
                        processed_ids |= (1 << index);
            
                        char rssi_buf[16];
                        format_double(smoothed_rssi[index], rssi_buf, sizeof(rssi_buf));
                        printk("Detection: Beacon %c RSSI=%s\n", id, rssi_buf);
                    } else {
                    }
                } else {
                    printk("[WARN] ID %c out of expected range A–H. Ignored.\n", id);
                }
            }}
              if (mlen >= ULTRASONIC_ADV_DATA_LEN && m[0] == 0xAA && m[1] == 0xAA && m[2] == ULTRASONIC_PACKET_TYPE) {
                uint8_t node_id = m[3];
                uint8_t x_ultra = m[4];
                uint16_t y_ultra = ((uint16_t)m[6] << 8) | m[5];
                printk("Ultrasonic Node %d: x=%d, y=%d mm\n", node_id, x_ultra, y_ultra);

                // Enqueue ultrasonic position (convert mm to meters)
                struct position_data pos = {
                    .x = 0,
                    .y = y_ultra / 1000.0,
                    .source = 1 // Ultrasonic
                };

                if (y_ultra == 4000) {
                    printk("Ultrasonic Node %d: Invalid reading (y=4000), skipped\n", node_id);
                    continue;             }

                k_msgq_put(&position_queue, &pos, K_NO_WAIT);
            }
             if (mlen >= ULTRASONIC_ADV_DATA_LEN && m[0] == 0xBB && m[1] == 0xBB && m[2] == ULTRASONIC_PACKET_TYPE )  {
                uint8_t node_id = m[3];
                uint8_t x_ultra = m[4];
                uint16_t y_ultra = ((uint16_t)m[6] << 8) | m[5];
                printk("Ultrasonic Node two %d: x=%d, y=%d mm\n", node_id, x_ultra, y_ultra);

                // Enqueue ultrasonic position (convert mm to meters)
                struct position_data pos = {
                    .x = 0,
                    .y = y_ultra / 1000.0,
                    .source = 1 // Ultrasonic
                };

                if (y_ultra == 4000) {
                    printk("Ultrasonic Node %d: Invalid reading (y=4000), skipped\n", node_id);
                    continue; // Exit current handler or block
                }

                k_msgq_put(&position_queue, &pos, K_NO_WAIT);
            }
        }
        i += field_len + 1;
    }
}

static struct bt_le_scan_cb unified_scan_callbacks = {
    .recv = scan_recv,
};

static void rssi_thread(void *p1, void *p2, void *p3) {
    printk("RSSI thread started\n");

    struct bt_le_scan_param scan_param = {
        .type = BT_LE_SCAN_TYPE_PASSIVE,
        .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = 0x0020, // 20ms
        .window = 0x0010,   // 10ms
    };

    if (bt_le_scan_start(&scan_param, NULL)) {
        printk("Scan start failed!\n");
        return;
    }

    while (1) {
        k_sleep(K_SECONDS(3)); 
        processed_ids = 0;

        double x_rssi, y_rssi;
        if (multilateration(&x_rssi, &y_rssi)) {
            char pos_buf[64];
            snprintk(pos_buf, sizeof(pos_buf), "RSSI Position: x=%d.%02d y=%d.%02d\n",
                     (int)x_rssi, (int)(fabs(x_rssi * 100)) % 100,
                     (int)y_rssi, (int)(fabs(y_rssi * 100)) % 100);
            printk("%s", pos_buf);

            // Enqueue RSSI position
            struct position_data pos = {
                .x = x_rssi,
                .y = y_rssi,
                .source = 0 // RSSI
            };
            k_msgq_put(&position_queue, &pos, K_NO_WAIT);
        }

        memset(rssi_updated, 0, sizeof(rssi_updated));
    }
}

void send_position_adv(double x_m, double y_m) {
    // Convert meters to centimeters and clamp to grid size (300 cm × 400 cm)
    uint16_t x_cm = CLAMP((uint16_t)lrint(x_m * 100.0), 0, 300);
    uint16_t y_cm = CLAMP((uint16_t)lrint(y_m * 100.0), 0, 400);
    uint32_t timestamp_ms = (uint32_t)k_uptime_get();  // fits in 4 bytes

    // Update mfg array with new position data
    uint8_t temp_mfg[11] = {
        COMPANY_ID_LSB,
        COMPANY_ID_MSB,
        POS_PACKET_TYPE,
        (uint8_t)(x_cm & 0xFF),
        (uint8_t)(x_cm >> 8),
        (uint8_t)(y_cm & 0xFF),
        (uint8_t)(y_cm >> 8),
        (uint8_t)(timestamp_ms & 0xFF),
        (uint8_t)((timestamp_ms >> 8) & 0xFF),
        (uint8_t)((timestamp_ms >> 16) & 0xFF),
        (uint8_t)((timestamp_ms >> 24) & 0xFF)
    };
    printk("MFG data: ");
    for (int i = 0; i < 11; i++) {
        printk("%02X ", temp_mfg[i]);
    }
    printk("\n");
    // Copy to global mfg array atomically to avoid race conditions
    k_mutex_lock(&adv_mutex, K_FOREVER);
    memcpy(mfg, temp_mfg, sizeof(mfg));
    k_mutex_unlock(&adv_mutex);

    // Update advertisement data
    int err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Failed to update advertisement data (err %d)\n", err);
    } else {
        printk("Updated advertisement: x=%u cm, y=%u cm\n", x_cm, y_cm);
    }
}

void start_advertising(void) {
    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = BT_LE_ADV_OPT_NONE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2, // 100 ms
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2, // 150 ms
        .peer = NULL,
    };

    int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising start failed (err %d)\n", err);
    } else {
        printk("Started advertising\n");
    }
}


static void fusion_thread(void *p1, void *p2, void *p3) {
    printk("Fusion thread started\n");

    char fused_buf[64], x_buf[16], y_buf[16];
    while (1) {
        struct position_data pos;
        if (k_msgq_get(&position_queue, &pos, K_MSEC(500)) == 0) {
            double obs[2] = {pos.x, pos.y};
            kalman_predict(&kalman);
            kalman_update(&kalman, obs, pos.source);

            format_double(kalman.x[0], x_buf, sizeof(x_buf));
            format_double(kalman.x[1], y_buf, sizeof(y_buf));
            snprintk(fused_buf, sizeof(fused_buf), "** Fused Position: x=%s y=%s (%s) **\n",
                     x_buf, y_buf, pos.source == 0 ? "RSSI" : "Ultrasonic");
            printk("%s", fused_buf);

            // Update advertisement with fused position
            send_position_adv(kalman.x[0], kalman.x[1]);
        } else {
            kalman_predict(&kalman);
        }
    }
}


void main(void) {
    printk("Bluetooth RSSI and Ultrasonic localization with 2D Kalman filter started\n");

    if (bt_enable(NULL)) {
        printk("Bluetooth init failed!\n");
        return;
    }

    start_advertising();
    ibeacon_node_manager_init();


    bt_le_scan_cb_register(&unified_scan_callbacks);

    
    k_thread_create(&rssi_thread_data, rssi_thread_stack,
                    K_THREAD_STACK_SIZEOF(rssi_thread_stack),
                    rssi_thread, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&fusion_thread_data, fusion_thread_stack,
                    K_THREAD_STACK_SIZEOF(fusion_thread_stack),
                    fusion_thread, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);
}
