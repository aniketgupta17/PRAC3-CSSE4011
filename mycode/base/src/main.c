

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <math.h>

#define COMPANY_ID_LSB 0xFF
#define COMPANY_ID_MSB 0xFF
#define NUM_BEACONS 8
#define ULTRASONIC_ADV_DATA_LEN 7
#define ULTRASONIC_PACKET_TYPE 0x01

// Coordinates of beacons
static const double mock_coordinates[NUM_BEACONS][2] = {
    {0.0, 0.0}, {1.5, 0.0}, {3.0, 0.0}, {3.0, 2.0},
    {3.0, 4.0}, {1.5, 4.0}, {0.0, 4.0}, {0.0, 2.0}
};

#define TX_POWER -59.0
#define PATH_LOSS_N 2.0

static int8_t rssi_values[NUM_BEACONS] = {0};
static bool rssi_updated[NUM_BEACONS] = {false};
static uint8_t processed_ids = 0;

// Threads
#define THREAD_STACK_SIZE 2048
#define THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(rssi_thread_stack, THREAD_STACK_SIZE);
static struct k_thread rssi_thread_data;

static double rssi_to_distance(int8_t rssi) {
    return pow(10.0, (TX_POWER - rssi) / (10.0 * PATH_LOSS_N));
}

static bool invert2x2(double m[2][2], double inv[2][2]) {
    double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    if (fabs(det) < 1e-6) return false;
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

    for (int i = 0; i < NUM_BEACONS; i++) {
        if (rssi_updated[i]) {
            distances[valid_count] = rssi_to_distance(rssi_values[i]);
            valid_indices[valid_count++] = i;
        }
    }

    if (valid_count < 3) return false;

    int ref_idx = valid_indices[0];
    double xr = mock_coordinates[ref_idx][0];
    double yr = mock_coordinates[ref_idx][1];
    double dr = distances[0];

    double A[NUM_BEACONS][2], b[NUM_BEACONS];
    int used = valid_count - 1;

    for (int i = 1; i <= used; i++) {
        int idx = valid_indices[i];
        double xi = mock_coordinates[idx][0];
        double yi = mock_coordinates[idx][1];
        double di = distances[i];

        A[i - 1][0] = 2.0 * (xi - xr);
        A[i - 1][1] = 2.0 * (yi - yr);
        b[i - 1] = (xi * xi - xr * xr) + (yi * yi - yr * yr) + (dr * dr - di * di);
    }

    double ATA[2][2] = {{0}}, ATb[2] = {0};
    for (int i = 0; i < used; i++) {
        ATA[0][0] += A[i][0] * A[i][0];
        ATA[0][1] += A[i][0] * A[i][1];
        ATA[1][0] += A[i][1] * A[i][0];
        ATA[1][1] += A[i][1] * A[i][1];
        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }

    double ATA_inv[2][2];
    if (!invert2x2(ATA, ATA_inv)) return false;

    *x = ATA_inv[0][0] * ATb[0] + ATA_inv[0][1] * ATb[1] + xr;
    *y = ATA_inv[1][0] * ATb[0] + ATA_inv[1][1] * ATb[1] + yr;

    return true;
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
                int8_t rssi = (int8_t)m[2];
                char id = (char)m[3];

                if (id >= 'A' && id <= 'H') {
                    int index = id - 'A';
                    if (!(processed_ids & (1 << index))) {
                        rssi_values[index] = rssi;
                        rssi_updated[index] = true;
                        processed_ids |= (1 << index);
                        printk("✓ Beacon %c RSSI=%d\n", id, rssi);
                    }
                }
            } else if (mlen >= ULTRASONIC_ADV_DATA_LEN && m[0] == 0xAA && m[1] == 0xAA && m[2] == ULTRASONIC_PACKET_TYPE) {
                uint8_t node_id = m[3];
                uint8_t x_position = m[4];
                uint16_t dist = ((uint16_t)m[6] << 8) | m[5];
                printk("Ultrasonic Node %d: x=%d, y=%d mm\n", node_id, x_position, dist);
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
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
    };

    if (bt_le_scan_start(&scan_param, NULL)) {
        printk("Scan start failed!\n");
        return;
    }

    while (1) {
        k_sleep(K_SECONDS(2));
        processed_ids = 0;

        double x, y;
				if (multilateration(&x, &y)) {
						char pos_buf[64];
						snprintk(pos_buf, sizeof(pos_buf), "Position: x=%d.%02d y=%d.%02d\n",
										 (int)x, (int)(fabs(x * 100)) % 100,
										 (int)y, (int)(fabs(y * 100)) % 100);
						printk("%s", pos_buf);
				}

        memset(rssi_updated, 0, sizeof(rssi_updated));
    }
}

void main(void) {
    printk("Bluetooth RSSI and Ultrasonic localization started\n");

    if (bt_enable(NULL)) {
        printk("Bluetooth init failed!\n");
        return;
    }

    bt_le_scan_cb_register(&unified_scan_callbacks);

    k_thread_create(&rssi_thread_data, rssi_thread_stack,
                    K_THREAD_STACK_SIZEOF(rssi_thread_stack),
                    rssi_thread, NULL, NULL, NULL,
                    THREAD_PRIORITY, 0, K_NO_WAIT);
}

