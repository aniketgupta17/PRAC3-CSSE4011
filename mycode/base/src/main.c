
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#define COMPANY_ID_LSB 0xFF
#define COMPANY_ID_MSB 0xFF
#define NUM_BEACONS 8

// Beacon coordinates
static const double mock_coordinates[NUM_BEACONS][2] = {
    {0.0, 0.0}, {1.5, 0.0}, {3.0, 0.0}, {3.0, 2.0},
    {3.0, 4.0}, {1.5, 4.0}, {0.0, 4.0}, {0.0, 2.0}
};

#define TX_POWER -59.0
#define PATH_LOSS_N 2.0

static int8_t rssi_values[NUM_BEACONS] = {0};
static bool rssi_updated[NUM_BEACONS] = {false};

static double rssi_to_distance(int8_t rssi) {
    return pow(10.0, (TX_POWER - rssi) / (10.0 * PATH_LOSS_N));
}

static double det2x2(double a11, double a12, double a21, double a22) {
    return a11 * a22 - a12 * a21;
}

static bool invert2x2(double m[2][2], double inv[2][2]) {
    double det = det2x2(m[0][0], m[0][1], m[1][0], m[1][1]);
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
    int valid_count = 0;
    int valid_indices[NUM_BEACONS];

    for (int i = 0; i < NUM_BEACONS; i++) {
        if (rssi_updated[i]) {
            distances[valid_count] = rssi_to_distance(rssi_values[i]);
            valid_indices[valid_count] = i;
            valid_count++;
        }
    }

    if (valid_count < 3) return false;

    int ref_idx = valid_indices[0];
    double xr = mock_coordinates[ref_idx][0];
    double yr = mock_coordinates[ref_idx][1];
    double dr = distances[0];

    double A[NUM_BEACONS - 1][2];
    double b[NUM_BEACONS - 1];
    int used_count = valid_count - 1;

    for (int i = 1; i <= used_count; i++) {
        int idx = valid_indices[i];
        double xi = mock_coordinates[idx][0];
        double yi = mock_coordinates[idx][1];
        double di = distances[i];

        A[i - 1][0] = 2.0 * (xi - xr);
        A[i - 1][1] = 2.0 * (yi - yr);
        b[i - 1] = (xi * xi - xr * xr) + (yi * yi - yr * yr) + (dr * dr - di * di);
    }

    double ATA[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
    double ATb[2] = {0.0, 0.0};
    for (int i = 0; i < used_count; i++) {
        ATA[0][0] += A[i][0] * A[i][0];
        ATA[0][1] += A[i][0] * A[i][1];
        ATA[1][0] += A[i][1] * A[i][0];
        ATA[1][1] += A[i][1] * A[i][1];

        ATb[0] += A[i][0] * b[i];
        ATb[1] += A[i][1] * b[i];
    }

    double ATA_inv[2][2];
    if (!invert2x2(ATA, ATA_inv)) return false;

    *x = ATA_inv[0][0] * ATb[0] + ATA_inv[0][1] * ATb[1];
    *y = ATA_inv[1][0] * ATb[0] + ATA_inv[1][1] * ATb[1];

    *x += xr;
    *y += yr;
    return true;
}

static uint8_t processed_ids = 0;

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
                int8_t sender_rssi = (int8_t)m[2];
                char id_char = (char)m[3];

                if (id_char >= 'A' && id_char <= 'H') {
                    int index = id_char - 'A';
                    if ((processed_ids & (1 << index)) == 0) {
                        rssi_values[index] = sender_rssi;
                        rssi_updated[index] = true;
                        processed_ids |= (1 << index); // Mark processed
                        printk("✓ Beacon %c RSSI=%d\n", id_char, sender_rssi);
                    }
                }
            }
        }
        i += field_len + 1;
    }
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

void main(void) {
    printk("Bluetooth RSSI-based localization started\n");

    if (bt_enable(NULL)) {
        printk("Bluetooth init failed!\n");
        return;
    }

    bt_le_scan_cb_register(&scan_callbacks);

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
        processed_ids = 0; // Reset processed beacon tracking

				double x, y;
				if (multilateration(&x, &y)) {
						char pos_buf[64];
						snprintk(pos_buf, sizeof(pos_buf), "📍 Position: x=%d.%02d y=%d.%02d\n",
										 (int)x, (int)(fabs(x * 100)) % 100,
										 (int)y, (int)(fabs(y * 100)) % 100);
						printk("%s", pos_buf);
				}

        for (int i = 0; i < NUM_BEACONS; i++) {
            rssi_updated[i] = false;
        }
    }
}
