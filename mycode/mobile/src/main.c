#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <SEGGER_RTT.h>
#include <stdio.h> // for fflush

#define NAME_LEN 30

struct beacon_info {
    const char *name;
    const char *addr_str;
    uint16_t major;
    uint16_t minor;
    uint8_t x;
    uint8_t y;
};

static const struct beacon_info beacons[] = {
    {"4011-A", "F5:75:FE:85:34:67", 275, 329, 0, 0},
    {"4011-B", "E5:73:87:06:1E:86", 32975, 20959, 2, 0},
    {"4011-C", "CA:99:9E:FD:98:B1", 26679, 40363, 4, 0},
    {"4011-D", "CB:1B:89:82:FF:FE", 41747, 38800, 4, 2},
    {"4011-E", "D4:D2:A0:A4:5C:AC", 30679, 51963, 4, 4},
    {"4011-F", "C1:13:27:E9:B7:7C", 6195, 18394, 2, 4},
    {"4011-G", "F1:04:48:06:39:A0", 30525, 30544, 0, 4},
    {"4011-H", "CA:0C:E0:DB:CE:60", 57395, 28931, 0, 2},
    {"Test iPhone Beacon", "5A:72:2D:86:D5:AA", 275, 329, 88, 88}
};

static const struct beacon_info *find_beacon(const char *addr_str, uint16_t major, uint16_t minor)
{
    char cleaned_addr[BT_ADDR_LE_STR_LEN];
    strncpy(cleaned_addr, addr_str, sizeof(cleaned_addr));
    cleaned_addr[sizeof(cleaned_addr) - 1] = '\0';

    char *paren = strchr(cleaned_addr, ' ');
    if (paren) {
        *paren = '\0'; 
    }
    printk("Cleaned Addr: %s\n", cleaned_addr);

    for (int i = 0; i < ARRAY_SIZE(beacons); i++) {
        if (strcasecmp(cleaned_addr, beacons[i].addr_str) == 0 &&
            beacons[i].major == major &&
            beacons[i].minor == minor) {
            return &beacons[i];
            
        }
    }
    return NULL;
}



static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf)
{
    printk("scan_recv() called, buf->len = %d\n", buf->len);
    fflush(stdout);

    if (buf->len < 30) {
        return;
    }

    const uint8_t *data = buf->data;

    // Print raw first bytes
    printk("AD header: %02X %02X %02X %02X %02X %02X\n", data[0], data[1], data[2], data[3], data[4], data[5]);

    if (data[5] == 0x4C && data[6] == 0x00 &&
        data[7] == 0x02 && data[8] == 0x15) {

        char le_addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

        uint16_t major = (data[25] << 8) | data[26];
        uint16_t minor = (data[27] << 8) | data[28];
        int8_t tx_power = (int8_t)data[29];

        printk("iBeacon detected - Addr: %s, Major: %u, Minor: %u, RSSI: %d, TxPower: %d\n",
               le_addr, major, minor, info->rssi, tx_power);
        const struct beacon_info *beacon = find_beacon(le_addr, major, minor);

        if (beacon) {
            printk("Match: %s at [%d,%d] (RSSI %d)\n",
                   beacon->name, beacon->x, beacon->y, info->rssi);
        } else {
            printk("Unknown beacon: %s (Major %u / Minor %u)\n", le_addr, major, minor);
        }
    } else {
        printk("Not an iBeacon packet\n");
    }

    fflush(stdout);
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

void main(void)
{
    int err;

    printk("Starting iBeacon Scanner\n");
    fflush(stdout);

    err = bt_enable(NULL);
    if (err) {
        printk("bt_enable() failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_le_scan_cb_register(&scan_callbacks);

    struct bt_le_scan_param scan_param = {
        .type       = BT_LE_SCAN_TYPE_PASSIVE,
        .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval   = BT_GAP_SCAN_FAST_INTERVAL,
        .window     = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_le_scan_start(&scan_param, NULL);
    if (err) {
        printk("Scan start failed (err %d)\n", err);
        return;
    }

    printk("Scanning started\n");

    int counter = 0;
    while (1) {
        fflush(stdout);
        k_sleep(K_MSEC(2000));
    }
}
