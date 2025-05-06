// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/hci.h>
// #include <zephyr/sys/util.h>
// #include <string.h>

// #define COMPANY_ID_LSB 0xFF
// #define COMPANY_ID_MSB 0xFF
// #define BT_ADDR_LE_STR_LEN 18

// struct beacon_info {
//     const char *name;       /* e.g. "4011-A" */
//     const char *addr_str;   /* MAC string */
//     uint16_t    major, minor;
//     uint8_t     x, y;
// };

// static const struct beacon_info beacons[] = {
//     {"4011-A", "F5:75:FE:85:34:67", 2753, 32998, 0, 0},
//     {"4011-B", "E5:73:87:06:1E:86", 32975, 20959, 2, 0},
//     {"4011-C", "CA:99:9E:FD:98:B1", 26679, 40363, 4, 0},
//     {"4011-D", "CB:1B:89:82:FF:FE", 41747, 38800, 4, 2},
//     {"4011-E", "D4:D2:A0:A4:5C:AC", 30679, 51963, 4, 4},
//     {"4011-F", "C1:13:27:E9:B7:7C", 6195,  18394, 2, 4},
//     {"4011-G", "F1:04:48:06:39:A0", 30525, 30544, 0, 4},
//     {"4011-H", "CA:0C:E0:DB:CE:60", 57395, 28931, 0, 2},
// };

// /* Lookup beacon by address+major+minor */
// static const struct beacon_info *
// find_beacon(const char *addr_str, uint16_t major, uint16_t minor)
// {
//     char cleaned[BT_ADDR_LE_STR_LEN];
//     strncpy(cleaned, addr_str, sizeof(cleaned));
//     cleaned[sizeof(cleaned)-1] = '\0';
//     // if (char *p = strchr(cleaned, ' ')) { *p = '\0'; }

//     for (size_t i = 0; i < ARRAY_SIZE(beacons); i++) {
//         if (strcasecmp(cleaned, beacons[i].addr_str) == 0
//             && beacons[i].major == major
//             && beacons[i].minor == minor) {
//             return &beacons[i];
//         }
//     }
//     return NULL;
// }

// /* Broadcast RSSI + ID (4 bytes total) */
// static void send_broadcast(int8_t rssi, char sender_id)
// {
//     uint8_t mfg_data[4] = {
//         COMPANY_ID_LSB,
//         COMPANY_ID_MSB,
//         (uint8_t)rssi,
//         (uint8_t)sender_id
//     };

//     printk(">> Advertise MfgData: %02X %02X %02X '%c' "
//            "(RSSI=%d, ID=%c)\n",
//            mfg_data[0], mfg_data[1],
//            mfg_data[2], sender_id,
//            rssi, sender_id);

//     const struct bt_data ad[] = {
//         BT_DATA(BT_DATA_MANUFACTURER_DATA,
//                 mfg_data, sizeof(mfg_data)),
//     };

//     int err = bt_le_adv_start(BT_LE_ADV_NCONN,
//                               ad, ARRAY_SIZE(ad),
//                               NULL, 0);
//     if (err) {
//         printk("!! adv_start err %d\n", err);
//         return;
//     }

//     k_msleep(1000);

//     err = bt_le_adv_stop();
//     if (err) {
//         printk("!! adv_stop err %d\n", err);
//     } else {
//         printk(">> Advertising stopped\n");
//     }
// }

// /* Called for every packet while scanning */
// static void scan_recv(const struct bt_le_scan_recv_info *info,
//                       struct net_buf_simple *buf)
// {
//     printk(">> scan_recv(): len=%u, RSSI=%d\n", buf->len, info->rssi);

//     /* Dump first bytes for debug */
//     if (buf->len >= 16) {
//         printk("   data[0..15]= ");
//         for (int i = 0; i < 16; i++) {
//             printk("%02X ", buf->data[i]);
//         }
//         printk("\n");
//     }

//     /* iBeacon prefix at bytes 5–8: 4C 00 02 15 */
//     if (buf->len >= 30 &&
//         buf->data[5] == 0x4C &&
//         buf->data[6] == 0x00 &&
//         buf->data[7] == 0x02 &&
//         buf->data[8] == 0x15) {

//         char le_addr[BT_ADDR_LE_STR_LEN];
//         bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

//         uint16_t major = (buf->data[25] << 8) | buf->data[26];
//         uint16_t minor = (buf->data[27] << 8) | buf->data[28];

//         const struct beacon_info *b =
//             find_beacon(le_addr, major, minor);
//         if (b) {
//             char id = b->name[strlen(b->name)-1];
//             printk(">> iBeacon match: %s ID='%c'\n",
//                    b->name, id);

//             send_broadcast(info->rssi, id);
//         }
//     }
// }

// static struct bt_le_scan_cb scan_callbacks = {
//     .recv = scan_recv,
// };

// void main(void)
// {
//     int err;

//     printk("Starting iBeacon Scanner & Broadcaster\n");

//     err = bt_enable(NULL);
//     if (err) {
//         printk("!! bt_enable err %d\n", err);
//         return;
//     }
//     printk("Bluetooth initialized\n");

//     bt_le_scan_cb_register(&scan_callbacks);

//     struct bt_le_scan_param scan_param = {
//         .type     = BT_LE_SCAN_TYPE_PASSIVE,
//         .options  = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
//         .interval = BT_GAP_SCAN_FAST_INTERVAL,
//         .window   = BT_GAP_SCAN_FAST_WINDOW,
//     };
//     err = bt_le_scan_start(&scan_param, NULL);
//     if (err) {
//         printk("!! bt_le_scan_start err %d\n", err);
//         return;
//     }
//     printk("Scanning started\n");

//     while (1) {
//         k_sleep(K_SECONDS(2));
//     }
// }



#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <string.h>

#define COMPANY_ID_LSB 0xFF
#define COMPANY_ID_MSB 0xFF
#define BT_ADDR_LE_STR_LEN 18

struct beacon_info {
    const char *name;       /* e.g. "4011-A" */
    const char *addr_str;   /* MAC string */
    uint16_t    major, minor;
    uint8_t     x, y;
};

static const struct beacon_info beacons[] = {
    {"4011-A", "F5:75:FE:85:34:67", 2753, 32998, 0, 0},
    {"4011-B", "E5:73:87:06:1E:86", 32975, 20959, 2, 0},
    {"4011-C", "CA:99:9E:FD:98:B1", 26679, 40363, 4, 0},
    {"4011-D", "CB:1B:89:82:FF:FE", 41747, 38800, 4, 2},
    {"4011-E", "D4:D2:A0:A4:5C:AC", 30679, 51963, 4, 4},
    {"4011-F", "C1:13:27:E9:B7:7C", 6195,  18394, 2, 4},
    {"4011-G", "F1:04:48:06:39:A0", 30525, 30544, 0, 4},
    {"4011-H", "CA:0C:E0:DB:CE:60", 57395, 28931, 0, 2},
};

/* Lookup beacon by address+major+minor */
static const struct beacon_info *
find_beacon(const char *addr_str, uint16_t major, uint16_t minor)
{
    char cleaned[BT_ADDR_LE_STR_LEN];
    strncpy(cleaned, addr_str, sizeof(cleaned));
    cleaned[sizeof(cleaned)-1] = '\0';

    for (size_t i = 0; i < ARRAY_SIZE(beacons); i++) {
        if (strcasecmp(cleaned, beacons[i].addr_str) == 0
            && beacons[i].major == major
            && beacons[i].minor == minor) {
            return &beacons[i];
        }
    }
    return NULL;
}

/* Broadcast RSSI + ID (4 bytes total) */
static void send_broadcast_full(const struct beacon_info *b, int8_t rssi)
{
    uint8_t mfg_data[21];

    // Header
    mfg_data[0] = 0xFF;
    mfg_data[1] = 0xFF;

    // RSSI (cast to uint8_t)
    mfg_data[2] = (uint8_t)rssi;

    // Name: "4011-B" (max 6 bytes)
    memcpy(&mfg_data[3], b->name, 6);

    // MAC Address (convert string to bytes)
    bt_addr_t mac;
    if (bt_addr_from_str(b->addr_str, &mac)) {
        printk("!! Invalid MAC format: %s\n", b->addr_str);
        return;
    }
    memcpy(&mfg_data[9], mac.val, 6);

    // Major
    mfg_data[15] = (b->major >> 8) & 0xFF;
    mfg_data[16] = b->major & 0xFF;

    // Minor
    mfg_data[17] = (b->minor >> 8) & 0xFF;
    mfg_data[18] = b->minor & 0xFF;

    // X and Y
    mfg_data[19] = b->x;
    mfg_data[20] = b->y;

    const struct bt_data ad[] = {
        BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
    };

    int err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("!! adv start failed: %d\n", err);
        return;
    }

    printk(">> Advertise MfgData: ");
    for (int i = 0; i < sizeof(mfg_data); i++) {
        printk("%02X ", mfg_data[i]);
    }
    printk("(RSSI=%d, ID=%s)\n", rssi, b->name);
    
    k_msleep(1000);
    bt_le_adv_stop();
}

/* Called for every packet while scanning */
static void scan_recv(const struct bt_le_scan_recv_info *info,
                      struct net_buf_simple *buf)
{
    printk(">> scan_recv(): len=%u, RSSI=%d\n", buf->len, info->rssi);

    /* Dump first bytes for debug */
    if (buf->len >= 16) {
        printk("   data[0..15]= ");
        for (int i = 0; i < 16; i++) {
            printk("%02X ", buf->data[i]);
        }
        printk("\n");
    }

    /* iBeacon prefix at bytes 5–8: 4C 00 02 15 */
    if (buf->len >= 30 &&
        buf->data[5] == 0x4C &&
        buf->data[6] == 0x00 &&
        buf->data[7] == 0x02 &&
        buf->data[8] == 0x15) {

        char le_addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

        uint16_t major = (buf->data[25] << 8) | buf->data[26];
        uint16_t minor = (buf->data[27] << 8) | buf->data[28];

        const struct beacon_info *b =
            find_beacon(le_addr, major, minor);
        if (b) {
            char id = b->name[strlen(b->name)-1];
            printk(">> iBeacon match: %s ID='%c'\n",
                   b->name, id);

            send_broadcast_full(b, info->rssi);
        }
    }
}

static struct bt_le_scan_cb scan_callbacks = {
    .recv = scan_recv,
};

void main(void)
{
    int err;

    printk("Starting iBeacon Scanner & Broadcaster\n");

    err = bt_enable(NULL);
    if (err) {
        printk("!! bt_enable err %d\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    bt_le_scan_cb_register(&scan_callbacks);

    struct bt_le_scan_param scan_param = {
        .type     = BT_LE_SCAN_TYPE_PASSIVE,
        .options  = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_WINDOW,
    };
    err = bt_le_scan_start(&scan_param, NULL);
    if (err) {
        printk("!! bt_le_scan_start err %d\n", err);
        return;
    }
    printk("Scanning started\n");

    while (1) {
        k_sleep(K_SECONDS(2));
    }
}

