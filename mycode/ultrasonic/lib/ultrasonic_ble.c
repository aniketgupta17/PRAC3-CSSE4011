/**
 * @file ultrasonic_ble.c
 * @brief BLE communication library for ultrasonic sensor node
 */

#include "ultrasonic_ble.h"
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

/* Stack size and priority for BLE broadcasting thread */
#define BLE_THREAD_STACK_SIZE       4096
#define BLE_THREAD_PRIORITY         6

/* Broadcast interval */
#define ULTRASONIC_BROADCAST_INTERVAL_MS 1000  /* 1 second */

/* Company ID for manufacturer-specific data */
#define COMPANY_ID_LSB             0xFF
#define COMPANY_ID_MSB             0xFF

/* Ultrasonic data packet format */
#define ULTRASONIC_PACKET_TYPE     0x01
#define ULTRASONIC_ADV_DATA_LEN    7

/* Global state */
static atomic_t *g_distance_atomic;          /* pointer to latest distance */
static uint8_t   g_node_id;                  /* node identifier */
static uint8_t   g_x_position;               /* fixed X position on grid */
static atomic_t  g_broadcasting = ATOMIC_INIT(0);

/* BLE thread context */
K_THREAD_STACK_DEFINE(ble_thread_stack, BLE_THREAD_STACK_SIZE);
static struct k_thread  ble_thread_data;
static k_tid_t          ble_thread_id = NULL;

/**
 * @brief Hex-dump a set of BT advertising or scan-response entries
 */
static void print_adv_packet(const struct bt_data *ad, size_t ad_len,
                             const struct bt_data *sd, size_t sd_len)
{
    printk(">>> Advertisement packet dump:\n");
    for (size_t i = 0; i < ad_len; i++) {
        printk("  AD[%zu]: type=0x%02x, len=%u, data=",
               i, ad[i].type, ad[i].data_len);
        const uint8_t *d = ad[i].data;
        for (size_t j = 0; j < ad[i].data_len; j++) {
            printk("%02x ", d[j]);
        }
        printk("\n");
    }
    for (size_t i = 0; i < sd_len; i++) {
        printk("  SD[%zu]: type=0x%02x, len=%u, data=",
               i, sd[i].type, sd[i].data_len);
        const uint8_t *d = sd[i].data;
        for (size_t j = 0; j < sd[i].data_len; j++) {
            printk("%02x ", d[j]);
        }
        printk("\n");
    }
}

/**
 * @brief Thread that packages and broadcasts ultrasonic readings over BLE
 */
static void ble_broadcast_thread(void *p1, void *p2, void *p3)
{
    int     err;
    uint16_t current_distance;

    printk("Ultrasonic BLE broadcasting thread started\n");

    while (atomic_get(&g_broadcasting)) {
        /* Fetch latest distance */
        current_distance = (uint16_t)atomic_get(g_distance_atomic);

        /* Build manufacturer-specific adv packet */
        uint8_t adv_data[ULTRASONIC_ADV_DATA_LEN] = {
    0xBB, 0xBB,                // Your node ID
    g_x_position,                     // X coordinate
    (uint8_t)(current_distance & 0xFF),           // LSB of distance
    (uint8_t)((current_distance >> 8) & 0xFF)     // MSB of distance
};

        struct bt_data ad = BT_DATA(BT_DATA_MANUFACTURER_DATA,
                                    adv_data, ULTRASONIC_ADV_DATA_LEN);

        /* Dump raw bytes to console */
        print_adv_packet(&ad, 1, NULL, 0);

        /* Start non-connectable advertising */
        err = bt_le_adv_start(BT_LE_ADV_NCONN, &ad, 1, NULL, 0);
        if (err) {
            printk("BLE advertising start failed: %d\n", err);
            k_msleep(1000);
            continue;
        }

        /* Advertise for a short burst */
        k_msleep(200);

        /* Stop advertising */
        err = bt_le_adv_stop();
        if (err) {
            printk("BLE advertising stop failed: %d\n", err);
        }

        /* Wait until next cycle */
        k_msleep(ULTRASONIC_BROADCAST_INTERVAL_MS - 200);
    }

    printk("Ultrasonic BLE broadcasting thread exiting\n");
}

/**
 * @brief Initialize Bluetooth and configure node parameters
 */
int ultrasonic_ble_init(uint8_t node_id,
                        uint8_t x_position,
                        atomic_t *distance_atomic)
{
    int err;

    printk("Initializing ultrasonic BLE communication\n");

    g_node_id         = node_id;
    g_x_position      = x_position;
    g_distance_atomic = distance_atomic;

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed: %d\n", err);
        return err;
    }

    printk("Bluetooth initialized\n");
    printk("Ultrasonic node configured: ID=%u, X=%u\n",
           g_node_id, g_x_position);

    return 0;
}

/**
 * @brief Start the BLE broadcasting thread
 */
int ultrasonic_ble_start_broadcasting(void)
{
    if (atomic_get(&g_broadcasting)) {
        printk("BLE broadcasting already active\n");
        return -EALREADY;
    }

    atomic_set(&g_broadcasting, 1);

    ble_thread_id = k_thread_create(&ble_thread_data,
                                    ble_thread_stack,
                                    BLE_THREAD_STACK_SIZE,
                                    ble_broadcast_thread,
                                    NULL, NULL, NULL,
                                    BLE_THREAD_PRIORITY,
                                    0, K_NO_WAIT);
    if (ble_thread_id == NULL) {
        printk("Failed to create BLE broadcasting thread\n");
        atomic_set(&g_broadcasting, 0);
        return -ENOMEM;
    }

    printk("Ultrasonic BLE broadcasting started\n");
    return 0;
}

/**
 * @brief Stop broadcasting and join the BLE thread
 */
int ultrasonic_ble_stop_broadcasting(void)
{
    if (!atomic_get(&g_broadcasting)) {
        printk("BLE broadcasting not active\n");
        return -EALREADY;
    }

    atomic_set(&g_broadcasting, 0);

    if (ble_thread_id != NULL) {
        k_thread_join(&ble_thread_data, K_SECONDS(1));
        ble_thread_id = NULL;
    }

    bt_le_adv_stop();
    printk("Ultrasonic BLE broadcasting stopped\n");

    return 0;
}

/**
 * @brief Update the shared distance value
 */
void ultrasonic_ble_update_distance(uint16_t distance_mm)
{
    if (g_distance_atomic != NULL) {
        atomic_set(g_distance_atomic, distance_mm);
    }
}