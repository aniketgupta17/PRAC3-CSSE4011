/**
 * @file main.c
 * @brief Main application for ultrasonic sensor node with BLE communication
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/gpio.h>
#include "ultrasonic_sensor.h"
#include "ultrasonic_ble.h"

/* Using device tree aliases for trigger and echo pins */
#define TRIGGER_NODE DT_ALIAS(trigger)
#define ECHO_NODE    DT_ALIAS(echo)

/* Check if device tree nodes are enabled */
#if !DT_NODE_HAS_STATUS(TRIGGER_NODE, okay)
#error "Trigger pin not available in device tree"
#endif

#if !DT_NODE_HAS_STATUS(ECHO_NODE, okay)
#error "Echo pin not available in device tree"
#endif

/* Extract GPIO info from device tree */
#define TRIGGER_GPIO_CTLR DT_GPIO_CTLR(TRIGGER_NODE, gpios)
#define TRIGGER_PIN       DT_GPIO_PIN(TRIGGER_NODE, gpios)
#define ECHO_GPIO_CTLR    DT_GPIO_CTLR(ECHO_NODE, gpios)
#define ECHO_PIN          DT_GPIO_PIN(ECHO_NODE, gpios)

/* Ultrasonic node configuration - Change Node ID based on board */
#if defined(CONFIG_BOARD_DISCO_L475_IOT1)
  #define ULTRASONIC_NODE_ID 1  /* ID for STM32L475 Discovery node */
  #define ULTRASONIC_X_POS   0  /* Fixed X position on grid (leftmost position) */
#elif defined(CONFIG_BOARD_NRF52840DK_NRF52840)
  #define ULTRASONIC_NODE_ID 2  /* ID for nRF52840 DK node */
  #define ULTRASONIC_X_POS   0  /* Fixed X position on grid (position 1) */
#else
  #define ULTRASONIC_NODE_ID 3  /* Default ID for other boards */
  #define ULTRASONIC_X_POS   2  /* Default X position */
#endif

/* Shared state */
static atomic_t last_distance_mm = ATOMIC_INIT(0);
static atomic_t system_ready = ATOMIC_INIT(0);

int main(void)
{
    int rc;
    
    printk("=================================================\n");
    printk("Ultrasonic node starting up (Node ID: %d)\n", ULTRASONIC_NODE_ID);
    printk("Application version: 1.2.1\n");
    printk("=================================================\n");

    /* Get GPIO devices for trigger and echo pins */
    const struct device *trigger_dev = DEVICE_DT_GET(TRIGGER_GPIO_CTLR);
    const struct device *echo_dev = DEVICE_DT_GET(ECHO_GPIO_CTLR);

    printk("Trigger GPIO: %s pin %d\n", trigger_dev->name, TRIGGER_PIN);
    printk("Echo GPIO: %s pin %d\n", echo_dev->name, ECHO_PIN);

    /* Add detailed device ready checks */
    if (!device_is_ready(trigger_dev)) {
        printk("ERROR: Trigger device not ready\n");
        return -ENODEV;
    } else {
        printk("Trigger device ready\n");
    }

    if (!device_is_ready(echo_dev)) {
        printk("ERROR: Echo device not ready\n");
        return -ENODEV;
    } else {
        printk("Echo device ready\n");
    }
    
    /* Wait a moment for GPIO subsystem to fully initialize */
    k_msleep(100);

    /* Start the ultrasonic sensor thread */
    rc = ultrasonic_thread_start(trigger_dev, TRIGGER_PIN, 
                                echo_dev, ECHO_PIN,
                                &system_ready, &last_distance_mm);
    
    if (rc != 0) {
        printk("ERROR: Failed to start ultrasonic thread: %d\n", rc);
        return rc;
    }

    /* Wait for sensor system to be marked as ready with timeout */
    printk("Waiting for ultrasonic sensor to initialize...\n");
    int timeout_count = 0;
    while (atomic_get(&system_ready) == 0 && timeout_count < 50) {
        k_msleep(100);
        timeout_count++;
    }
    
    if (atomic_get(&system_ready) == 0) {
        printk("WARNING: Ultrasonic sensor initialization timed out, continuing anyway\n");
        atomic_set(&system_ready, 1);  /* Force ready state */
    } else {
        printk("Ultrasonic sensor initialized!\n");
    }

    /* Initialize BLE with retry */
    int retry = 0;
    while (retry < 3) {
        rc = ultrasonic_ble_init(ULTRASONIC_NODE_ID, ULTRASONIC_X_POS, &last_distance_mm);
        if (rc == 0) {
            break;
        }
        printk("ERROR: Failed to initialize BLE: %d (attempt %d/3)\n", rc, retry+1);
        k_msleep(500 * (retry + 1));
        retry++;
    }
    
    if (rc != 0) {
        printk("CRITICAL ERROR: BLE initialization failed after retries\n");
        return rc;
    }

    /* Start BLE broadcasting with retry */
    retry = 0;
    while (retry < 3) {
        rc = ultrasonic_ble_start_broadcasting();
        if (rc == 0) {
            break;
        }
        printk("ERROR: Failed to start BLE broadcasting: %d (attempt %d/3)\n", rc, retry+1);
        k_msleep(500 * (retry + 1));
        retry++;
    }
    
    if (rc != 0) {
        printk("CRITICAL ERROR: BLE broadcasting failed to start\n");
        return rc;
    }

    printk("Ultrasonic node fully operational!\n");

    /* Main loop for additional processing if needed */
    while (1) {
        /* Get the current distance measurement */
        uint16_t distance = get_last_distance();
        
        /* Print current distance every 5 seconds */
        printk("Current distance: %u.%03u m (%u mm)\n", 
               distance / 1000, distance % 1000, distance);
        
        k_sleep(K_SECONDS(5));
    }

    return 0;
}