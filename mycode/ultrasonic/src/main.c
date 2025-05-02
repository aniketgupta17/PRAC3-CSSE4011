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

/* Ultrasonic node configuration */
#define ULTRASONIC_NODE_ID 1     /* ID for this ultrasonic node */
#define ULTRASONIC_X_POS   0     /* Fixed X position on grid (leftmost position) */

/* Shared state */
static atomic_t last_distance_mm = ATOMIC_INIT(0);
static atomic_t system_ready = ATOMIC_INIT(0);

int main(void)
{
    printk("=================================================\n");
    printk("Ultrasonic node starting up on STM32L475 Discovery\n");
    printk("Application version: 1.2.0\n");
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

    /* Start the ultrasonic sensor thread */
    int rc = ultrasonic_thread_start(trigger_dev, TRIGGER_PIN, 
                                    echo_dev, ECHO_PIN,
                                    &system_ready, &last_distance_mm);
    
    if (rc != 0) {
        printk("ERROR: Failed to start ultrasonic thread: %d\n", rc);
        return rc;
    }

    /* Wait for sensor system to be marked as ready */
    printk("Waiting for ultrasonic sensor to initialize...\n");
    while (atomic_get(&system_ready) == 0) {
        k_msleep(100);
    }
    printk("Ultrasonic sensor initialized!\n");

    /* Initialize BLE for ultrasonic node */
    rc = ultrasonic_ble_init(ULTRASONIC_NODE_ID, ULTRASONIC_X_POS, &last_distance_mm);
    if (rc != 0) {
        printk("ERROR: Failed to initialize BLE: %d\n", rc);
        return rc;
    }

    /* Start BLE broadcasting */
    rc = ultrasonic_ble_start_broadcasting();
    if (rc != 0) {
        printk("ERROR: Failed to start BLE broadcasting: %d\n", rc);
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