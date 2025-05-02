/**
 * @file ultrasonic_ble.h
 * @brief BLE communication library for ultrasonic sensor node
 */

#ifndef ULTRASONIC_BLE_H
#define ULTRASONIC_BLE_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>

/**
 * @brief Ultrasonic node position data
 */
struct ultrasonic_position {
    uint8_t node_id;       /* Identifier for this ultrasonic node */
    uint8_t x;             /* Fixed X position on grid (in grid units) */
    uint16_t distance_mm;  /* Distance measured by ultrasonic sensor in mm */
};

/**
 * @brief Initialize BLE for ultrasonic nodes
 * 
 * @param node_id Identifier for this ultrasonic node (1-255)
 * @param x_position Fixed X position of this node on the grid
 * @param distance_atomic Pointer to atomic variable holding the latest distance measurement
 * 
 * @return 0 on success, negative error code on failure
 */
int ultrasonic_ble_init(uint8_t node_id, uint8_t x_position, atomic_t *distance_atomic);

/**
 * @brief Start BLE advertising with ultrasonic distance data
 * 
 * This function starts a background thread that periodically broadcasts
 * the ultrasonic distance data using BLE advertising packets.
 * 
 * @return 0 on success, negative error code on failure
 */
int ultrasonic_ble_start_broadcasting(void);

/**
 * @brief Stop BLE advertising
 * 
 * @return 0 on success, negative error code on failure
 */
int ultrasonic_ble_stop_broadcasting(void);

/**
 * @brief Update the distance value to be broadcast
 * 
 * This function is typically not needed as the library automatically
 * reads from the atomic variable passed during initialization.
 * 
 * @param distance_mm New distance value in millimeters
 */
void ultrasonic_ble_update_distance(uint16_t distance_mm);

#endif /* ULTRASONIC_BLE_H */