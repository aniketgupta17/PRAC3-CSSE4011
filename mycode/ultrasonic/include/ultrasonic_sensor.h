/**
 * @file ultrasonic_sensor.h
 * @brief HC-SR04 ultrasonic sensor driver header
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

/* HC-SR04 specific constants */
#define HC_SR04_MIN_DISTANCE_MM    20    /* Minimum distance (mm) */
#define HC_SR04_MAX_DISTANCE_MM    4000  /* Maximum distance (mm) */

/**
 * @brief Initialize the HC-SR04 ultrasonic sensor
 *
 * @param trigger_dev Pointer to the trigger GPIO device
 * @param trigger_pin Trigger GPIO pin number
 * @param echo_dev Pointer to the echo GPIO device
 * @param echo_pin Echo GPIO pin number
 * @return 0 on success, negative error code on failure
 */
int hc_sr04_init(const struct device *trigger_dev, uint8_t trigger_pin,
                const struct device *echo_dev, uint8_t echo_pin);

/**
 * @brief Measure distance using HC-SR04 ultrasonic sensor
 *
 * @param trigger_dev Pointer to the trigger GPIO device
 * @param trigger_pin Trigger GPIO pin number
 * @param echo_dev Pointer to the echo GPIO device
 * @param echo_pin Echo GPIO pin number
 * @param distance_mm Pointer to store the measured distance in millimeters
 * @return 0 on success, negative error code on failure
 */
int hc_sr04_measure(const struct device *trigger_dev, uint8_t trigger_pin,
                   const struct device *echo_dev, uint8_t echo_pin,
                   uint16_t *distance_mm);

/**
 * @brief Start the ultrasonic sensor thread
 *
 * @param trigger_dev Pointer to the trigger GPIO device
 * @param trigger_pin Trigger GPIO pin number
 * @param echo_dev Pointer to the echo GPIO device
 * @param echo_pin Echo GPIO pin number
 * @param system_ready Pointer to atomic variable to signal system readiness
 * @param last_distance_mm Pointer to atomic variable to store the last measured distance
 * @return 0 on success, negative error code on failure
 */
int ultrasonic_thread_start(const struct device *trigger_dev, uint8_t trigger_pin,
                           const struct device *echo_dev, uint8_t echo_pin,
                           atomic_t *system_ready, atomic_t *last_distance_mm);

/**
 * @brief Get the last measured distance
 *
 * @return Last measured distance in millimeters
 */
uint16_t get_last_distance(void);

#endif /* ULTRASONIC_SENSOR_H */