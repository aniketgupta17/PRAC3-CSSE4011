#ifndef IBEACON_NODE_MANAGER_H
#define IBEACON_NODE_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/slist.h>

/* Structure to hold iBeacon node information */
struct ibeacon_node {
    sys_snode_t node; /* Linked list node */
    char name[16]; /* BLE Name, e.g., "4011-A" */
    bt_addr_le_t addr; /* BLE MAC address */
    uint16_t major; /* BLE Major number */
    uint16_t minor; /* BLE Minor number */
    float x_coord; /* Fixed X coordinate */
    float y_coord; /* Fixed Y coordinate */
    char left_neighbour[16]; /* Left neighbour BLE Name */
    char right_neighbour[16]; /* Right neighbour BLE Name */
};

/* Initialize the iBeacon node manager */
void ibeacon_node_manager_init(void);
bool is_ibeacon_node(const char *name, const bt_addr_le_t *addr);
/* Shell command to add an iBeacon node */
int cmd_ibeacon_add(const struct shell *sh, size_t argc, char **argv);

/* Shell command to remove an iBeacon node */
int cmd_ibeacon_remove(const struct shell *sh, size_t argc, char **argv);

/* Shell command to view iBeacon node(s) */
int cmd_ibeacon_view(const struct shell *sh, size_t argc, char **argv);

#endif /* IBEACON_NODE_MANAGER_H */
