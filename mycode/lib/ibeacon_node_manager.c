#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/slist.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <string.h>
#include <stdlib.h>
#include "ibeacon_node_manager.h"

/* Linked list to store iBeacon nodes */
static sys_slist_t ibeacon_list;

bool is_ibeacon_node(const char *name, const bt_addr_le_t *addr)
{
    struct ibeacon_node *node;
    SYS_SLIST_FOR_EACH_CONTAINER(&ibeacon_list, node, node) {
        if (name && strcmp(node->name, name) == 0) {
            return true;
        }
        if (addr && bt_addr_le_cmp(&node->addr, addr) == 0) {
            return true;
        }
    }
    return false;
}

/* Initialize the iBeacon node manager */
void ibeacon_node_manager_init(void)
{
    sys_slist_init(&ibeacon_list);
    printk("iBeacon node manager initialized\n");
}

/* Helper function to find a node by name */
static struct ibeacon_node *find_node_by_name(const char *name)
{
    struct ibeacon_node *node;
    SYS_SLIST_FOR_EACH_CONTAINER(&ibeacon_list, node, node) {
        if (strcmp(node->name, name) == 0) {
            return node;
        }
    }
    return NULL;
}

/* Shell command to add an iBeacon node */
int cmd_ibeacon_add(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 9) {
        shell_error(sh, "Usage: ibeacon add <name> <addr> <major> <minor> <x> <y> <left> <right>");
        return -EINVAL;
    }

    struct ibeacon_node *new_node = k_malloc(sizeof(struct ibeacon_node));
    if (!new_node) {
        shell_error(sh, "Failed to allocate memory for new node");
        return -ENOMEM;
    }

    /* Populate node fields */
    strncpy(new_node->name, argv[1], sizeof(new_node->name) - 1);
    new_node->name[sizeof(new_node->name) - 1] = '\0';

    /* Parse BLE address */
    char addr_str[18];
    strncpy(addr_str, argv[2], sizeof(addr_str) - 1);
    addr_str[sizeof(addr_str) - 1] = '\0';
    int err = bt_addr_le_from_str(addr_str, "random", &new_node->addr);
    if (err) {
        shell_error(sh, "Invalid BLE address format");
        k_free(new_node);
        return -EINVAL;
    }

    new_node->major = atoi(argv[3]);
    new_node->minor = atoi(argv[4]);
    new_node->x_coord = atoi(argv[5]);
    new_node->y_coord = atoi(argv[6]);
    strncpy(new_node->left_neighbour, argv[7], sizeof(new_node->left_neighbour) - 1);
    new_node->left_neighbour[sizeof(new_node->left_neighbour) - 1] = '\0';
    strncpy(new_node->right_neighbour, argv[8], sizeof(new_node->right_neighbour) - 1);
    new_node->right_neighbour[sizeof(new_node->right_neighbour) - 1] = '\0';

    /* Check for duplicate name */
    if (find_node_by_name(new_node->name)) {
        shell_error(sh, "Node with name %s already exists", new_node->name);
        k_free(new_node);
        return -EEXIST;
    }

    /* Add to linked list */
    sys_slist_append(&ibeacon_list, &new_node->node);
    shell_print(sh, "Added iBeacon node: %s", new_node->name);
    return 0;
}

/* Shell command to remove an iBeacon node */
int cmd_ibeacon_remove(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Usage: ibeacon remove <name>");
        return -EINVAL;
    }

    struct ibeacon_node *node = find_node_by_name(argv[1]);
    if (!node) {
        shell_error(sh, "Node %s not found", argv[1]);
        return -ENOENT;
    }

    sys_slist_remove(&ibeacon_list, NULL, &node->node);
    shell_print(sh, "Removed iBeacon node: %s", node->name);
    k_free(node);
    return 0;
}

/* Shell command to view iBeacon node(s) */
int cmd_ibeacon_view(const struct shell *sh, size_t argc, char **argv)
{
    bool view_all = (argc == 2 && strcmp(argv[1], "-a") == 0);
    if (argc > 2 || (argc == 2 && !view_all)) {
        shell_error(sh, "Usage: ibeacon view [-a]");
        return -EINVAL;
    }

    if (view_all) {
        struct ibeacon_node *node;
        SYS_SLIST_FOR_EACH_CONTAINER(&ibeacon_list, node, node) {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&node->addr, addr_str, sizeof(addr_str));
            shell_print(sh, "Node: %s, Addr: %s, Major: %u, Minor: %u, X: %.2f, Y: %.2f, Left: %s, Right: %s",
                        node->name, addr_str, node->major, node->minor,
                        node->x_coord, node->y_coord, node->left_neighbour, node->right_neighbour);
        }
        if (sys_slist_is_empty(&ibeacon_list)) {
            shell_print(sh, "No iBeacon nodes in list");
        }
    } else {
        shell_print(sh, "Specify -a to view all nodes");
    }
    return 0;
}

/* Shell command registration */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_ibeacon,
    SHELL_CMD(add, NULL, "Add iBeacon node: <name> <addr> <major> <minor> <x> <y> <left> <right>", cmd_ibeacon_add),
    SHELL_CMD(remove, NULL, "Remove iBeacon node: <name>", cmd_ibeacon_remove),
    SHELL_CMD(view, NULL, "View iBeacon node(s): [-a]", cmd_ibeacon_view),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ibeacon, &sub_ibeacon, "iBeacon node management commands", NULL);
