/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <zephyr/kernel.h>
 #include <zephyr/sys/printk.h>
 #include <zephyr/bluetooth/bluetooth.h>
 #include <zephyr/bluetooth/conn.h>
 #include <zephyr/bluetooth/gatt.h>
 #include <zephyr/bluetooth/hci.h>
 
 static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11
 );
 static struct bt_uuid_16 char_uuid = BT_UUID_INIT_16(1849);
 static struct bt_uuid_16 uuid = BT_UUID_INIT_16(1849); // In read_cmd // Appearance (fallback)
 
 static struct bt_gatt_discover_params discover_params;
 static struct bt_gatt_read_params read_params;
 static uint16_t char_handle = 0;
 static uint16_t service_end_handle = 0xffff; // Track service end handle
 
 extern int mtu_exchange(struct bt_conn *conn);
//  extern int write_cmd(struct bt_conn *conn);
 extern struct bt_conn *conn_connected;
 extern uint32_t last_write_rate;
 extern void (*start_scan_func)(void);
 
 /* Flag to track if the target UUID was found */
 static bool uuid_found_flag = false;
 
 /* Log raw advertising data for debugging */
 static void log_ad_data(struct net_buf_simple *ad)
 {
     printk("Raw AD data (len %u): ", ad->len);
     for (size_t i = 0; i < ad->len; i++) {
         printk("%02x ", ad->data[i]);
     }
     printk("\n");
 }
 
 /* Callback to parse advertising data and check for service UUID */
 static bool ad_data_cb(struct bt_data *data, void *user_data)
 {
     struct bt_uuid *target_uuid = (struct bt_uuid *)user_data;
 
     /* Log each AD structure */
     printk("AD type 0x%02x, len %u: ", data->type, data->data_len);
     for (size_t i = 0; i < data->data_len; i++) {
         printk("%02x ", data->data[i]);
     }
     printk("\n");
 
     /* Check for 128-bit UUIDs (complete or incomplete list) */
     if (data->type == BT_DATA_UUID128_SOME || data->type == BT_DATA_UUID128_ALL) {
         printk("Found 128-bit UUID list (type 0x%02x, len %u): ", data->type, data->data_len);
         /* Each UUID in the data is 16 bytes (128-bit UUID) */
         for (size_t i = 0; i < data->data_len; i += 16) {
             struct bt_uuid_128 uuid;
             memcpy(uuid.val, &data->data[i], 16);
             uuid.uuid.type = BT_UUID_TYPE_128;
             printk("0x");
             for (int j = 15; j >= 0; j--) {
                 printk("%02x", uuid.val[j]);
                 if (j == 12 || j == 10 || j == 8 || j == 6) {
                     printk("-");
                 }
             }
             if (bt_uuid_cmp(&uuid.uuid, target_uuid) == 0) {
                 uuid_found_flag = true;
                 printk(" (matches target UUID)");
                 return false; /* Stop parsing */
             }
             printk(" ");
         }
         printk("\n");
     }
 
     return true; /* Continue parsing */
 }
 
 static uint8_t read_cb(struct bt_conn *conn, uint8_t err,
    struct bt_gatt_read_params *params,
    const void *data, uint16_t length)
{
printk("✅ read_cb() triggered. err=%d, len=%d\n", err, length);
if (err) {
printk("Read failed (err %d)\n", err);
} else if (data) {
printk("Read %d bytes: ", length);
for (int i = 0; i < length; i++) {
printk("%02x ", ((const uint8_t *)data)[i]);
}
printk("\n");
} else {
printk("Read complete (no more data)\n");
}

return BT_GATT_ITER_STOP;
}

 
 static uint8_t characteristic_discover_func(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            struct bt_gatt_discover_params *params)
 {
     if (!attr) {
         printk("Characteristic discovery complete, no characteristic found for UUID 0x%04x\n", char_uuid.val);
         bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
         return BT_GATT_ITER_STOP;
     }
 
     struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
     uint16_t uuid_val = (chrc->uuid->type == BT_UUID_TYPE_16) ? ((struct bt_uuid_16 *)chrc->uuid)->val : 0;
 
     printk("Found characteristic handle: %u, value handle: %u, UUID 0x%04x\n",
            attr->handle, chrc->value_handle, uuid_val);
 
     char_handle = chrc->value_handle;
 
     memset(&read_params, 0, sizeof(read_params));
     read_params.func = read_cb;
     read_params.handle_count = 1;
     read_params.single.handle = char_handle;
     read_params.single.offset = 0;
 
     int err = bt_gatt_read(conn, &read_params);
     if (err) {
         printk("Read failed (err %d)\n", err);
     }
     printk("✅ Discovery and reading process completed successfully.\n");

     return BT_GATT_ITER_STOP;
 }
 
 static uint8_t service_discover_func(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     struct bt_gatt_discover_params *params)
 {
     if (!attr) {
         printk("Service discovery complete, no service found\n");
         bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
         return BT_GATT_ITER_STOP;
     }
 
     struct bt_gatt_service_val *service = (struct bt_gatt_service_val *)attr->user_data;
     if (service->uuid->type == BT_UUID_TYPE_128) {
         struct bt_uuid_128 *uuid = (struct bt_uuid_128 *)service->uuid;
         printk("Found service. Handle: %u, UUID 0x", attr->handle);
         for (int i = 15; i >= 0; i--) {
             printk("%02x", uuid->val[i]);
             if (i == 12 || i == 10 || i == 8 || i == 6) {
                 printk("-");
             }
         }
         printk(", End Handle: %u\n", service->end_handle);
     } else {
         printk("Found service. Handle: %u, UUID type %u, End Handle: %u\n",
                attr->handle, service->uuid->type, service->end_handle);
     }
 
     service_end_handle = service->end_handle;
 
     memset(&discover_params, 0, sizeof(discover_params));
     discover_params.uuid = &char_uuid.uuid;
     discover_params.start_handle = attr->handle + 1;
     discover_params.end_handle = service_end_handle;
     discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
     discover_params.func = characteristic_discover_func;
 
     int err = bt_gatt_discover(conn, &discover_params);
     if (err) {
         printk("Characteristic discovery failed (err %d)\n", err);
         bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
     }
 
     return BT_GATT_ITER_STOP;
 }
 
 int discover_and_read(struct bt_conn *conn)
 {
     memset(&discover_params, 0, sizeof(discover_params));
 
     discover_params.uuid = &service_uuid.uuid;
     discover_params.func = service_discover_func;
     discover_params.start_handle = 0x0001;
     discover_params.end_handle = 0xffff;
     discover_params.type = BT_GATT_DISCOVER_PRIMARY;
 
     int err = bt_gatt_discover(conn, &discover_params);
     if (err) {
         printk("Service discovery failed (err %d)\n", err);
         bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
     }
 
     return err;
 }
 
 int read_cmd(struct bt_conn *conn)
 {
     static struct bt_gatt_read_params read_params;
     static struct bt_uuid_16 uuid = BT_UUID_INIT_16(1849); // Match char_uuid
 
     memset(&read_params, 0, sizeof(read_params));
 
     read_params.func = read_cb;
     read_params.by_uuid.uuid = &uuid.uuid;
     read_params.by_uuid.start_handle = 0x0001;
     read_params.by_uuid.end_handle = 0xffff;
 
     int err = bt_gatt_read(conn, &read_params);
     if (err) {
         printk("bt_gatt_read failed (err %d)\n", err);
     }
 
     return err;
 }
 
 static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
 {
     char dev[BT_ADDR_LE_STR_LEN];
     struct bt_conn *conn;
     int err;
     if (!is_ibeacon_node(NULL, addr)) {
        printk("Device %s not in iBeacon list, ignoring\n", dev);
        return;
    }
     bt_addr_le_to_str(addr, dev, sizeof(dev));
     printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
            dev, type, ad->len, rssi);
 
     /* Log raw advertising data */
     log_ad_data(ad);
 
     /* Parse advertising data or scan response to log UUIDs */
     uuid_found_flag = false;
     bt_data_parse(ad, ad_data_cb, &service_uuid);
 
     /* Log whether it’s connectable */
     if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
         printk("Non-connectable advertisement (type %u)\n", type);
         /* For debugging, proceed with scan responses to check UUID */
         if (type == BT_GAP_ADV_TYPE_SCAN_RSP && uuid_found_flag) {
             printk("Scan response contains target UUID, but not connectable\n");
         }
         return;
     }
 
     /* Filter by RSSI (disabled for debugging) */
     // if (rssi < -90) {
     //     printk("RSSI too low (%i < -90)\n", rssi);
     //     return;
     // }
 
     /* Check if the target UUID was found */
     if (!uuid_found_flag) {
         printk("Service UUID 11111111-1111-1111-1111-111111111111 not found in advertisement\n");
         return;
     }
 
     printk("Found device advertising service UUID 11111111-1111-1111-1111-111111111111\n");
 
     /* Stop scanning */
     err = bt_le_scan_stop();
     if (err) {
         printk("%s: Stop LE scan failed (err %d)\n", __func__, err);
         return;
     }
 
     /* Connection parameters */
     static const struct bt_le_conn_param conn_params = {
         .interval_min = BT_GAP_INIT_CONN_INT_MIN,
         .interval_max = BT_GAP_INIT_CONN_INT_MAX,
         .latency = 0,
         .timeout = 3000, /* 30 seconds */
     };
 
     /* Attempt to connect */
     err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, &conn_params, &conn);
     if (err) {
         printk("%s: Create conn failed (err %d)\n", __func__, err);
         start_scan_func();
     } else {
        //  bt_conn_unref(conn);
     }
 }
 
 static void start_scan(void)
 {
     int err;
 
     /* Custom scan parameters for better detection */
     static const struct bt_le_scan_param scan_param = {
         .type = BT_LE_SCAN_TYPE_ACTIVE, /* Active scanning */
         .options = BT_LE_SCAN_OPT_NONE,
         .interval = BT_GAP_SCAN_FAST_INTERVAL, /* 0x0040 (40 ms) */
         .window = BT_GAP_SCAN_FAST_WINDOW,     /* 0x0030 (30 ms) */
         .timeout = 0, /* No timeout */
     };
 
     err = bt_le_scan_start(&scan_param, device_found);
     if (err) {
         printk("%s: Scanning failed to start (err %d)\n", __func__, err);
         return;
     }
 
     printk("%s: Scanning successfully started\n", __func__);
 }
 
 static void auth_cancel(struct bt_conn *conn)
 {
     printk("Pairing cancelled\n");
 }
 
 static void pairing_confirm(struct bt_conn *conn)
 {
     printk("Pairing confirmed\n");
     bt_conn_auth_pairing_confirm(conn);
 }
 
 static struct bt_conn_auth_cb auth_cb = {
     .cancel = auth_cancel,
     .pairing_confirm = pairing_confirm,
     /* Removed passkey_entry and passkey_display */
 };
 
 static void connected(struct bt_conn *conn, uint8_t err)
 {
     char addr[BT_ADDR_LE_STR_LEN];
     struct bt_conn_info info;
 
     bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
     if (err) {
         printk("Connection failed (err %d)\n", err);
         start_scan_func();
     } else {
         err = bt_conn_get_info(conn, &info);
         if (err) {
             printk("Failed to get connection info (err %d)\n", err);
         } else {
             printk("Connected to %s, role %d\n", addr, info.role);
             conn_connected = conn;
             /* Try BT_SECURITY_NONE for testing */
             err = bt_conn_set_security(conn,    BT_SECURITY_L0);
             if (err) {
                 printk("Failed to set security (err %d)\n", err);
             }
         }
     }
 }
 
 static void disconnected(struct bt_conn *conn, uint8_t reason)
 {
     char addr[BT_ADDR_LE_STR_LEN];
     bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
     printk("Disconnected from %s (reason 0x%02x)\n", addr, reason);
     conn_connected = NULL;
     service_end_handle = 0xffff;
     start_scan_func();
 }
 
 static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
 {
     char addr[BT_ADDR_LE_STR_LEN];
     bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
     if (!err) {
         printk("Security changed for %s: level %u\n", addr, level);
     } else {
         printk("Security failed for %s: level %u, err %u\n", addr, level, err);
     }
 }
 
 void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
 {
     printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
 }
 
 static struct bt_conn_cb conn_callbacks = {
     .connected = connected,
     .disconnected = disconnected,
     .security_changed = security_changed,
 };
 
 static struct bt_gatt_cb gatt_callbacks = {
     .att_mtu_updated = mtu_updated
 };
 
 uint32_t central_gatt_write(uint32_t count)
 {
     int err;
 count=1;
     err = bt_enable(NULL);
     if (err) {
         printk("Bluetooth init failed (err %d)\n", err);
         return 0U;
     }
     printk("Bluetooth initialized\n");
 
     bt_conn_auth_cb_register(&auth_cb);
     bt_conn_cb_register(&conn_callbacks);
     bt_gatt_cb_register(&gatt_callbacks);
 
     conn_connected = NULL;
     last_write_rate = 0U;
     service_end_handle = 0xffff;
 
     start_scan_func = start_scan;
     start_scan_func();
 
     while (true) {
         struct bt_conn *conn = NULL;
 
         if (conn_connected) {
             conn = bt_conn_ref(conn_connected);
         }
 
         if (conn) {
             err = discover_and_read(conn);
             if (err) {
                 printk("Discovery failed (err %d)\n", err);
                 bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                 bt_conn_unref(conn);
                 break;
             }
 
             bt_conn_unref(conn);
             count =1;
             if (count) {
                 count--;
                 if (!count) {
                     break;
                 }
             }
 
             k_yield();
         } else {
             k_sleep(K_SECONDS(1));
         }
     }
  printk("Executed");
     return last_write_rate;
 }
