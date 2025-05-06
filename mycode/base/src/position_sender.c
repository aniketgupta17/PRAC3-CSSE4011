#include <zephyr/net/socket.h>
#include <zephyr/data/json.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_ip.h>
#include "position_sender.h"

LOG_MODULE_REGISTER(position_sender, LOG_LEVEL_INF);

// Replace with your server IP and port
#define SERVER_ADDR "192.168.1.100"
#define SERVER_PORT 5001

struct position_data {
    double x;
    double y;
    double vx;
    double vy;
};

static const struct json_obj_descr position_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct position_data, x, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct position_data, y, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct position_data, vx, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct position_data, vy, JSON_TOK_NUMBER),
};

void send_position_data(double x, double y, double vx, double vy) {
    struct position_data pos = { x, y, vx, vy };
    char json_buf[128];

    int ret = json_obj_encode_buf(position_descr, ARRAY_SIZE(position_descr), &pos, json_buf, sizeof(json_buf));
    if (ret < 0) {
        LOG_ERR("JSON encoding failed: %d", ret);
        return;
    }

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Socket creation failed: %d", errno);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
    };
    inet_pton(AF_INET, SERVER_ADDR, &addr.sin_addr);

    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Connection failed: %d", errno);
        close(sock);
        return;
    }

    char http_buf[512];
    snprintf(http_buf, sizeof(http_buf),
             "POST /position HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n\r\n%s",
             SERVER_ADDR, strlen(json_buf), json_buf);

    ret = send(sock, http_buf, strlen(http_buf), 0);
    if (ret < 0) {
        LOG_ERR("Failed to send request: %d", errno);
    } else {
        LOG_INF("Position sent: x=%.2f y=%.2f vx=%.2f vy=%.2f", x, y, vx, vy);
    }

    close(sock);
}