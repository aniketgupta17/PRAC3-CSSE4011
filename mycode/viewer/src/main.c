#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <stdio.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

// M5Core2 screen resolution
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

// Grid units in meters
#define GRID_X_MIN 0.0
#define GRID_X_MAX 6.0
#define GRID_Y_MIN 0.0
#define GRID_Y_MAX 4.0

// Pixel margins
#define SCREEN_MARGIN_X 10
#define SCREEN_MARGIN_Y 10

#define DRAW_AREA_X_MIN SCREEN_MARGIN_X
#define DRAW_AREA_X_MAX (SCREEN_WIDTH - SCREEN_MARGIN_X)
#define DRAW_AREA_Y_MIN SCREEN_MARGIN_Y
#define DRAW_AREA_Y_MAX (SCREEN_HEIGHT - SCREEN_MARGIN_Y)

// Convert grid to screen coordinates
static int screen_x(double x) {
    return DRAW_AREA_X_MIN + (int)((x - GRID_X_MIN) / (GRID_X_MAX - GRID_X_MIN) * (DRAW_AREA_X_MAX - DRAW_AREA_X_MIN));
}
static int screen_y(double y) {
    return DRAW_AREA_Y_MAX - (int)((y - GRID_Y_MIN) / (GRID_Y_MAX - GRID_Y_MIN) * (DRAW_AREA_Y_MAX - DRAW_AREA_Y_MIN));
}

// Red position marker
static lv_obj_t *position_marker;

void draw_grid(void) {
    const struct { char id; double x, y; } beacons[] = {
        {'A', 0, 0}, {'B', 1.5, 0}, {'C', 3, 0}, {'D', 3, 2},
        {'E', 3, 4}, {'F', 1.5, 4}, {'G', 0, 4}, {'H', 0, 2},
        {'I', 4.5, 0}, {'J', 6, 0}, {'K', 6, 2}, {'L', 6, 4}, {'M', 4.5, 4}
    };

    // Draw horizontal lines at y = 0, 2, 4
    int y_lines[] = {0, 2, 4};
    for (int i = 0; i < 3; i++) {
        lv_point_t *line_pts = k_malloc(sizeof(lv_point_t) * 2);
        line_pts[0].x = screen_x(0);
        line_pts[0].y = screen_y(y_lines[i]);
        line_pts[1].x = screen_x(6);
        line_pts[1].y = screen_y(y_lines[i]);

        lv_obj_t *line = lv_line_create(lv_scr_act());
        lv_line_set_points(line, line_pts, 2);
        lv_obj_set_style_line_color(line, lv_color_white(), 0);
        lv_obj_set_style_line_width(line, 1, 0);
    }

    // Draw vertical lines at x = 0, 1.5, 3.0, 4.5, 6.0
    double x_lines[] = {0.0, 1.5, 3.0, 4.5, 6.0};
    for (int i = 0; i < 5; i++) {
        lv_point_t *line_pts = k_malloc(sizeof(lv_point_t) * 2);
        line_pts[0].x = screen_x(x_lines[i]);
        line_pts[0].y = screen_y(0);
        line_pts[1].x = screen_x(x_lines[i]);
        line_pts[1].y = screen_y(4);

        lv_obj_t *line = lv_line_create(lv_scr_act());
        lv_line_set_points(line, line_pts, 2);
        lv_obj_set_style_line_color(line, lv_color_white(), 0);
        lv_obj_set_style_line_width(line, 1, 0);
    }

    // Beacon labels
    for (int i = 0; i < sizeof(beacons)/sizeof(beacons[0]); i++) {
        int x_px = screen_x(beacons[i].x);
        int y_px = screen_y(beacons[i].y);

        int x_offset = 0;
        int y_offset = 0;

        if (beacons[i].x <= GRID_X_MIN + 0.1) x_offset = 6;
        else if (beacons[i].x >= GRID_X_MAX - 0.1) x_offset = -10;

        if (beacons[i].y <= GRID_Y_MIN + 0.1) y_offset = -10;
        else if (beacons[i].y >= GRID_Y_MAX - 0.1) y_offset = 6;

        lv_obj_t *label = lv_label_create(lv_scr_act());
        lv_label_set_text_fmt(label, "%c", beacons[i].id);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, x_px + x_offset, y_px + y_offset);
    }
}

int main(void) {
    const struct device *display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_ERR("Display device not ready");
        return 0;
    }

    // Initialize screen
    lv_obj_clean(lv_scr_act());
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(lv_scr_act(), LV_SCROLLBAR_MODE_OFF);

    draw_grid();

    // Create red marker
    position_marker = lv_obj_create(lv_scr_act());
    lv_obj_set_size(position_marker, 12, 12);
    lv_obj_set_style_bg_color(position_marker, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_radius(position_marker, LV_RADIUS_CIRCLE, 0);

    display_blanking_off(display_dev);  // Turn screen on

    // Simulated position updates
    const double mock_path[][2] = {
        {0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {2.5, 2.0}, {3.0, 2.0},
        {4.0, 2.5}, {6.0, 4.0}, {4.5, 4.0}, {3.0, 4.0}, {1.5, 4.0},
        {0.0, 4.0}, {0.0, 2.0}
    };
    const int path_len = sizeof(mock_path) / sizeof(mock_path[0]);
    int index = 0;

    while (1) {
        double x = mock_path[index][0];
        double y = mock_path[index][1];

        // Center marker at position
        lv_obj_align(position_marker, LV_ALIGN_TOP_LEFT, screen_x(x) - 6, screen_y(y) - 6);

        index = (index + 1) % path_len;

        // Animate for 2 seconds
        for (int i = 0; i < 100; i++) {
            lv_timer_handler();
            k_msleep(20);
        }
    }
}
