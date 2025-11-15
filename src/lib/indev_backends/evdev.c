/**
 *
 * @file evdev.c
 *
 * The lib evdev driver
 *
 * Based on the original file from the repository
 *
 * - Move the driver to a separate file to avoid excessive conditional
 *   compilation
 *   Author: EDGEMTech Ltd, Erik Tagirov (erik.tagirov@edgemtech.ch)
 *
 * Copyright (c) 2025 EDGEMTech Ltd.
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <string.h>

#include "lvgl/lvgl.h"
#if LV_USE_EVDEV
#include "lvgl/src/core/lv_global.h"
#include "../backends.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void indev_deleted_cb(lv_event_t *e);
static void discovery_cb(lv_indev_t *indev, lv_evdev_type_t type, void *user_data);
static void set_mouse_cursor_icon(lv_indev_t *indev, lv_display_t *display);
static lv_indev_t *init_pointer_evdev(lv_display_t *display);
static const char* find_touch_device_fallback(void);

/**********************
 *  STATIC VARIABLES
 **********************/

static char *backend_name = "EVDEV";

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/*
 * Initialize the evdev driver
 *
 * @param backend the backend descriptor
 */
int backend_init_evdev(backend_t *backend)
{
    LV_ASSERT_NULL(backend);
    backend->handle->indev = malloc(sizeof(indev_backend_t));
    LV_ASSERT_NULL(backend->handle->indev);

    backend->handle->indev->init_indev = init_pointer_evdev;

    backend->name = backend_name;
    backend->type = BACKEND_INDEV;
    return 0;
}


/**********************
 *   STATIC FUNCTIONS
 **********************/

/*
 * Remove cursor icon
 *
 * @description When the indev is deleted remove the mouse cursor icon
 * @note called by the LVGL evdev driver
 * @param e the deletion event
 */
static void indev_deleted_cb(lv_event_t *e)
{
    if(LV_GLOBAL_DEFAULT()->deinit_in_progress) return;
    lv_obj_t *cursor_obj = lv_event_get_user_data(e);
    lv_obj_delete(cursor_obj);
}


/*
 * Set cursor icon
 *
 * @description Once the input device is discovered set the mouse cursor icon
 * @note called by the LVGL evdev driver
 * @param indev the input device
 * @param type the type of the input device
 * @param user_data the user data
 */
#if LV_USE_EVEDV_DISCOVER
static void discovery_cb(lv_indev_t *indev, lv_evdev_type_t type, void *user_data)
{
    LV_LOG_USER("new '%s' device discovered", type == LV_EVDEV_TYPE_REL ? "REL" :
                                              type == LV_EVDEV_TYPE_ABS ? "ABS" :
                                              type == LV_EVDEV_TYPE_KEY ? "KEY" :
                                              "unknown");

    lv_display_t *disp = user_data;
    lv_indev_set_display(indev, disp);

    if(type == LV_EVDEV_TYPE_REL) {
        set_mouse_cursor_icon(indev, disp);
    }
}
#endif

/*
 * Set cursor icon
 *
 * @description Enables a pointer (touchscreen/mouse) input device
 * @param display the display on which to create
 * @param indev the input device to set the cursor on
 */
static void set_mouse_cursor_icon(lv_indev_t *indev, lv_display_t *display)
{
    /* Set the cursor icon */
    LV_IMAGE_DECLARE(mouse_cursor_icon);
    lv_obj_t *cursor_obj = lv_image_create(lv_display_get_screen_active(display));
    lv_image_set_src(cursor_obj, &mouse_cursor_icon);
    lv_indev_set_cursor(indev, cursor_obj);

    /* delete the mouse cursor icon if the device is removed */
    lv_indev_add_event_cb(indev, indev_deleted_cb, LV_EVENT_DELETE, cursor_obj);
}

/*
 * Fallback: Auto-detect touch device
 *
 * @description Searches for touch device if symlink not found
 * @return device path or NULL
 */
static const char* find_touch_device_fallback(void)
{
    static char device_path[64];
    char name[256];

    LV_LOG_WARN("Touchscreen symlink not found, auto-detecting...");

    for (int i = 0; i < 32; i++) {
        snprintf(device_path, sizeof(device_path), "/dev/input/event%d", i);

        int fd = open(device_path, O_RDONLY);
        if (fd < 0) continue;

        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) >= 0) {
            // Search for ADS7846 or generic touch devices
            if (strstr(name, "ADS7846") ||
                strstr(name, "Touchscreen") ||
                strstr(name, "Touch") ||
                strstr(name, "touch")) {
                LV_LOG_USER("Auto-detected: %s (%s)", device_path, name);
                close(fd);
                return device_path;
            }
        }
        close(fd);
    }

    LV_LOG_ERROR("No touch device found in /dev/input/event*");
    return NULL;
}

/*
 * Initialize a mouse pointer device
 *
 * Enables a pointer (touchscreen/mouse) input device
 * Priority:
 * 1. Environment variable LV_LINUX_EVDEV_POINTER_DEVICE
 * 2. udev symlink /dev/input/touchscreen
 * 3. Auto-detection by device name
 *
 * @param display the LVGL display
 *
 * @return input device or NULL on failure
 */
static lv_indev_t *init_pointer_evdev(lv_display_t *display)
{
    const char *input_device = getenv("LV_LINUX_EVDEV_POINTER_DEVICE");

    if (input_device == NULL) {
        // Priority 1: Try stable udev symlink
        if (access("/dev/input/touchscreen", F_OK) == 0) {
            input_device = "/dev/input/touchscreen";
            LV_LOG_USER("Using udev symlink: %s", input_device);
        }
        // Priority 2: Try SPI-specific symlink
        else if (access("/dev/input/touchscreen-spi", F_OK) == 0) {
            input_device = "/dev/input/touchscreen-spi";
            LV_LOG_USER("Using SPI symlink: %s", input_device);
        }
        // Priority 3: Auto-detect
        else {
            input_device = find_touch_device_fallback();
            if (input_device == NULL) {
                LV_LOG_ERROR("Touch device not found! Check udev rules.");
                return NULL;
            }
        }
    } else {
        LV_LOG_USER("Using env device: %s", input_device);
    }

    lv_indev_t *indev = lv_evdev_create(LV_INDEV_TYPE_POINTER, input_device);

    if (indev == NULL) {
        LV_LOG_ERROR("Failed to open: %s", input_device);
        return NULL;
    }

    lv_indev_set_display(indev, display);

    // Touch calibration for ADS7846
    lv_evdev_set_calibration(indev, 0, 4095, 4095, 0);

    LV_LOG_USER("Touch device initialized: %s", input_device);

    return indev;
}

#endif /*#if LV_USE_EVDEV*/