#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "pmw3360.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include <stdio.h>

#define PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE
#define PIN_LMB 16
#define PIN_RMB 17

void hid_task();

void pin_init(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void pins_init(void)
{
    pin_init(PIN_LMB);
    pin_init(PIN_RMB);
}

int main()
{
    // Initialize chosen serial port
    stdio_init_all();
    pmw3360_init();
    board_init();
    pins_init();

    tusb_init();

    while (1) {
        tud_task(); // tinyusb device task
        hid_task();
    }
}

void tud_mount_cb(void) { }
void tud_unmount_cb(void) { }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; }
void tud_resume_cb(void) { }

void hid_task(void)
{
    // Poll every 10ms
    const uint32_t interval_ms = 10;
    static uint32_t start_ms = 0;

    if (board_millis() - start_ms < interval_ms)
        return; // not enough time
    start_ms += interval_ms;

    // Remote wakeup
    if (tud_suspended()) {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        tud_remote_wakeup();
    } else {
        // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
        if (!tud_hid_ready())
            return;
        int16_t dx, dy;
        pmw3360_get_deltas(&dx, &dy);
        uint8_t buttons = 0x00;
        buttons |= (!gpio_get(PIN_LMB) << 0);
        buttons |= (!gpio_get(PIN_RMB) << 1);
        tud_hid_mouse_report(REPORT_ID_MOUSE, buttons, -dx, -dy, 0, 0);
    }
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
    (void)instance;
    (void)report;
    (void)len;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    // TODO not Implemented
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}
