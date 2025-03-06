#include "bsp/board_api.h"
#include "class/hid/hid_device.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pmw3360.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include <pico/time.h>
#include <stdio.h>

#define PIN_LMB 16
#define PIN_RMB 17

#define PIN_ENCODER_A 21
#define PIN_ENCODER_B 23

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

    pin_init(PIN_ENCODER_A);
    pin_init(PIN_ENCODER_B);
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

int8_t wheel_progress = 0;
uint8_t wheel_state_a = 0;
uint8_t wheel_state_b = 0;
uint8_t wheel_state_output = 0;

void update_wheel()
{
    uint8_t wheel_new_a = gpio_get(PIN_ENCODER_A);
    uint8_t wheel_new_b = gpio_get(PIN_ENCODER_B);

    if (wheel_new_a != wheel_state_a || wheel_new_b != wheel_state_b) {
        if (wheel_new_a == wheel_new_b && wheel_state_output != wheel_new_a) {
            // when scrolling up, B changes first. when scrolling down, A changes first
            if (wheel_new_b == wheel_state_b)
                wheel_progress += 1;
            // the wheel can glitch and jump straight from 00 to 11 (or vice versa)
            // so we need to check that only one state has changed since the last test
            else if(wheel_new_a == wheel_state_a)
                wheel_progress -= 1;

            wheel_state_output = wheel_new_a;
        }
        wheel_state_a = wheel_new_a;
        wheel_state_b = wheel_new_b;
    }
}

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
        if(tud_hid_n_ready(0)) {
            int16_t dx, dy;
            pmw3360_get_deltas(&dx, &dy);
            uint8_t buttons = 0x00;
            buttons |= (!gpio_get(PIN_LMB) << 0);
            buttons |= (!gpio_get(PIN_RMB) << 1);
            int8_t wheel = wheel_progress;
            wheel_progress = 0;
            tud_hid_n_mouse_report(0, REPORT_ID_MOUSE, buttons, -dx, -dy, wheel, 0);
        }
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

const char reset_string[] = "Restarted";
const char set_cpi_string[] = "Set CPI";
const char unknown_report_string[] = "Unknown report type";

char message_buffer[64];

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_type;
    switch(buffer[0]) {
    case 0:
        pmw3360_set_cpi(buffer[1] * 100);
        sprintf(message_buffer, "%s %i", set_cpi_string, buffer[1] * 100);
        break;
    case 1:
        sprintf(message_buffer, "%s", reset_string);
        tud_hid_n_report(1, 0, message_buffer, 64);
        sleep_ms(1000);
        reset_usb_boot(1<<PICO_DEFAULT_LED_PIN,0);
    default:
        sprintf(message_buffer, "%s", unknown_report_string);
    }
    tud_hid_n_report(1, 0, message_buffer, 64);
}