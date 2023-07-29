/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/


/*********************************************************************
MIT License

Copyright (c) 2023 touchgadgetdev@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*********************************************************************/

/* This program converts two USB arcade controllers to a Nintendo
 * Switch compatible gamepad. Some code is taken from an Adafruit example
 * program so Adafruit's copyright is included.
 *
 * - Device run on the native usb controller with type C USB connector.
 * - Host run on bit-banging 2 GPIOs with the help of Pico-PIO-USB library
 *   with type A USB connector.
 *
 * Requirements:
 * - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 * - 2 consecutive GPIOs: D+ is defined by PIN_PIO_USB_HOST_DP, D- = D+ +1
 * - Provide VBus (5v) and GND for peripheral
 * - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

// Set this to 0 for use with a Nintendo Switch.
#define USB_DEBUG 0

#if USB_DEBUG
#define DBG_print(...)    Serial.print(__VA_ARGS__)
#define DBG_println(...)  Serial.println(__VA_ARGS__)
#define DBG_printf(...)   Serial.printf(__VA_ARGS__)
#else
#define DBG_print(...)
#define DBG_println(...)
#define DBG_printf(...)
#endif

#include <stdint.h>

/************************************************************/

typedef struct __attribute__ ((packed)) {
  uint8_t x;
  uint8_t y;
  uint8_t z;
  uint8_t z2;
  uint8_t Rz;
  uint16_t hat:4;
  uint16_t buttons:12;
  uint8_t vendorspec1;
} Arcade_t ;

// Arcade USB controller
typedef struct arcade_state {
  Arcade_t report;
  const uint16_t USB_VID = 0x0079;
  const uint16_t USB_PID = 0x0006;
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t report_len;
  bool connected = false;
  bool available = false;
  bool debug = false;
} Arcade_state_t;

#define MAX_ARCADE  (2)
volatile Arcade_state_t Arcade[MAX_ARCADE];

// pio-usb is required for rp2040 usb host
#include "pio_usb.h"
#include "pio-usb-host-pins.h"
#include "Adafruit_TinyUSB.h"
#include "switch_tinyusb.h"

// USB Device gamepad
Adafruit_USBD_HID G_usb_hid;
NSGamepad Gamepad(&G_usb_hid);

// USB Host object for arcade controllers
Adafruit_USBH_Host USBHost;

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup()
{
#if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
  // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
  TinyUSB_Device_Init(0);
#endif
#if USB_DEBUG
  Serial.begin(115200);
#else
  Serial.end();     // Remove CDC ACM port
#endif
  TinyUSBDevice.setID(0x0f0d, 0x00ee);
  Gamepad.begin();

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif

  DBG_println("Switch TinyUSB Gamepad mounted");
}

// Swap buttons 0 and 2 so the joystick trigger maps to the gamepad A button.
uint16_t remap(uint16_t buttons) {
  uint16_t b0 = buttons & (1<<0);
  uint16_t b2 = (buttons & (1<<2));
  buttons &= ~5;
  buttons |= (b2 >> 2) | (b0 << 2);
  return buttons;
}

void loop() {
  delay(1);
  for (size_t instance = 0; instance < MAX_ARCADE; instance++) {
    if (Arcade[instance].connected) {
      if (Arcade[instance].available) {
        if (sizeof(Arcade[instance].report) == Arcade[instance].report_len) {
          if (Arcade[instance].debug) {
            uint8_t *rpt = (uint8_t *)&Arcade[instance].report;
            DBG_printf("Arcade[instance] report(%d): ", Arcade[instance].report_len);
            for (uint16_t i = 0; i < Arcade[instance].report_len; i++) {
              DBG_printf("0x%02X ", rpt[i]);
            }
            DBG_println();
          }
          // Remote wakeup
          if ( TinyUSBDevice.suspended() ) {
            // Wake up host if we are in suspend mode
            // and REMOTE_WAKEUP feature is enabled by host
            TinyUSBDevice.remoteWakeup();
          }
          if (Gamepad.ready()) {
            volatile Arcade_t *rpt = &Arcade[instance].report;
            static uint16_t old_buttons = 0;
            uint16_t buttons = old_buttons;
            if (Arcade[instance].debug) {
              DBG_printf("x=%4d, y=%4d, buttons=0x%02x\r\n",
                  rpt->x, rpt->y, rpt->buttons);
            }
            if (instance == 0) {
              uint16_t left_trigger = (rpt->buttons & (1<<0)) << 4;
              uint16_t left_throttle = (rpt->buttons & (1<<1)) << 5;
              uint16_t minus = (rpt->buttons & (1<<2)) << 6;
              uint16_t left_stick = (rpt->buttons & (1<<3)) << 7;
              uint16_t capture = (rpt->buttons & (1<<4)) << 9;
              buttons = (buttons & ~0b10010101010000U) |
                left_trigger | left_throttle | minus | left_stick | capture;
              bool up = rpt->buttons & (1<<5);
              bool down = rpt->buttons & (1<<6);
              bool left = rpt->buttons & (1<<7);
              bool right = rpt->buttons & (1<<8);
              Gamepad.dPad(up, down, left, right);
              Gamepad.leftXAxis(rpt->x);
              Gamepad.leftYAxis(rpt->y);
            } else {
              uint16_t right_trigger = (rpt->buttons & (1<<0)) << 5;
              uint16_t right_throttle = (rpt->buttons & (1<<1)) << 6;
              uint16_t plus = (rpt->buttons & (1<<2)) << 7;
              uint16_t right_stick = (rpt->buttons & (1<<3)) << 8;
              uint16_t home = (rpt->buttons & (1<<4)) << 8;
              uint16_t ybax = (rpt->buttons & (0b1111U << 5)) >> 5;
              buttons = ((buttons & ~0b1101010101111U)) | ybax |
                right_trigger | right_throttle | plus | right_stick | home;
              Gamepad.rightXAxis(rpt->x);
              Gamepad.rightYAxis(rpt->y);
            }
            Gamepad.buttons(buttons);
            Gamepad.loop();
            old_buttons = buttons;
          }
        }
        Arcade[instance].available = false;
      }
    }
  }
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

void setup1() {
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif
  DBG_println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
#if USB_DEBUG
    while (!Serial) { delay(1); }
#endif
    DBG_printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    DBG_println("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed");
    while(1) delay(1);
  }

#ifdef PIN_PIO_USB_HOST_VBUSEN
  pinMode(PIN_PIO_USB_HOST_VBUSEN, OUTPUT);
  digitalWrite(PIN_PIO_USB_HOST_VBUSEN, PIN_PIO_USB_HOST_VBUSEN_STATE);
#endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_PIO_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

// core1's loop
void loop1()
{
  USBHost.task();
}

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  DBG_printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  DBG_printf("VID = %04x, PID = %04x\r\n", vid, pid);
  DBG_print("HID report descriptor: ");
  for (size_t i = 0; i < desc_len; i++) {
    DBG_print(desc_report[i], HEX);
    DBG_print(',');
  }
  DBG_println();
  if ((vid == Arcade[instance].USB_VID) && (pid == Arcade[instance].USB_PID)) {
    DBG_printf("Arcade[instance] (%d) connected\r\n", instance);
    Arcade[instance].connected = true;
    Arcade[instance].available = false;
    Arcade[instance].dev_addr = dev_addr;
    Arcade[instance].instance = instance;
    memset((Arcade_t *)&Arcade[instance].report, 0, sizeof(Arcade[instance].report));
  }
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  DBG_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  if ((Arcade[instance].dev_addr == dev_addr) && (Arcade[instance].instance == instance)) {
    if (Arcade[instance].connected) {
      Arcade[instance].connected = false;
      Arcade[instance].available = false;
      DBG_printf("Arcade controller (%d) disconnected\r\n", instance);
    }
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (Arcade[instance].connected && (Arcade[instance].dev_addr == dev_addr) && (Arcade[instance].instance == instance)) {
    memcpy((Arcade_t *)&Arcade[instance].report, report, min(sizeof(Arcade[instance].report), len));
    Arcade[instance].report_len = len;
    Arcade[instance].available = true;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}
