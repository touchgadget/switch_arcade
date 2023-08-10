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
#include <limits.h>

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

typedef enum HID_type {
  HID_dev_unknown,
  HID_dev_keyboard,
  HID_dev_mouse,
  HID_dev_joystick,
  HID_dev_gamepad,
  HID_dev_arcade_stick,
} HID_type_t;

const uint16_t ARCADE_VID = 0x0079;
const uint16_t ARCADE_PID = 0x0006;

typedef struct HID_device_state {
  HID_type_t hid_type;
  uint8_t report[64];
  uint8_t report_len;
  uint8_t dev_addr;
  uint8_t instance;
  int16_t xmin;
  int16_t xmax;
  int16_t ymin;
  int16_t ymax;
  uint32_t report_count;
  uint32_t available_count;
  uint32_t last_millis;
  bool available;
  bool connected;
  bool skip_report_id;
} HID_device_state_t;

#define MAX_HID_DEVICES (4)
volatile HID_device_state_t HID_devices[MAX_HID_DEVICES];

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

uint8_t X_SENSITIVITY[129];
uint8_t Y_SENSITIVITY[129];

void sensitivity(float x_speed, float y_speed, uint8_t deadzone) {
  for (size_t i = 0; i < 128; i++) {
    if (i < deadzone) {
      X_SENSITIVITY[i] = 0;
      Y_SENSITIVITY[i] = 0;
    } else {
      X_SENSITIVITY[i] = int((pow(i/127.0f, x_speed) * 127) + 0.5);
      Y_SENSITIVITY[i] = int((pow(i/127.0f, y_speed) * 127) + 0.5);
      if (i > 0) {
        if (X_SENSITIVITY[i] == 0) X_SENSITIVITY[i] = 1;
        if (Y_SENSITIVITY[i] == 0) Y_SENSITIVITY[i] = 1;
      }
    }
    DBG_printf("%d: X_SENSITIVITY %d Y_SENSITIVITY %d\r\n",
        i, X_SENSITIVITY[i], Y_SENSITIVITY[i]);
  }
  X_SENSITIVITY[128] = X_SENSITIVITY[127];
  Y_SENSITIVITY[128] = Y_SENSITIVITY[127];
}

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
  while( !TinyUSBDevice.mounted() ) delay(10);
#if USB_DEBUG
  while (!Serial && (millis() < 3000)) { delay(10); }
#endif

  DBG_println("Switch TinyUSB Gamepad mounted");
  sensitivity(0.5f, 0.5f, 2);
}

// Swap buttons 0 and 2 so the joystick trigger maps to the gamepad A button.
uint16_t remap(uint16_t buttons) {
  uint16_t b0 = buttons & (1<<0);
  uint16_t b2 = (buttons & (1<<2));
  buttons &= ~5;
  buttons |= (b2 >> 2) | (b0 << 2);
  return buttons;
}

static inline int smin(int x, int y) {return (x < y) ? x : y;}
static inline int smax(int x, int y) {return (x > y) ? x : y;}

void handle_mouse(volatile HID_device_state_t *hid_dev) {
  if (hid_dev->report_len > 2) {
    uint8_t buttons = hid_dev->report[0];
    if (buttons & 1) {
      Gamepad.press(NSButton_A);
    } else {
      Gamepad.release(NSButton_A);
    }
    if (buttons & 2) {
      Gamepad.press(NSButton_B);
    } else {
      Gamepad.release(NSButton_B);
    }
    int8_t x = (int8_t)hid_dev->report[1];
    int8_t y = (int8_t)hid_dev->report[2];
    if (x < 0)
      x = -X_SENSITIVITY[-x];
    else
      x = X_SENSITIVITY[x];
    if (y < 0)
      y = -Y_SENSITIVITY[-y];
    else
      y = Y_SENSITIVITY[y];
    hid_dev->xmin = smin(x, hid_dev->xmin);
    hid_dev->xmax = smax(x, hid_dev->xmax);
    hid_dev->ymin = smin(y, hid_dev->ymin);
    hid_dev->ymax = smax(y, hid_dev->ymax);
    uint8_t u8x = map(x, hid_dev->xmin, hid_dev->xmax, 0, 255);
    uint8_t u8y = map(y, hid_dev->ymin, hid_dev->ymax, 0, 255);
    DBG_printf("xmin %d x %d xmax %d u8x %d ymin %d y %d ymax %d u8y %d\r\n",
        hid_dev->xmin, x, hid_dev->xmax, u8x, hid_dev->ymin, y, hid_dev->ymax, u8y);
    if (hid_dev->dev_addr & 1) {
      Gamepad.leftXAxis(u8x);
      Gamepad.leftYAxis(u8y);
    } else {
      Gamepad.rightXAxis(u8x);
      Gamepad.rightYAxis(u8y);
    }
  }
  hid_dev->available = false;
}

void handle_timeout(volatile HID_device_state_t *hid_dev) {
  if ((millis() - hid_dev->last_millis) > 31) {
    // Center x,y if no HID report for 32 ms.
    if (hid_dev->dev_addr & 1) {
      Gamepad.leftXAxis(127);
      Gamepad.leftYAxis(127);
    } else {
      Gamepad.rightXAxis(127);
      Gamepad.rightYAxis(127);
    }
    hid_dev->last_millis = millis();
    Gamepad.loop();
  }
}

void handle_arcade_stick(volatile HID_device_state_t *hid_dev) {
  if (sizeof(Arcade_t) == hid_dev->report_len) {
    volatile Arcade_t *rpt = (volatile Arcade_t *)&hid_dev->report;
    static uint16_t old_buttons = 0;
    uint16_t buttons = old_buttons;
    if (hid_dev->dev_addr & 1) {
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
    old_buttons = buttons;
  }
}

void loop() {
  delay(1);
  for (size_t dev_addr = 0; dev_addr < MAX_HID_DEVICES; dev_addr++) {
    volatile HID_device_state_t *hid_dev = &HID_devices[dev_addr];
    if (hid_dev->connected) {
      if (hid_dev->available) {
        // Remote wakeup
        if ( TinyUSBDevice.suspended() ) {
          // Wake up host if we are in suspend mode
          // and REMOTE_WAKEUP feature is enabled by host
          TinyUSBDevice.remoteWakeup();
        }
        if (Gamepad.ready()) {
          switch (hid_dev->hid_type) {
            case HID_dev_mouse:
              handle_mouse(hid_dev);
              break;
            case HID_dev_arcade_stick:
              handle_arcade_stick(hid_dev);
              break;
            default:
              break;
          }
          Gamepad.loop();
        }
        hid_dev->available = false;
      } else {
        handle_timeout(hid_dev);
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
  if (dev_addr > 4) return;
  volatile HID_device_state_t *hid_dev = &HID_devices[dev_addr-1];
  memset((void *)hid_dev->report, 0, sizeof(hid_dev->report));
  hid_dev->report_len = 0;
  hid_dev->dev_addr = dev_addr;
  hid_dev->instance = instance;
  hid_dev->report_count = 0;
  hid_dev->available_count = 0;
  hid_dev->last_millis = 0;
  hid_dev->available = false;
  hid_dev->xmin = hid_dev->xmax = 0;
  hid_dev->ymin = hid_dev->ymax = 0;
  if ((vid == ARCADE_VID) && (pid == ARCADE_PID)) {
    DBG_println("arcade stick connected");
    hid_dev->connected = true;
    hid_dev->hid_type = HID_dev_arcade_stick;
    hid_dev->skip_report_id = false;
  } else {
    uint8_t const protocol_mode = tuh_hid_get_protocol(dev_addr, instance);
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    DBG_printf("protocol_mode=%d,itf_protocol=%d\r\n",
        protocol_mode, itf_protocol);
    const size_t REPORT_INFO_MAX = 8;
    tuh_hid_report_info_t report_info[REPORT_INFO_MAX];
    uint8_t report_num = tuh_hid_parse_report_descriptor(report_info,
        REPORT_INFO_MAX, desc_report, desc_len);
    Serial.printf("HID descriptor reports:%d\r\n", report_num);
    if ((report_num == 0) && (itf_protocol == 2)) {
      if (!hid_dev->connected) {
        // Mouse report
        DBG_println("mouse connected");
        hid_dev->connected = true;
        hid_dev->hid_type = HID_dev_mouse;
      }
    } else {
      for (size_t i = 0; i < report_num; i++) {
        DBG_printf("%d,%d,%d\r\n", report_info[i].report_id, report_info[i].usage,
            report_info[i].usage_page);
        hid_dev->skip_report_id = false;
        hid_dev->hid_type = HID_dev_unknown;
        if (!hid_dev->connected && (report_info[i].usage_page == 1) && (report_info[i].usage == 2)) {
          // Mouse report
          DBG_println("mouse connected");
          hid_dev->connected = true;
          hid_dev->hid_type = HID_dev_mouse;
          if (itf_protocol == HID_ITF_PROTOCOL_NONE)
            hid_dev->skip_report_id = report_info[i].report_id != 0;
          else if (protocol_mode == HID_PROTOCOL_BOOT)
            hid_dev->skip_report_id = false;
          else
            hid_dev->skip_report_id = report_info[i].report_id != 0;
          break;
        }
      }
    }
  }

  DBG_print("HID report descriptor: ");
  for (size_t i = 0; i < desc_len; i++) {
    DBG_print(desc_report[i], HEX);
    DBG_print(',');
  }
  DBG_println();
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  DBG_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  if (dev_addr > 4) return;
  volatile HID_device_state_t *hid_dev = &HID_devices[dev_addr-1];
  if ((hid_dev->dev_addr == dev_addr) && (hid_dev->instance == instance)) {
    if (hid_dev->connected) {
      hid_dev->connected = false;
      hid_dev->available = false;
      hid_dev->hid_type= HID_dev_unknown;
      DBG_printf("HID device (%d) disconnected\r\n", instance);
    }
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (dev_addr > 4) return;
  volatile HID_device_state_t *hid_dev = &HID_devices[dev_addr-1];
  if (hid_dev->connected && (hid_dev->dev_addr == dev_addr) && (hid_dev->instance == instance)) {
    memcpy((void *)hid_dev->report, report, min(sizeof(hid_dev->report), len));
    hid_dev->report_len = len;
    hid_dev->last_millis = millis();
    hid_dev->report_count++;
    hid_dev->available_count++;
    hid_dev->available = true;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}
