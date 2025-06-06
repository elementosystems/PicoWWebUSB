/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/* This example demonstrates WebUSB as web serial with browser with WebUSB support (e.g Chrome).
 * After enumerated successfully, browser will pop-up notification
 * with URL to landing page, click on it to test
 *  - Click "Connect" and select device, When connected the on-board LED will litted up.
 *  - Any charters received from either webusb/Serial will be echo back to webusb and Serial
 *
 * Note:
 * - The WebUSB landing page notification is currently disabled in Chrome
 * on Windows due to Chromium issue 656702 (https://crbug.com/656702). You have to
 * go to landing page (below) to test
 *
 * - On Windows 7 and prior: You need to use Zadig tool to manually bind the
 * WebUSB interface with the WinUSB driver for Chrome to access. From windows 8 and 10, this
 * is done automatically by firmware.
 *
 * - On Linux/macOS, udev permission may need to be updated by
 *   - copying '/examples/device/99-tinyusb.rules' file to /etc/udev/rules.d/ then
 *   - run 'sudo udevadm control --reload-rules && sudo udevadm trigger'
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "hardware/adc.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "ssd1306.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED     = 1000,
  BLINK_SUSPENDED   = 2500,

  BLINK_ALWAYS_ON   = UINT32_MAX,
  BLINK_ALWAYS_OFF  = 0
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

#define URL  "powerflashutility.bsadashi.work"


#define ACS712_PIN 26          // ADC pin for ACS712
#define VLTG_DVDR_PIN 27       // ADC pin for voltage divider
#define ACS712_VCC 5.0          // ACS712 VCC is now 5V
#define IDEAL_MID_POINT_VOLTAGE (ACS712_VCC / 2.0) // Ideal midpoint is now 2.5V
#define ADC_INTERVAL_MS 1000    // Now 1000ms for 1 second interval
#define ADC_RANGE 4095
#define ADC_REF_VOLTAGE 3.3f

#define OS_DELAY_10MS 10
#define OS_DELAY_1MS 1
#define OS_DELAY_100MS 100

#define CHANNEL_0 0   // Your existing ADC0 operation
#define CHANNEL_1 1   // 12V measurement

// Voltage divider resistor values
#define R1 30000.0f // 30k ohms
#define R2 7500.0f  // 7.5k ohms

// OLED parameters
#define OLED_WIDTH 64
#define OLED_HEIGHT 48
#define OLED_ADDRESS 0x3C // Most common address, adjust if needed
#define OLED_SDA_PIN 16    // Change these to your wiring
#define OLED_SCL_PIN 17
i2c_inst_t *i2c = i2c0;    // Change to i2c1 if needed

// Global display object
ssd1306_t disp;


const tusb_desc_webusb_url_t desc_url = {
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};

static bool web_serial_connected = false;

// GPIO Pins and Pin Mapping
const uint GPIO_PINS[] = {11, 12, 13, 14};
const int NUM_GPIO_PINS = sizeof(GPIO_PINS) / sizeof(GPIO_PINS[0]);

// Command definitions
enum {
  CMD_GPIO_11 = 0x0B,
  CMD_GPIO_12 = 0x0C,
  CMD_GPIO_13 = 0x0D,
  CMD_GPIO_14 = 0x0E,
  CMD_ON = 0x01,
  CMD_OFF = 0x00
};

float measured_midpoint_voltage;
float mV_PER_AMP = 0.185; 

//------------- prototypes -------------//
void led_blinking_task(void);
void cdc_task(void);
void gpio_init_all(); // Function prototype for gpio_init_all
void process_usb_commands(const uint8_t* buffer, uint32_t count);
float read_current(uint8_t channel);
void adc_task(void);
float measure_midpoint(void);
float read_voltage(uint8_t channel);
void setup_adc();
void update_oled(float current, float voltage);

void ostask_cont_c0(void *pvParameters) {
    while (1) {
    tud_task(); // tinyusb device task
    cdc_task();
    led_blinking_task();
    // led_blinking_task();
    // adc_task();
       // tinyusb device task
    //vTaskDelay(pdMS_TO_TICKS(OS_DELAY_10MS)); // Delay for 10ms
    }
}

void ostask_10ms_c1(void *pvParameters) {
    while (1) {
    adc_task();
       // tinyusb device task
    vTaskDelay(pdMS_TO_TICKS(OS_DELAY_10MS)); // Delay for 10ms
    }
}

/*------------- MAIN -------------*/
int main(void) {
  TaskHandle_t xHandle0; 
  TaskHandle_t xHandle1; 
  board_init();
  gpio_init_all();
  i2c_init(i2c, 100 * 1000); // 100 kHz
  gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(OLED_SDA_PIN);
  gpio_pull_up(OLED_SCL_PIN);
  setup_adc();

  // init device stack on configured roothub port
  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed");
    return -1;
  }
      // Initialize OLED
  if (!ssd1306_init(&disp, OLED_WIDTH, OLED_HEIGHT, OLED_ADDRESS, i2c)) {
    printf("OLED initialization failed!\n");
  }
    ssd1306_clear(&disp);
    ssd1306_show(&disp); // Clear the display initially

measured_midpoint_voltage = measure_midpoint();
printf("Measured Midpoint: %.3f V\n", measured_midpoint_voltage);
//We calculate the new mV per AMP
mV_PER_AMP = (measured_midpoint_voltage- IDEAL_MID_POINT_VOLTAGE) /0.0391708 ;
printf("New mv per amp = %.3f", mV_PER_AMP);

xTaskCreate(
    ostask_cont_c0,  // Task function
    "CONTINUOUS_TASK",    // Name
    4096,              // Stack size
    NULL,             // Parameters
    255,                // Priority
    &xHandle0         // Task handle
);

xTaskCreate(
    ostask_10ms_c1,  // Task function
    "TEN_MS_TASK",    // Name
    4096,              // Stack size
    NULL,             // Parameters
    10,                // Priority
    &xHandle1         // Task handle
);

vTaskCoreAffinitySet(xHandle0, (1 << 0)); // Core 0
vTaskCoreAffinitySet(xHandle1, (1 << 1)); // Core 1

  // Start FreeRTOS scheduler
vTaskStartScheduler();
  while (1) {

  }
}

// send characters to both CDC and WebUSB
void echo_all(const uint8_t buf[], uint32_t count) {
  // echo to web serial
  if (web_serial_connected) {
    tud_vendor_write(buf, count);
    tud_vendor_write_flush();
  }

  // echo to cdc
  if (tud_cdc_connected()) {
    for (uint32_t i = 0; i < count; i++) {
      tud_cdc_write_char(buf[i]);
      if (buf[i] == '\r') {
        tud_cdc_write_char('\n');
      }
    }
    tud_cdc_write_flush();
  }
}

// --------------------------------------------------------------------+
// Device callbacks
// --------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

// --------------------------------------------------------------------+
// WebUSB use vendor class
// --------------------------------------------------------------------+

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
  // nothing to with DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
      switch (request->bRequest) {
        case VENDOR_REQUEST_WEBUSB:
          // match vendor request in BOS descriptor
          // Get landing page url
          return tud_control_xfer(rhport, request, (void*)(uintptr_t)&desc_url, desc_url.bLength);

        case VENDOR_REQUEST_MICROSOFT:
          if (request->wIndex == 7) {
            // Get Microsoft OS 2.0 compatible descriptor
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20 + 8, 2);

            return tud_control_xfer(rhport, request, (void*)(uintptr_t)desc_ms_os_20, total_len);
          } else {
            return false;
          }

        default: break;
      }
      break;

    case TUSB_REQ_TYPE_CLASS:
      if (request->bRequest == 0x22) {
        // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to connect and disconnect.
        web_serial_connected = (request->wValue != 0);

        // Always lit LED if connected
        if (web_serial_connected) {
          board_led_write(true);
          blink_interval_ms = BLINK_ALWAYS_ON;

          tud_vendor_write_str("\r\nWebUSB interface connected\r\n");
          tud_vendor_write_flush();
        } else {
          blink_interval_ms = BLINK_MOUNTED;
        }

        // response with status OK
        return tud_control_status(rhport, request);
      }
      break;

    default: break;
  }

  // stall unknown request
  return false;
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const* buffer, uint16_t bufsize) {
  (void) itf;

  process_usb_commands(buffer, bufsize);
  echo_all(buffer, bufsize);

  // if using RX buffered is enabled, we need to flush the buffer to make room for new data
  #if CFG_TUD_VENDOR_RX_BUFSIZE > 0
  tud_vendor_read_flush();
  #endif
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void) {
  if (tud_cdc_connected()) {
    // connected and there are data available
    if (tud_cdc_available()) {
      uint8_t buf[64];

      uint32_t count = tud_cdc_read(buf, sizeof(buf));

      // echo back to both web serial and cdc
      echo_all(buf, count);
    }
  }
}

//Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void)itf;

  // connected
  if (dtr && rts) {
    // print initial message when connected
    tud_cdc_write_str("\r\nConnect to the PowerFlash Utility\r\n");
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
  (void)itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, (int)led_state);
  led_state = 1 - led_state; // toggle

}

void setup_adc() {
  adc_init();
  adc_gpio_init(ACS712_PIN);  // ADC0 (GPIO26)
  adc_gpio_init(VLTG_DVDR_PIN);  // ADC1 (GPIO27)
  adc_set_round_robin(0);  // Disable round-robin mode
}

void gpio_init_all() {
  for (int i = 0; i < NUM_GPIO_PINS; i++) {
    gpio_init(GPIO_PINS[i]);
    gpio_set_dir(GPIO_PINS[i], GPIO_OUT);
    gpio_put(GPIO_PINS[i], 0); // Initialize to OFF
  }
}

void process_usb_commands(const uint8_t* buffer, uint32_t count) {
  // We expect 2 byte commands: [GPIO_PIN_SELECTOR, ON/OFF_STATE]
  if (count >= 2) {
    uint8_t pin_selector = buffer[0];
    uint8_t on_off_state = buffer[1];

    int pin_index = -1;

    switch (pin_selector) {
        case CMD_GPIO_11:
            pin_index = 0;
            break;
        case CMD_GPIO_12:
            pin_index = 1;
            break;
        case CMD_GPIO_13:
            pin_index = 2;
            break;
        case CMD_GPIO_14:
            pin_index = 3;
            break;
        default:
            // Unknown pin selector
            break;
    }

    if (pin_index >= 0) {
        if (on_off_state == CMD_ON) {
          gpio_put(GPIO_PINS[pin_index], 1);
        } else if (on_off_state == CMD_OFF) {
          gpio_put(GPIO_PINS[pin_index], 0);
        } else {
          // Unknown on/off state
        }
    }
    // Process any remaining data
      if (count>2){
        process_usb_commands(buffer + 2, count - 2);
      }
  }
}

float measure_midpoint(void) {
  const int num_samples = 100;
  float total_voltage = 0.0;
  for (int i = 0; i < num_samples; i++) {
    uint16_t adc_value = adc_read();
    float voltage = (adc_value * ADC_REF_VOLTAGE ) / ADC_RANGE; // Convert ADC reading to voltage
    total_voltage += voltage;
    sleep_ms(1); // Small delay between samples
  }
  return total_voltage / num_samples;
}

float read_current(uint8_t channel) {
  adc_select_input(channel);
  uint16_t adc_value = adc_read();
  float voltage = (adc_value * ADC_REF_VOLTAGE) / ADC_RANGE; // Convert ADC reading to voltage
  float current = (voltage - measured_midpoint_voltage) / mV_PER_AMP;
  return current;
}

void adc_task(void) {
  static uint32_t last_adc_read_ms = 0; // Use uint32_t for accurate millisecond tracking

  // Check if 1 second (1000ms) has passed since the last ADC reading
  if (board_millis() - last_adc_read_ms >= ADC_INTERVAL_MS) {
    last_adc_read_ms = board_millis(); // Update the last reading time

    float current = read_current(CHANNEL_0);
    float voltage = read_voltage(CHANNEL_1);
    char current_str[32];
    char voltage_str[32];
    snprintf(current_str, sizeof(current_str), "Current: %.2f A\r\n", current);
    snprintf(voltage_str, sizeof(voltage_str), "Voltage: %.2f V\r\n", voltage);

    // Combined message with both strings
    uint32_t len_current = strlen(current_str);
    uint32_t len_voltage = strlen(voltage_str);
    uint32_t len_total = len_current + len_voltage;

    uint8_t current_message[len_total + 1];
    memcpy(current_message, current_str, len_current);
    memcpy(current_message + len_current, voltage_str, len_voltage);
    current_message[len_total] = '\0';

    echo_all(current_message, len_total);
    // Update OLED with current values
    update_oled(current, voltage);
  }
}

float read_voltage(uint8_t channel) {
  adc_select_input(channel);
  sleep_us(10);  // Allow ADC to stabilize
  uint16_t raw = adc_read();
  float measured = (raw * ADC_REF_VOLTAGE) / ADC_RANGE;
  
  // // Subtract an offset to account for ADC baseline (adjust as measured)
  // const float baseline_offset = 0.026f; // ~26 mV offset from ADC (tweak if needed)
  // if (measured > baseline_offset) {
  //     measured -= baseline_offset;
  // } else {
  //     measured = 0;
  // }
  
  // Apply voltage divider correction: Vin = V_measured * ((R1 + R2) / R2)
  return measured * ((R1 + R2)/R2);
}

void update_oled(float current, float voltage) {
  char current_str[32];
  char voltage_str[32];

  ssd1306_clear(&disp); // Clear the buffer

  // Format and draw the voltage and current strings
  snprintf(current_str, sizeof(current_str), "%.1f A", current);
  snprintf(voltage_str, sizeof(voltage_str), "%.1f V", voltage);

  ssd1306_draw_string(&disp, 0, 0, 1.6, voltage_str);
  ssd1306_draw_string(&disp, 0, 16, 1.6, current_str);

  ssd1306_show(&disp); // Update the physical display
}

//--------------------------------------------------------------------+
// FreeRTOS Application Hooks
//--------------------------------------------------------------------+
void vApplicationPassiveIdleHook( void )
{
  // This hook is called when a core other than core 0 is idle.
  // You can put low power management code here, for example.
  // For now, we'll leave it empty.
}