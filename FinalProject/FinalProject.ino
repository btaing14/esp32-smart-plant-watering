// Filename: FinalProject.ino
// Author: Bobby Taing
// Date: 08/21/2025
// Description: Smart plant watering system on ESP32-S3.
//              - Reads soil moisture (capacitive sensor, ADC).
//              - Drives relay (active-LOW) to run a water pump.
//              - Manual hold button to pump water.
//              - Suspend toggle button for the pump with LED indicator.
//              - Ultrasonic sensor detecting water depth with LED indicator.
//              - 16x2 I2C LCD UI: Top = moisture % + idle timer; Bottom = tank distance + cm + pump state.
//              - AUTO: when soil moisture reading is below the threshold, pump the water
//                      for ten seconds and wait for the water to seep into the soil for ninety seconds,
//                      then checks the reading again if more is needed.
//
// ========== Citations ==========
// https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/
// https://www.robotique.tech/robotics/control-a-water-pump-by-arduino/
// https://lastminuteengineers.com/i2c-lcd-arduino-tutorial/
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/memory-types.html
// https://docs.espressif.com/projects/esp-idf/en/v4.4.2/esp32/api-reference/system/freertos.html
// https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/esp_timer.html
// https://esp32tutorials.com/high-resolution-timer-esp-timer-esp-idf/
// https://esp32tutorials.com/esp32-esp-idf-freertos-timer-delay-esp-idf/
//
// ========== Includes ==========
#include <Wire.h>               // I2C for LCD
#include <LiquidCrystal_I2C.h>  // 16x2 I2C LCD
#include <string.h>             // memset, strcpy, strcat, strlen
#include "freertos/FreeRTOS.h"  // FreeRTOS core definitions
#include "freertos/task.h"      // FreeRTOS task management
#include "freertos/queue.h"     // FreeRTOS queues
#include "esp_timer.h"          // High-resolution timers

// ========== Macros ==========
#define SENSOR_PIN         14    // Soil moisture sensor
#define RELAY_PIN          16    // Relay IN (active-LOW) — LOW=ON, HIGH=OFF
#define BTN_MANUAL_PIN     12    // Manual water — hold to pump
#define BTN_SUSP_PIN       13    // Suspend/Resume toggle
#define ULTRA_TRIG_PIN      6    // Ultrasonic TRIG
#define ULTRA_ECHO_PIN      7    // Ultrasonic ECHO
#define WATER_LOW_PIN       1    // LED: ON when tank empty or suspended

#define DEBOUNCE_MS        15U   // Button debounce time (ms) (U for unsigned constant)

#define DRY_RAW_DEFAULT   3190   // ADC reading in air (fully dry)
#define WET_RAW_DEFAULT   1300   // ADC reading fully wet
#define MOISTURE_ON_THRESHOLD 30 // AUTO ON when < 30%; OFF when >= 30%

#define ECHO_TIMEOUT_US  15000UL // Max time to wait for the echo pulse to return (0.015s)
#define US_TO_CM (1.0f/58.0f)    // Conversion from microseconds to centimeters
#define EMPTY_DISTANCE_CM 10.0f  // Tank considered empty at >= 10 cm

// Auto watering
#define MIN_ON_MS        10000UL // Auto pumps ON for 10 seconds
#define SOAK_MS          90000UL // Auto pumps OFF for 90 seconds

// ========== LCD Setup ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);   // Address 0x27, 16 columns x 2 rows

// ========== Custom icons for LCD ==========
byte ICON_DROP[8]  = {0b00100,0b00100,0b01110,0b11111,0b11111,0b01110,0b00100,0b00000}; // moisture
byte ICON_CLOCK[8] = {0b01110,0b10001,0b10101,0b10111,0b10001,0b10001,0b01110,0b00000}; // clock
byte ICON_TANK[8]  = {0b11111,0b10001,0b10101,0b10101,0b10101,0b10001,0b11111,0b00000}; // tank
byte ICON_PUMP[8]  = {0b00100,0b01110,0b10101,0b10101,0b10101,0b01110,0b00100,0b00000}; // pump

// ========== Data Structures ==========
typedef struct {
  int      moisturePct;      // Soil moisture percentage (0..100)
  uint8_t  pumpOn;           // Pump state (1=ON, 0=OFF)
  uint32_t idleSeconds;      // Seconds since pump last turned OFF
  uint8_t  suspended;        // 1 = suspended (user or auto)
  int      distanceX100;     // Tank distance in cm * 100 (two decimals)
  uint8_t  tankEmpty;        // 1 = tank empty (>= EMPTY_DISTANCE_CM)
} status_t;

// ========== Globals ==========
static int DRY_RAW = DRY_RAW_DEFAULT; // Calibration (dry)
static int WET_RAW = WET_RAW_DEFAULT; // Calibration (wet)

static TaskHandle_t PumpTaskHandle = NULL;
static TaskHandle_t LCDTaskHandle = NULL;

static QueueHandle_t qStatus   = NULL; // Pump -> LCD snapshot
static QueueHandle_t qCtrlTick = NULL; // 50 Hz control tick
static QueueHandle_t qUITick   = NULL; // 64 Hz UI tick

static esp_timer_handle_t ctrlTimer = NULL; // 50 Hz
static esp_timer_handle_t uiTimer   = NULL; // 64 Hz

// ========== Function Prototypes ==========
static int   moisturePercentFromRaw(int raw, int dryRaw, int wetRaw);
static float pingUltrasonicCM(void);
static void  IRAM_ATTR ctrlTickCb(void *arg);
static void  IRAM_ATTR uiTickCb(void *arg);
void         TaskPump(void *pv);
void         TaskLCD(void *pv);

// ========== Helper Implementations ==========

// Name: moisturePercentFromRaw
// Description: Maps raw ADC reading to 0..100% given DRY/WET calibration.
static int moisturePercentFromRaw(int raw, int dryRaw, int wetRaw) {
  // If calibration values are equal return 0% just to be safe
  if (dryRaw == wetRaw) return 0; 

  float pct;
  // If the reading is more dry or wet than the other
  // Calculate the percent for moisture
  if (dryRaw > wetRaw) {
    pct = ((float)(dryRaw - raw) / (float)(dryRaw - wetRaw)) * 100.0f;
  } else {
    pct = ((float)(raw - dryRaw) / (float)(wetRaw - dryRaw)) * 100.0f;
  }

  // Ensures noise or out-of-range values do not go below 0% or above 100%
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;

  // Convers float to int, rouning to nearest whole percent
  return (int)(pct + 0.5f);
}

// Name: pingUltrasonicCM
// Description: Sends one HC-SR04-style ping and returns distance in cm.
static float pingUltrasonicCM(void) {
  // Sequence:
  // LOW to reset
  // Sending 10 microseconds HIGH pulse to the TRIG pin to trigger the sensor
  // Back to low to end pulse
  digitalWrite(ULTRA_TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  // pulseIn() waits for the ECHO pin to go HIGH, then times how long it stays HIGH
  // The duration is the round-trip travel time of the ultrasonic sound wave
  unsigned long dur = pulseIn(ULTRA_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  // If no echo is received before timeout, return a sentinel value (invalid)
  if (dur == 0) return -1.0f;

  // Convert time to distance
  return dur * US_TO_CM;
}

// Name: ctrlTickCb
// Description: Callback is trigged by the timer at 50 Hz
//              and signals TaskPump via a queue,
//              Think of it like a timer alarm bell where it rings
//              and drops a note into the queue, the task listens for
//              the notes and knows when it is time to do a control cycle.
//              For the pump, this is to ensure that the system is fast enough
//              to check soil moisture and ultrasonic regularly, reacting quickly
//              when soil is dry. Every 20ms, the system decides if the pump should be ON/OFF.
static void IRAM_ATTR ctrlTickCb(void *arg) {
  uint8_t b = 1;
  // Ensure the queue was created successfully
  if (qCtrlTick != NULL) {
    // Pushes the byte into the control tick queue
    // xQueueSendFromISR to put data into queue but in a safe way from an interrupt
    // This wakes up TaskPump, which is waiting on qCtrlTick
    xQueueSendFromISR(qCtrlTick, &b, NULL);
  }
}

// Name: uiTickCb
// Description: 64 Hz timer callback -> posts a byte into qUITick.
//              Same style as the TaskPump but for LCDTask and faster.
//              Keeps the LCD feeling smooth, without visible lag.
static void IRAM_ATTR uiTickCb(void *arg) {
  uint8_t b = 1;
  if (qUITick != NULL) {
    xQueueSendFromISR(qUITick, &b, NULL);
  }
}

// ========== Task Implementations ==========

// Name: TaskPump
// Description: Reads sensor/button states, controls relay + LED, AUTO watering,
//              and publishes a snapshot to the LCD task via qStatus.
// Notes:
//   - Priority over pump: SUSPEND (OFF) -> MANUAL (ON) -> AUTO (ON/OFF).
//   - Again, when AUTO starts, pump remains ON for at least 10 seconds, then waits for 90 seconds before checking again.
//   - Idle timer starts once water pump finishes executing to show when it was lasted watered.
void TaskPump(void *pv) {
  (void)pv;

  // Attenuation = how much the input voltage is "scaled down" before conversion for soil moisture sensor pin
  // ADC_11db - input voltage range 0.0 - 3.6V
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);

  // Button debounce state
  int lastManual = HIGH;     // last read of manual btn
  int manualStable = HIGH;   // debounced manual
  uint32_t manualAt = 0;     // last change time

  int lastSuspend = HIGH;    // last read of suspend btn
  int suspendStable = HIGH;  // debounced suspend
  uint32_t suspendAt = 0;    // last change time

  // Suspend flags
  uint8_t suspendedButton = 0; // user toggle
  uint8_t suspendedAuto   = 0; // tank empty

  // Pump + idle timer
  uint8_t pumpOn = 0;
  uint8_t lastPumpOn = 0;
  uint8_t first = 1;
  TickType_t offStart = 0;

  // AUTO
  uint8_t  inAutoBurst   = 0;
  uint32_t autoOnUntilMs = 0;
  uint32_t soakUntilMs   = 0;

  // Reservoir distance tracker
  float lastDistCM = -1.0f;

  // Snapshot to LCD
  status_t s;                 // struct
  memset(&s, 0, sizeof(s));   // fills all fields of the struct with 0
  s.distanceX100 = -1;        // marks distance as invalid

  uint8_t tick;
  TickType_t fastScanDelay = pdMS_TO_TICKS(5); // ~200 Hz button scan

  while (1) {
    // Current time in milliseconds
    uint32_t nowms = millis();

    // Debounce: Manual (hold)
    int mRead = digitalRead(BTN_MANUAL_PIN);  // sample button
    if (mRead != lastManual) {                // state changes?
      manualAt = nowms;                       // start/restart the debounce timer
    }
    if ((nowms - manualAt) >= DEBOUNCE_MS) {  // state stayed unchanged long enough?
      if (manualStable != mRead) {            // is it a new stable value?
        manualStable = mRead;                 // accepts as the debounced state
      }
    }
    lastManual = mRead;                       // remember for next iteration

    // Debounce: Suspend (toggle on press)
    int sRead = digitalRead(BTN_SUSP_PIN);
    if (sRead != lastSuspend) {
      suspendAt = nowms;
    }
    if ((nowms - suspendAt) >= DEBOUNCE_MS) {
      if (suspendStable != sRead) {
        // falling edge (pressed = LOW) toggles suspend
        if (suspendStable == HIGH) {  // previously HIGH (not pressed)
          if (sRead == LOW) {         // now LOW (pressed)
            // If it was 0 (auto active), it becomes 1 (auto suspended)
            if (suspendedButton == 0) suspendedButton = 1;
            // If it was 1 (auto suspended), it becomes 0 (auto active)
            else suspendedButton = 0;
          }
        }
        suspendStable = sRead;
      }
    }
    lastSuspend = sRead;

    // 50 Hz control tick
    // Only execute the control loop when the timer ISR drops a byto into qCtrlTick
    if (xQueueReceive(qCtrlTick, &tick, fastScanDelay) == pdTRUE) {
      // Read soil moisture and turn into a %
      int raw = analogRead(SENSOR_PIN);
      if (raw > DRY_RAW) raw = DRY_RAW;
      if (raw < WET_RAW) raw = WET_RAW;
      int moisturePct = moisturePercentFromRaw(raw, DRY_RAW, WET_RAW);

      // Read ultrasonic tank distance and keep last value
      float dcm = pingUltrasonicCM();
      if (dcm > 0.0f) {
        lastDistCM = dcm;
      }

      // Detect if tank is empty and set auto-suspend
      uint8_t tankEmpty = 0;
      if (lastDistCM > 0.0f) {
        if (lastDistCM >= EMPTY_DISTANCE_CM) {
          tankEmpty = 1;
        }
      }
      suspendedAuto = tankEmpty;

      // Drive the status LED whether the button is pressed or tank is empty
      if (tankEmpty == 1 || suspendedButton == 1) {
        digitalWrite(WATER_LOW_PIN, HIGH);
      } else {
        digitalWrite(WATER_LOW_PIN, LOW);
      }

      // Combined suspend sources
      // Single flag that says whether any suspension is active
      uint8_t suspended = 0;
      if (suspendedButton == 1) suspended = 1;  // user toggle
      if (suspendedAuto == 1)  suspended = 1;   // tank empty

      // -Decide pump state with priority: Suspend -> Manual -> Auto
      uint8_t prev = pumpOn;

      if (suspended == 1) {
        // Force OFF and cancel any AUTO
        pumpOn = 0;
        inAutoBurst = 0;
        digitalWrite(RELAY_PIN, HIGH);
      } else if (manualStable == LOW) {
        // Manual hold: force ON (cancel AUTO timing)
        pumpOn = 1;
        inAutoBurst = 0;
        digitalWrite(RELAY_PIN, LOW);
      } else {
        // AUTO section
        if (inAutoBurst == 1) {
          // Enforce minimum ON duration regardless of threshold
          if (nowms < autoOnUntilMs) {
            pumpOn = 1;
            digitalWrite(RELAY_PIN, LOW);
          } else {
            // Finish -> OFF until 90 seconds elasped from current time
            inAutoBurst = 0;
            pumpOn = 0;
            digitalWrite(RELAY_PIN, HIGH);
            soakUntilMs = nowms + SOAK_MS;
          }
        }

        // If the state is not in AUTO
        if (inAutoBurst == 0) {
          // Pump stays OFF
          uint8_t autoAllowed = 1;
          if (nowms < soakUntilMs) autoAllowed = 0;

          // READY state
          if (autoAllowed == 1) {
            // Pump ON
            if (moisturePct < MOISTURE_ON_THRESHOLD) {
              inAutoBurst   = 1;
              autoOnUntilMs = nowms + MIN_ON_MS; 
              pumpOn = 1;
              digitalWrite(RELAY_PIN, LOW);
            } else {
              // READY but not dry enough
              pumpOn = 0;
              digitalWrite(RELAY_PIN, HIGH);
            }
          } else {
            // Force pump OFF regardless of moisture
            pumpOn = 0;
            digitalWrite(RELAY_PIN, HIGH);
          }
        }
      }

      // Idle timer logic (seconds since last watering OFF)
      // Record the current pump state
      // If pump starts OFF, record the tick count as the beginning of idle period
      if (first == 1) {
        first = 0;
        lastPumpOn = pumpOn;
        if (pumpOn == 0) {
          offStart = xTaskGetTickCount();
        }
      }

      // Detect OFF to ON transition (pump just started)
      // Reset idle counter when it starts watering
      if (prev == 0) {
        if (pumpOn == 1) {
          // OFF -> ON
          s.idleSeconds = 0;
        }
      }

      // Detects ON to OFF transition (watering finished)
      // Reset idle counter again and mark this time as the new offStart
      if (prev == 1) {
        if (pumpOn == 0) {
          // ON -> OFF
          offStart = xTaskGetTickCount();
          s.idleSeconds = 0;
        }
      }

      // While pump is OFF: compute elasped time since it lasted turned off
      // While it is ON: report 0
      // Update lastPumpOn for the next cycle's transition check
      if (pumpOn == 0) {
        TickType_t nowt = xTaskGetTickCount();
        // Convert ticks to ms to seconds
        uint32_t ms = (uint32_t)((nowt - offStart) * portTICK_PERIOD_MS);
        s.idleSeconds = ms / 1000U;
      } else {
        s.idleSeconds = 0;
      }
      lastPumpOn = pumpOn;

      // Record soil moisture, pump state, and suspend flag
      s.moisturePct = moisturePct;
      s.pumpOn      = pumpOn;
      s.suspended   = suspended;

      // Store tank distance
      // If no valid reading, set 01
      // Flag if tank is empty
      if (lastDistCM > 0.0f) {
        int x100 = (int)(lastDistCM * 100.0f + 0.5f);
        s.distanceX100 = x100;
      } else {
        s.distanceX100 = -1;
      }
      s.tankEmpty = tankEmpty;

      // Push this fresh snapshot to qStatus
      if (qStatus != NULL) {
        // Ensure the LCD task always has the latest status
        xQueueOverwrite(qStatus, &s);
      }
    }
  }
}

// Name: TaskLCD
// Description: display task pinned to the other core
//              - initializes the LCD and custom icons
//              - waits for a 64 Hz UI tick
//              - pulls the latest snapshot from the queue
//              - only redraws parts of the screen that actually changed
// Note: Top row: moisture (%) and timer (--:--:--); Bottom row: tank (--:--cm) and pump (ON/OFF)
void TaskLCD(void *pv) {
  (void)pv;

  // Initialize LCD + icons
  lcd.clear();
  lcd.createChar(0, ICON_DROP);
  lcd.createChar(1, ICON_CLOCK);
  lcd.createChar(2, ICON_TANK);
  lcd.createChar(3, ICON_PUMP);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Cached state for redraw minimization avoiding constant rewrite the same characters
  status_t s;                 // struct
  memset(&s, 0, sizeof(s));   // initialize all fields to 0
  s.moisturePct = -1;
  s.distanceX100 = -1;

  // These hold the last values actually drawn to the LCD
  // Initialize to "impossible" values to guarntee the very first update forces a redraw
  int      lastPct  = -1;
  uint32_t lastSecs = 0xFFFFFFFF;
  int      lastDist = -999999;
  uint8_t  lastPumpOn = 0xFF;

  // Clear bottom once
  lcd.setCursor(0, 1);
  lcd.print("                ");

  uint8_t tick;
  while (1) {
    // wait for UI tick
    if (xQueueReceive(qUITick, &tick, portMAX_DELAY) == pdTRUE) {
      // pull latest snapshot (non-blocking)
      // reading the latest system state from qStatus
      status_t tmp;
      if (qStatus != NULL) {
        if (xQueueReceive(qStatus, &tmp, 0) == pdTRUE) {
          s = tmp;
        }
      }

      // TOP ROW
      // Only redraw if moisture or timer changed
      if (s.moisturePct != lastPct || s.idleSeconds != lastSecs) {
        lcd.setCursor(0, 0);
        lcd.print("                ");

        // Ensure % stays in 0-100 range
        int pct = s.moisturePct;
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;

        // Compute hrs:mins:secs
        uint32_t secs = s.idleSeconds;
        uint32_t hh = secs / 3600U;
        uint32_t mm = (secs / 60U) % 60U;
        uint32_t ss = secs % 60U;

        // Compose the row with icons and spacing
        // column 0: draw custom char ICON_DROP at index 0
        // column 1: space
        // Cursor col is incremented each time so the next write is placed correctly
        int col = 0;
        lcd.setCursor(col, 0); lcd.write((uint8_t)0); col++;   // drop
        lcd.setCursor(col, 0); lcd.print(' '); col++;

        // Print moisture percent
        // Format the number into string
        char pctbuf[4];
        if (pct == 100) {
          strcpy(pctbuf, "100");
        } else {
          snprintf(pctbuf, sizeof(pctbuf), "%d", pct);
        }
        lcd.setCursor(col, 0); lcd.print(pctbuf); col += (int)strlen(pctbuf);

        // Add percent sign, space, clock icon
        lcd.setCursor(col, 0); lcd.print('%'); col++;
        lcd.setCursor(col, 0); lcd.print(' '); col++;

        lcd.setCursor(col, 0); lcd.write((uint8_t)1); col++;   // clock
        lcd.setCursor(col, 0); lcd.print(' '); col++;

        // Print timer
        char timebuf[9];
        snprintf(timebuf, sizeof(timebuf), "%02u:%02u:%02u",
                 (unsigned)hh, (unsigned)mm, (unsigned)ss);

        // special layout case for 100% to prevent overflow
        if (pct == 100) {
          col = 8; // keep within 16 cols when 3-digit percent
        }
        lcd.setCursor(col, 0); lcd.print(timebuf);

        // Update "last drawn" cache
        lastPct  = s.moisturePct;
        lastSecs = s.idleSeconds;
      }

      // Bottom Row
      // Only redraw when distance of pump state changes
      if (s.distanceX100 != lastDist || s.pumpOn != lastPumpOn) {
        // Format the distance string
        char distStr[7]; // 6 chars + NUL
        
        // Valid distance
        // Split into integer part and two digit fractional part
        if (s.distanceX100 >= 0) {
          int whole = s.distanceX100 / 100; // integer cm
          int frac  = s.distanceX100 % 100; // hundredths

          // Show a clmap marker if it exceeds 6 chars
          // Otherwise, format right-align
          if (whole > 999) {
            strcpy(distStr, "###.##");  // clamp
          } else {
            // Build the raw distance text
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "%d.%02d", whole, frac);

            // Measure its length
            int len = (int)strlen(tmp);

            // Compute how much padding is needed
            // Ex. "1.23" = length 4 = pad 2
            //     "12.34" = length 5 = pad 1
            int pad = 6 - len;
            if (pad < 0) pad = 0;

            // Fill in left padding
            // Ex. pad = 2 -> " "
            //     pad = 1 -> " "
            //     pad = 0 -> ""
            int i = 0;
            while (i < pad) { distStr[i] = ' '; i++; }
            distStr[i] = '\0';

            // Append the actual number
            // Ex. pad = 2 "1.23" -> " 1.23"
            //     pad = 1 "12.34" -> " 12.34"
            //     pad = 0 "123.45" -> "123.45"
            strcat(distStr, tmp);
          }
        } else {
          // If there is no valid reading, indicate no data
          strcpy(distStr, " --.--");
        }

        // Clear row and lay out fixed columns
        lcd.setCursor(0, 1); lcd.print("                ");
        lcd.setCursor(0, 1); lcd.write((uint8_t)2);        // [tank]
        lcd.setCursor(1, 1); lcd.print(' ');
        lcd.setCursor(2, 1); lcd.print(distStr);           // DD.DD
        lcd.setCursor(8, 1); lcd.print('c');
        lcd.setCursor(9, 1); lcd.print('m');
        lcd.setCursor(10,1); lcd.print(' ');
        lcd.setCursor(11,1); lcd.write((uint8_t)3);        // [pump]
        lcd.setCursor(12,1); lcd.print(' ');
        lcd.setCursor(13,1);                               // ON/OFF
        if (s.pumpOn == 1) {
          lcd.print("ON ");
        } else {
          lcd.print("OFF");
        }

        // Update "last drawn" cache
        lastDist   = s.distanceX100;
        lastPumpOn = s.pumpOn;
      }
    }
  }
}

// ========== Setup / Loop ==========

// Name: setup
// Description: Initializes pins, I2C/LCD, queues, timers, and creates tasks.
void setup() {
  // Relay default OFF (active-LOW)
  digitalWrite(RELAY_PIN, HIGH);
  pinMode(RELAY_PIN, OUTPUT);

  // Buttons
  pinMode(BTN_MANUAL_PIN, INPUT_PULLUP);
  pinMode(BTN_SUSP_PIN,   INPUT_PULLUP);

  // Ultrasonic
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  // LED
  pinMode(WATER_LOW_PIN, OUTPUT);
  digitalWrite(WATER_LOW_PIN, LOW);

  // I2C + LCD
  Wire.begin();         // ESP32-S3 default: SDA=8, SCL=9
  lcd.init();
  lcd.backlight();

  // Queues
  qStatus   = xQueueCreate(1, sizeof(status_t));
  qCtrlTick = xQueueCreate(8, sizeof(uint8_t));
  qUITick   = xQueueCreate(8, sizeof(uint8_t));

  // Timers: control @ 50 Hz
  // Configure an ESP-IDF timer that calls ctrlTickCb()
  // ctrlTickCb posts a byte into qCtrlTick
  // This drives TaskPump's control loop at 50 Hz
  esp_timer_create_args_t ctrlArgs;
  memset(&ctrlArgs, 0, sizeof(ctrlArgs));
  ctrlArgs.callback = &ctrlTickCb;
  ctrlArgs.dispatch_method = ESP_TIMER_TASK;
  ctrlArgs.name = "ctrl50Hz";
  esp_timer_create(&ctrlArgs, &ctrlTimer);
  esp_timer_start_periodic(ctrlTimer, 20000); // 20,000 microseconds

  // Timers: UI @ 64 Hz
  // Same setup as the previous, but for uiTickCb()
  // Post bytes into qUITick
  // This paces the LCD redraws
  esp_timer_create_args_t uiArgs;
  memset(&uiArgs, 0, sizeof(uiArgs));
  uiArgs.callback = &uiTickCb;
  uiArgs.dispatch_method = ESP_TIMER_TASK;
  uiArgs.name = "ui64Hz";
  esp_timer_create(&uiArgs, &uiTimer);
  esp_timer_start_periodic(uiTimer, 15625);   // 15,625 microseconds

  // Tasks (pin to cores)
  xTaskCreatePinnedToCore(TaskPump, "TaskPump", 6144, NULL, 2, &PumpTaskHandle, 1); // Core 1
  xTaskCreatePinnedToCore(TaskLCD,  "TaskLCD",  2048, NULL, 1, &LCDTaskHandle, 0); // Core 0
}

// Name: loop
// Description: Unused (FreeRTOS + timers drive the app).
void loop() {}