#include <Arduino.h>
#include <driver/timer.h>

// Define constants
const int PPM_CHANNEL_COUNT = 6;
const int PPM_SYNC_PULSE_WIDTH = 3000; // Synchronization pulse width in microseconds
const int PPM_PULSE_WIDTH_MIN = 1000; // Minimum PWM pulse width in microseconds
const int PPM_PULSE_WIDTH_MAX = 2000; // Maximum PWM pulse width in microseconds

// Define PWM input pins (assuming these are used for future reading or control)
const int PWM_INPUT_PINS[PPM_CHANNEL_COUNT] = {34, 35, 32, 33, 25, 26};

// Define PPM output pin
const int PPM_OUTPUT_PIN = 4; // Example pin for PPM output

// Define predefined PWM values for each channel
int pwmValues[PPM_CHANNEL_COUNT] = {2000, 1500, 1500, 1500, 1500, 1500};

volatile bool updatePPM = false;

// Timer interrupt handler
void IRAM_ATTR onTimer(void* arg) {
  static int channel = 0;
  static int pulseWidth = 0;

  if (updatePPM) {
    // Generate the PPM pulse
    digitalWrite(PPM_OUTPUT_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(PPM_OUTPUT_PIN, LOW);

    // Move to the next channel
    channel++;
    if (channel >= PPM_CHANNEL_COUNT) {
      // Reset channel and set synchronization pulse width
      channel = 0;
      pulseWidth = PPM_SYNC_PULSE_WIDTH;
    } else {
      // Set pulse width for the current channel
      pulseWidth = pwmValues[channel];
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Set PWM input pins
  for (int i = 0; i < PPM_CHANNEL_COUNT; i++) {
    pinMode(PWM_INPUT_PINS[i], INPUT);
  }

  // Set PPM output pin
  pinMode(PPM_OUTPUT_PIN, OUTPUT);

  // Initialize hardware timer
  timer_config_t config;
  config.divider = 80; // 1 tick = 1 microsecond (80 MHz / 80)
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE; // Timer initially paused
  config.alarm_en = TIMER_ALARM_EN;
  config.auto_reload = TIMER_AUTORELOAD_EN; // Enable auto-reload
  config.intr_type = TIMER_INTR_LEVEL;
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 20000); // 20 ms period
  timer_enable_intr(TIMER_GROUP_0, TIMER_0);
  timer_isr_register(TIMER_GROUP_0, TIMER_0, onTimer, NULL, 0, NULL);

  // Start the timer
  timer_start(TIMER_GROUP_0, TIMER_0);
}

void loop() {
  // Update PWM values if needed (for demonstration, predefined values are used)
  Serial.println("PWM Values:");
  for (int i = 0; i < PPM_CHANNEL_COUNT; i++) {
    pwmValues[i] = constrain(pwmValues[i], PPM_PULSE_WIDTH_MIN, PPM_PULSE_WIDTH_MAX);

    // Print PWM values to the Serial Monitor
    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(pwmValues[i]);
  }

  updatePPM = true;
  delay(10); // Update rate in milliseconds
}

