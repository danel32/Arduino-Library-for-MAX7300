
/**
 * MultipleDevices.ino - Example for controlling multiple MAX7300 devices
 * 
 * This example demonstrates how to use multiple MAX7300 devices on the same I2C bus
 * by assigning different addresses to each device.
 * 
 * Hardware setup:
 * - Connect two MAX7300 devices to Arduino via I2C (SDA, SCL)
 * - First MAX7300:  A0=L, A1=L, A2=L (0x40)
 * - Second MAX7300: A0=H, A1=L, A2=L (0x42)
 * - Connect LEDs to pins 8-15 on first device (with appropriate resistors)
 * - Connect buttons to pins 8-15 on second device (with appropriate resistors)
 * 
 * Created by danel32, 2023-04-23
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create two MAX7300 objects with different addresses
MAX7300 gpio1(0x40); // First device (A0=L, A1=L, A2=L)
MAX7300 gpio2(0x42); // Second device (A0=H, A1=L, A2=L)

// Pin assignments
const uint8_t FIRST_LED_PIN = 8;      // First LED pin on device 1
const uint8_t FIRST_BUTTON_PIN = 8;   // First button pin on device 2
const uint8_t NUM_PINS = 8;           // Number of LEDs and buttons

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Multiple Devices Example");
  
  // Initialize the first MAX7300
  Serial.println("Initializing first MAX7300 (0x40)...");
  if (gpio1.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize first MAX7300!");
    Serial.println("Check wiring and I2C address.");
    while (1); // Stop execution if initialization fails
  }
  
  // Initialize the second MAX7300
  Serial.println("Initializing second MAX7300 (0x42)...");
  if (gpio2.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize second MAX7300!");
    Serial.println("Check wiring and I2C address.");
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("Both MAX7300 devices initialized successfully!");
  
  // Configure pins on first device (LED outputs)
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    gpio1.pinMode(FIRST_LED_PIN + i, MAX7300_PIN_OUTPUT);
    gpio1.digitalWrite(FIRST_LED_PIN + i, LOW); // Turn off all LEDs initially
  }
  Serial.println("First MAX7300: Pins 8-15 configured as outputs (LEDs)");
  
  // Configure pins on second device (button inputs)
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    gpio2.pinMode(FIRST_BUTTON_PIN + i, MAX7300_PIN_INPUT);
  }
  Serial.println("Second MAX7300: Pins 8-15 configured as inputs (buttons)");
}

void loop() {
  // Read all button states from second device
  uint32_t buttonStates = 0;
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    int state = gpio2.digitalRead(FIRST_BUTTON_PIN + i);
    if (state == 1) {
      buttonStates |= (1UL << i);
    }
  }
  
  // Display button states in binary
  Serial.print("Button states: ");
  for (int i = NUM_PINS - 1; i >= 0; i--) {
    Serial.print((buttonStates & (1UL << i)) ? '1' : '0');
  }
  Serial.println();
  
  // Update LEDs to match button states
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    gpio1.digitalWrite(FIRST_LED_PIN + i, (buttonStates & (1UL << i)) ? HIGH : LOW);
  }
  
  delay(100); // Small delay to avoid serial monitor flooding
}
