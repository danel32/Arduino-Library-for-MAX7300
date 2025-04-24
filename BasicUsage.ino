/**
 * BasicUsage.ino - Basic example for the MAX7300 I2C GPIO Port Expander
 * 
 * This example demonstrates the basic functionality of the MAX7300 library:
 * - Initializing the device
 * - Configuring pins as inputs and outputs
 * - Reading from input pins
 * - Writing to output pins
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect an LED to pin 10 (with appropriate resistor)
 * - Connect a button to pin 11 (with pull-up or pull-down resistor)
 * 
 * Created by danel32, 2023-04-23
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Pin assignments
const uint8_t LED_PIN = 10;     // Output pin connected to LED
const uint8_t BUTTON_PIN = 11;  // Input pin connected to button

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Basic Usage Example");
  
  // Initialize the MAX7300
  Serial.println("Initializing MAX7300...");
  
  if (gpio.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300!");
    Serial.println("Check wiring and I2C address.");
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Configure pins
  gpio.pinMode(LED_PIN, MAX7300_PIN_OUTPUT);     // Set LED pin as output
  gpio.pinMode(BUTTON_PIN, MAX7300_PIN_INPUT);   // Set button pin as input
  
  Serial.println("Pins configured:");
  Serial.print("- Pin ");
  Serial.print(LED_PIN);
  Serial.println(" configured as OUTPUT");
  
  Serial.print("- Pin ");
  Serial.print(BUTTON_PIN);
  Serial.println(" configured as INPUT");
  
  // Initial state - LED off
  gpio.digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Read the button state
  int buttonState = gpio.digitalRead(BUTTON_PIN);
  
  // Write the button state to the LED
  // If button is pressed (LOW), the LED turns on
  gpio.digitalWrite(LED_PIN, buttonState);
  
  // Print the state to serial monitor
  Serial.print("Button state: ");
  Serial.print(buttonState ? "HIGH" : "LOW");
  Serial.print(" | LED state: ");
  Serial.println(buttonState ? "ON" : "OFF");
  
  // Small delay to avoid serial monitor flooding
  delay(100);
}
