/**
 * AdvancedInterrupts.ino - Example demonstrating improved interrupt handling
 * 
 * This example shows how to use the enhanced interrupt functionality of the MAX7300
 * library, including the new event-based callback system that provides detailed
 * information about which pin triggered the interrupt and its current state.
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect MAX7300 INT pin (pin 31) to Arduino pin 2 (interrupt capable)
 * - Connect buttons to pins 8, 9, 10, 11 (with pull-up resistors)
 * - Connect LEDs to pins 16, 17, 18, 19 (with appropriate resistors)
 * 
 * Created by danel32, 2025-04-24
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Pin assignments
const uint8_t MAX7300_INT_PIN = 2;  // Arduino pin connected to MAX7300 INT output
const uint8_t BUTTONS[] = {8, 9, 10, 11}; // Button pins on MAX7300
const uint8_t LEDS[] = {16, 17, 18, 19};  // LED pins on MAX7300
const uint8_t PIN_COUNT = 4;              // Number of buttons/LEDs

// Tracking variables
volatile bool eventOccurred = false;
volatile MAX7300Event lastEvent;
unsigned long lastPollTime = 0;
unsigned long pollInterval = 100; // Polling interval in ms

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Advanced Interrupt Handling Example");
  
  // Initialize the MAX7300
  Serial.println("Initializing MAX7300...");
  if (gpio.begin() != MAX7300_OK) {
    Serial.print("Error: Could not initialize MAX7300! ");
    Serial.println(gpio.getErrorMessage());
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Configure buttons as inputs with transition detection
  for (uint8_t i = 0; i < PIN_COUNT; i++) {
    gpio.pinMode(BUTTONS[i], MAX7300_PIN_INPUT);
    
    // Configure both rising and falling edge detection
    gpio.configureTransitionDetection(BUTTONS[i], MAX7300_TRANSITION_BOTH);
  }
  
  // Configure LEDs as outputs
  for (uint8_t i = 0; i < PIN_COUNT; i++) {
    gpio.pinMode(LEDS[i], MAX7300_PIN_OUTPUT);
    gpio.digitalWrite(LEDS[i], LOW);
  }
  
  // Attach interrupt handler (event-based)
  if (gpio.attachEventInterrupt(MAX7300_INT_PIN, handleEvent) != MAX7300_OK) {
    Serial.print("Failed to attach interrupt handler! Error: ");
    Serial.println(gpio.getErrorMessage());
  }
  
  // Enable transition detection
  gpio.enableTransitionDetection();
  
  // Display demo info
  Serial.println("\nInterrupt handling demo:");
  Serial.println("1. Event-based interrupts: Press any button to see events with pin and level info");
  Serial.println("2. Polling mode: System checks for events periodically");
  Serial.println("\nBoth modes are active simultaneously for comparison.\n");
}

void loop() {
  // Check for interrupt event (set by the event handler)
  if (eventOccurred) {
    eventOccurred = false;
    
    // LED index corresponds to button index
    uint8_t buttonIndex = 0xFF;
    for (uint8_t i = 0; i < PIN_COUNT; i++) {
      if (BUTTONS[i] == lastEvent.pin) {
        buttonIndex = i;
        break;
      }
    }
    
    if (buttonIndex != 0xFF) {
      // Update LED to match button state
      gpio.digitalWrite(LEDS[buttonIndex], lastEvent.level);
      
      Serial.print("[EVENT] Button ");
      Serial.print(buttonIndex + 1);
      Serial.print(" (pin ");
      Serial.print(lastEvent.pin);
      Serial.print(") changed to: ");
      Serial.println(lastEvent.level ? "RELEASED" : "PRESSED");
    }
  }
  
  // Demonstrate polling method (for comparison)
  if (millis() - lastPollTime >= pollInterval) {
    lastPollTime = millis();
    
    MAX7300Event polledEvent;
    if (gpio.pollEvent(&polledEvent)) {
      // Find which button this corresponds to
      uint8_t buttonIndex = 0xFF;
      for (uint8_t i = 0; i < PIN_COUNT; i++) {
        if (BUTTONS[i] == polledEvent.pin) {
          buttonIndex = i;
          break;
        }
      }
      
      if (buttonIndex != 0xFF) {
        Serial.print("[POLLED] Button ");
        Serial.print(buttonIndex + 1);
        Serial.print(" (pin ");
        Serial.print(polledEvent.pin);
        Serial.print(") detected at level: ");
        Serial.println(polledEvent.level ? "RELEASED" : "PRESSED");
      }
    }
  }
  
  // Small delay to avoid excessive CPU usage
  delay(10);
}

// Event-based interrupt handler (called by MAX7300 library)
void handleEvent(MAX7300Event event) {
  // Store the event information for processing in the main loop
  lastEvent = event;
  eventOccurred = true;
  
  // Note: This function is called from an interrupt context, so keep it short!
  // Avoid serial prints or other time-consuming operations here.
}
