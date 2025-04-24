* TransitionDetection.ino - Example for using the transition detection feature
 * 
 * This example demonstrates how to use the MAX7300's transition detection
 * functionality to efficiently detect changes on input pins without polling.
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect buttons or sensors to pins 4, 5, and 6 (with pull-up resistors)
 * - Optional: Connect the MAX7300 INT pin to Arduino interrupt pin
 * 
 * Note: Transition detection works only on pins 4-27
 * 
 * Created by danel32, 2023-04-23
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Pin assignments
const uint8_t NUM_INPUT_PINS = 3;
const uint8_t INPUT_PINS[NUM_INPUT_PINS] = {4, 5, 6}; // Pins to monitor

// Interrupt pin (optional, connect to MAX7300 INT pin)
const uint8_t INT_PIN = 2;

// Variables to track pin states
uint8_t lastPinState[NUM_INPUT_PINS] = {0};

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Transition Detection Example");
  
  // If using interrupt pin
  pinMode(INT_PIN, INPUT_PULLUP);
  
  // Initialize the MAX7300
  Serial.println("Initializing MAX7300...");
  
  if (gpio.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300!");
    Serial.println("Check wiring and I2C address.");
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Configure pins for input
  for (uint8_t i = 0; i < NUM_INPUT_PINS; i++) {
    gpio.pinMode(INPUT_PINS[i], MAX7300_PIN_INPUT);
    
    // Read initial state
    lastPinState[i] = gpio.digitalRead(INPUT_PINS[i]);
  }
  
  // Enable transition detection
  gpio.enableTransitionDetection();
  
  // Configure transition detection for each pin
  // We'll detect both rising and falling edges for this example
  for (uint8_t i = 0; i < NUM_INPUT_PINS; i++) {
    gpio.configureTransitionDetection(INPUT_PINS[i], MAX7300_TRANSITION_BOTH);
  }
  
  Serial.println("Transition detection enabled on pins 4, 5, and 6");
  Serial.println("Waiting for pin state changes...");
}

void loop() {
  // Check if a transition was detected
  // Either by polling the device or via the INT pin (if connected)
  
  bool transitionOccurred = false;
  
  // If using interrupt pin, check if it's LOW (active)
  if (digitalRead(INT_PIN) == LOW) {
    transitionOccurred = true;
  }
  // Otherwise poll the device
  else if (gpio.transitionDetected()) {
    transitionOccurred = true;
  }
  
  // If a transition was detected
  if (transitionOccurred) {
    // Get which pins had transitions
    uint32_t events = gpio.getTransitionEvents();
    
    Serial.println("Transition detected!");
    
    // Check each monitored pin
    for (uint8_t i = 0; i < NUM_INPUT_PINS; i++) {
      uint8_t pin = INPUT_PINS[i];
      
      // If this pin triggered a transition
      if (events & (1UL << pin)) {
        // Read the new state
        uint8_t newState = gpio.digitalRead(pin);
        
        // Only report if state actually changed (eliminates potential false positives)
        if (newState != lastPinState[i]) {
          Serial.print("Pin ");
          Serial.print(pin);
          Serial.print(" changed from ");
          Serial.print(lastPinState[i]);
          Serial.print(" to ");
          Serial.println(newState);
          
          // Update last known state
          lastPinState[i] = newState;
        }
      }
    }
    
    // Clear transition events
    gpio.clearTransitionEvents();
  }
  
  // Small delay
  delay(10);
}
