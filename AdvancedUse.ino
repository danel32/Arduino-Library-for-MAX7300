/**
 * AdvancedUse.ino - Advanced features example for the MAX7300 I2C GPIO Port Expander
 * 
 * This example demonstrates advanced usage of the MAX7300 library:
 * - Efficient batch operations for pin configuration
 * - Interrupt/transition detection configuration
 * - Interrupt-driven event handling
 * - Low power mode management
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect MAX7300 INT pin (normally pin 31) to Arduino pin 2
 * - Connect LEDs to pins 8-15 (with appropriate resistors)
 * - Connect buttons to pins 16-19 (with appropriate resistors)
 * 
 * Created by danel32, 2023-04-24
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Define pins
const uint8_t INTERRUPT_PIN = 2;     // Arduino pin connected to MAX7300 INT output
const uint8_t FIRST_LED_PIN = 8;     // First LED pin
const uint8_t LED_COUNT = 8;         // Number of LEDs
const uint8_t FIRST_BUTTON_PIN = 16; // First button pin
const uint8_t BUTTON_COUNT = 4;      // Number of buttons

// Variables to track the last button states
uint32_t lastButtonStates = 0;
uint32_t buttonMask = 0;

// Interrupt callback function
void transitionHandler(uint32_t changedPins) {
  Serial.print("Interrupt! Changed pins: 0x");
  Serial.println(changedPins, HEX);
  
  // Light up LEDs based on which buttons changed
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    uint8_t buttonPin = FIRST_BUTTON_PIN + i;
    uint8_t ledPin = FIRST_LED_PIN + i;
    
    // Check if this button triggered the interrupt
    if (changedPins & (1UL << buttonPin)) {
      // Read the new button state
      int buttonState = gpio.digitalRead(buttonPin);
      
      // Update the LED
      gpio.digitalWrite(ledPin, buttonState);
      
      Serial.print("Button ");
      Serial.print(i);
      Serial.print(" changed to: ");
      Serial.println(buttonState ? "HIGH" : "LOW");
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Advanced Usage Example");
  
  // Initialize the MAX7300 with custom I2C speed
  Serial.println("Initializing MAX7300...");
  if (gpio.begin(true, 100000) != MAX7300_OK) {
    Serial.print("Error: Could not initialize MAX7300! Error code: ");
    Serial.println(gpio.getLastError());
    Serial.println(gpio.getErrorMessage());
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Configure all LED pins as outputs using batch operation
  uint32_t ledPinMask = 0;
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    ledPinMask |= (1UL << (FIRST_LED_PIN + i));
  }
  
  if (gpio.setMultiplePinMode(ledPinMask, MAX7300_PIN_OUTPUT) != MAX7300_OK) {
    Serial.print("Error configuring LED pins: ");
    Serial.println(gpio.getErrorMessage());
  } else {
    Serial.println("LED pins configured successfully");
  }
  
  // Set all LEDs to off initially
  uint8_t ledValues[LED_COUNT] = {0};
  gpio.digitalWriteRange(FIRST_LED_PIN, FIRST_LED_PIN + LED_COUNT - 1, ledValues);
  
  // Configure button pins as inputs
  buttonMask = 0;
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    buttonMask |= (1UL << (FIRST_BUTTON_PIN + i));
  }
  
  if (gpio.setMultiplePinMode(buttonMask, MAX7300_PIN_INPUT) != MAX7300_OK) {
    Serial.print("Error configuring button pins: ");
    Serial.println(gpio.getErrorMessage());
  } else {
    Serial.println("Button pins configured successfully");
  }
  
  // Read initial button states
  lastButtonStates = gpio.digitalReadAll() & buttonMask;
  
  // Configure transition detection for all button pins
  if (gpio.configureMultipleTransitionDetection(buttonMask, MAX7300_TRANSITION_BOTH) != MAX7300_OK) {
    Serial.print("Error configuring transition detection: ");
    Serial.println(gpio.getErrorMessage());
  } else {
    Serial.println("Transition detection configured successfully");
  }
  
  // Attach interrupt handler
  if (gpio.attachInterrupt(INTERRUPT_PIN, transitionHandler) != MAX7300_OK) {
    Serial.print("Error setting up interrupt: ");
    Serial.println(gpio.getErrorMessage());
  } else {
    Serial.println("Interrupt handler attached successfully");
  }
  
  // Demonstrate power management - we won't actually shut down here
  // but this shows how to do it
  Serial.println("Device is currently: " + String(gpio.isEnabled() ? "ENABLED" : "DISABLED"));
  
  Serial.println("Ready! Press buttons to see interrupt-driven events.");
}

void loop() {
  // Most of the work happens in the interrupt handler
  // We'll just periodically show the current state of all pins
  
  static unsigned long lastStatusTime = 0;
  unsigned long currentTime = millis();
  
  // Update status every 5 seconds
  if (currentTime - lastStatusTime >= 5000) {
    lastStatusTime = currentTime;
    
    uint32_t allPins = gpio.digitalReadAll();
    
    Serial.println("\n-- Current Pin States --");
    
    // Show LED states
    Serial.print("LEDs (pins 8-15): ");
    for (int i = LED_COUNT - 1; i >= 0; i--) {
      Serial.print((allPins & (1UL << (FIRST_LED_PIN + i))) ? "1" : "0");
    }
    Serial.println();
    
    // Show button states
    Serial.print("Buttons (pins 16-19): ");
    for (int i = BUTTON_COUNT - 1; i >= 0; i--) {
      Serial.print((allPins & (1UL << (FIRST_BUTTON_PIN + i))) ? "1" : "0");
    }
    Serial.println();
    
    // Show transition detection status
    Serial.print("Transition detection: ");
    Serial.println(gpio.isTransitionDetectionEnabled() ? "Enabled" : "Disabled");
    
    Serial.println("----------------------\n");
  }
  
  // Demonstration of polling method when interrupts aren't available
  // This is an alternative to the interrupt method shown above
  if (gpio.checkInterrupt()) {
    Serial.println("Interrupt detected through polling!");
    
    // Read all pin states and compare with last known states to find changes
    uint32_t currentStates = gpio.digitalReadAll();
    uint32_t changedButtons = (currentStates ^ lastButtonStates) & buttonMask;
    
    if (changedButtons) {
      Serial.print("Changed buttons (polling): 0x");
      Serial.println(changedButtons, HEX);
      
      // Update the last known states
      lastButtonStates = currentStates & buttonMask;
    }
    
    // Clear the interrupt flag
    gpio.clearTransitionEvents();
  }
  
  // Small delay to avoid excessive CPU usage
  delay(10);
}
