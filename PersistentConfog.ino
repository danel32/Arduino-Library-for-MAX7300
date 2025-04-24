
/**
 * PersistentConfig.ino - Example demonstrating configuration storage and restoration
 * 
 * This example shows how to save MAX7300 configuration to EEPROM and restore it
 * after power cycles or resets. This is especially useful for battery-powered
 * applications or scenarios where maintaining configuration across power cycles
 * is important.
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect LEDs to pins 8-15 (with appropriate resistors)
 * - Connect button to pin 16 (with pull-up resistor)
 * 
 * Created by danel32, 2025-04-24
 * Released under MIT License
 */

#include <Wire.h>
#include <EEPROM.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Pin assignments
const uint8_t FIRST_LED_PIN = 8;   // First LED pin
const uint8_t LED_COUNT = 8;       // Number of LEDs
const uint8_t BUTTON_PIN = 16;     // Button pin
const uint16_t EEPROM_ADDR = 0;    // Starting address in EEPROM for config storage

// State tracking
bool configurationSaved = false;
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 100; // Button debounce time in ms

// Example configurations
const uint8_t CONFIG_PATTERNS[][8] = {
  {1, 0, 1, 0, 1, 0, 1, 0}, // Alternating
  {1, 1, 1, 1, 0, 0, 0, 0}, // Half and half
  {1, 1, 0, 0, 1, 1, 0, 0}, // Pairs
  {0, 1, 1, 1, 1, 1, 1, 0}  // Middle six
};
const uint8_t CONFIG_COUNT = 4;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Persistent Configuration Example");
  
  // Initialize the MAX7300
  Serial.println("Initializing MAX7300...");
  if (gpio.begin() != MAX7300_OK) {
    Serial.print("Error: Could not initialize MAX7300! ");
    Serial.println(gpio.getErrorMessage());
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Configure input pin
  gpio.pinMode(BUTTON_PIN, MAX7300_PIN_INPUT);
  
  // Try to load configuration from EEPROM
  Serial.println("Checking for saved configuration in EEPROM...");
  
  if (gpio.loadConfiguration(EEPROM_ADDR) == MAX7300_OK) {
    Serial.println("Configuration loaded from EEPROM successfully!");
    configurationSaved = true;
  } else {
    Serial.println("No valid configuration found in EEPROM.");
    Serial.println("Setting up default configuration...");
    
    // Configure all LED pins as outputs with default pattern
    for (uint8_t i = 0; i < LED_COUNT; i++) {
      gpio.pinMode(FIRST_LED_PIN + i, MAX7300_PIN_OUTPUT);
      gpio.digitalWrite(FIRST_LED_PIN + i, CONFIG_PATTERNS[0][i]);
    }
    
    configurationSaved = false;
  }
  
  Serial.println("\nPress the button to:");
  Serial.println("- Short press: Cycle through LED patterns");
  Serial.println("- Long press (>2s): Save current configuration to EEPROM");
  Serial.println("\nAfter saving, reset the Arduino to demonstrate configuration persistence");
}

void loop() {
  // Check button state
  int8_t buttonState = gpio.digitalRead(BUTTON_PIN);
  static bool buttonPressed = false;
  static unsigned long pressStartTime = 0;
  
  // Button press started
  if (buttonState == 0 && !buttonPressed) {
    buttonPressed = true;
    pressStartTime = millis();
  }
  
  // Button released
  if (buttonState == 1 && buttonPressed) {
    unsigned long pressDuration = millis() - pressStartTime;
    buttonPressed = false;
    
    if (pressDuration > 2000) {
      // Long press - save configuration
      saveConfiguration();
    } else if (pressDuration > 50) {
      // Short press - change pattern
      cyclePattern();
    }
    
    // Debounce
    delay(50);
  }
  
  // If button is being held down, provide visual feedback after 2 seconds
  if (buttonPressed && (millis() - pressStartTime) > 2000) {
    // Flash all LEDs rapidly to indicate save action is ready
    static bool flashState = false;
    static unsigned long lastFlash = 0;
    
    if (millis() - lastFlash > 100) {
      flashState = !flashState;
      for (uint8_t i = 0; i < LED_COUNT; i++) {
        gpio.digitalWrite(FIRST_LED_PIN + i, flashState);
      }
      lastFlash = millis();
    }
  }
  
  delay(10);
}

void cyclePattern() {
  // Find out which pattern we're currently displaying
  uint8_t currentPattern = 0;
  bool matched = false;
  
  // Read the current LED states
  uint8_t ledStates[LED_COUNT];
  gpio.digitalReadRange(FIRST_LED_PIN, FIRST_LED_PIN + LED_COUNT - 1, ledStates);
  
  // Compare with patterns
  for (uint8_t p = 0; p < CONFIG_COUNT; p++) {
    bool patternMatches = true;
    
    for (uint8_t i = 0; i < LED_COUNT; i++) {
      if ((ledStates[i] & 0x01) != CONFIG_PATTERNS[p][i]) {
        patternMatches = false;
        break;
      }
    }
    
    if (patternMatches) {
      currentPattern = p;
      matched = true;
      break;
    }
  }
  
  // If no match or last pattern, go to first pattern, otherwise next pattern
  uint8_t newPattern;
  if (!matched || currentPattern >= CONFIG_COUNT - 1) {
    newPattern = 0;
  } else {
    newPattern = currentPattern + 1;
  }
  
  // Apply the new pattern
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    gpio.digitalWrite(FIRST_LED_PIN + i, CONFIG_PATTERNS[newPattern][i]);
  }
  
  Serial.print("Changed to pattern #");
  Serial.println(newPattern + 1);
}

void saveConfiguration() {
  Serial.println("Saving current configuration to EEPROM...");
  
  if (gpio.saveConfiguration(EEPROM_ADDR) == MAX7300_OK) {
    Serial.println("Configuration saved successfully!");
    
    // Visual confirmation
    for (uint8_t i = 0; i < 3; i++) {
      // All LEDs off
      for (uint8_t j = 0; j < LED_COUNT; j++) {
        gpio.digitalWrite(FIRST_LED_PIN + j, LOW);
      }
      delay(200);
      
      // All LEDs on
      for (uint8_t j = 0; j < LED_COUNT; j++) {
        gpio.digitalWrite(FIRST_LED_PIN + j, HIGH);
      }
      delay(200);
    }
    
    // Restore the current pattern
    uint8_t ledStates[LED_COUNT];
    gpio.digitalReadRange(FIRST_LED_PIN, FIRST_LED_PIN + LED_COUNT - 1, ledStates);
    
    configurationSaved = true;
    
    Serial.println("\nReset the Arduino now to test configuration persistence!");
  } else {
    Serial.print("Failed to save configuration! Error: ");
    Serial.println(gpio.getErrorMessage());
  }
}
