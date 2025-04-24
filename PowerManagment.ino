
/**
 * PowerManagement.ino - Example demonstrating advanced power management features
 * 
 * This example shows how to use the power management features of the MAX7300 library
 * to minimize power consumption in battery-powered applications. It demonstrates
 * normal, standby, and shutdown modes, as well as saving/restoring configurations.
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect LED to pin 8 (with appropriate resistor)
 * - Connect button to pin 16 (with pull-up resistor)
 * 
 * Created by danel32, 2025-04-24
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Pin assignments
const uint8_t LED_PIN = 8;        // LED pin
const uint8_t BUTTON_PIN = 16;    // Button pin
const uint8_t EEPROM_ADDR = 0;    // Starting address in EEPROM for config storage

// Power mode tracking
uint8_t currentPowerMode = MAX7300_POWER_NORMAL;
unsigned long lastStateChange = 0;
unsigned long powerModeDisplayTime = 5000; // Time to display current power mode in ms

// LED patterns
const uint8_t PATTERN_NORMAL[] =   {1, 0, 1, 0, 1, 0};   // Fast blink
const uint8_t PATTERN_STANDBY[] =  {1, 0, 0, 0, 1, 0, 0, 0}; // Medium blink
const uint8_t PATTERN_SHUTDOWN[] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Slow blink

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Advanced Power Management Example");
  
  // Initialize the MAX7300
  Serial.println("Initializing MAX7300...");
  if (gpio.begin() != MAX7300_OK) {
    Serial.print("Error: Could not initialize MAX7300! ");
    Serial.println(gpio.getErrorMessage());
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Set up pins
  gpio.pinMode(LED_PIN, MAX7300_PIN_OUTPUT);
  gpio.pinMode(BUTTON_PIN, MAX7300_PIN_INPUT);
  
  // Initial state
  gpio.digitalWrite(LED_PIN, LOW);
  
  // Try to load configuration from EEPROM
  bool configLoaded = false;
  
  if (gpio.loadConfiguration(EEPROM_ADDR) == MAX7300_OK) {
    Serial.println("Configuration loaded from EEPROM.");
    configLoaded = true;
  } else {
    Serial.println("No valid configuration in EEPROM. Using defaults.");
    
    // Save the current configuration as default
    if (gpio.saveConfiguration(EEPROM_ADDR) == MAX7300_OK) {
      Serial.println("Default configuration saved to EEPROM.");
    } else {
      Serial.println("Failed to save configuration to EEPROM.");
    }
  }
  
  Serial.println("Current power mode: NORMAL");
  Serial.println("\nPress button to cycle through power modes:");
  Serial.println("1. Normal mode: Full functionality, highest power consumption");
  Serial.println("2. Standby mode: Reduced power, maintains pin states");
  Serial.println("3. Shutdown mode: Minimum power, pins reset to high-Z");
  
  // Initialize state change tracker
  lastStateChange = millis();
}

void loop() {
  // Handle button press to change power modes
  static bool lastButtonState = HIGH;
  int8_t buttonState = gpio.digitalRead(BUTTON_PIN);
  
  if (buttonState == 0 && lastButtonState == HIGH) {
    // Button pressed, cycle through power modes
    cycleNextPowerMode();
    lastButtonState = LOW;
    delay(50); // Simple debounce
  } 
  else if (buttonState == 1 && lastButtonState == LOW) {
    lastButtonState = HIGH;
    delay(50); // Simple debounce
  }
  
  // Display appropriate LED pattern based on current power mode
  if (millis() - lastStateChange < powerModeDisplayTime) {
    displayModeLED();
  } else {
    // After display time, actually apply the power mode
    applyPowerMode();
  }
  
  delay(50); // Small delay for stability
}

void cycleNextPowerMode() {
  // Cycle to the next power mode
  switch (currentPowerMode) {
    case MAX7300_POWER_NORMAL:
      currentPowerMode = MAX7300_POWER_STANDBY;
      Serial.println("\nSwitching to STANDBY mode in 5 seconds...");
      break;
      
    case MAX7300_POWER_STANDBY:
      currentPowerMode = MAX7300_POWER_SHUTDOWN;
      Serial.println("\nSwitching to SHUTDOWN mode in 5 seconds...");
      Serial.println("Configuration will be saved to EEPROM first.");
      break;
      
    case MAX7300_POWER_SHUTDOWN:
      currentPowerMode = MAX7300_POWER_NORMAL;
      Serial.println("\nSwitching to NORMAL mode in 5 seconds...");
      Serial.println("Configuration will be restored from EEPROM.");
      break;
  }
  
  lastStateChange = millis();
}

void displayModeLED() {
  // Display a pattern on the LED to indicate the selected mode
  unsigned long patternTime;
  const uint8_t* pattern;
  uint8_t patternLength;
  
  switch (currentPowerMode) {
    case MAX7300_POWER_NORMAL:
      pattern = PATTERN_NORMAL;
      patternLength = sizeof(PATTERN_NORMAL);
      patternTime = 150; // Fast blink
      break;
      
    case MAX7300_POWER_STANDBY:
      pattern = PATTERN_STANDBY;
      patternLength = sizeof(PATTERN_STANDBY);
      patternTime = 250; // Medium blink
      break;
      
    case MAX7300_POWER_SHUTDOWN:
      pattern = PATTERN_SHUTDOWN;
      patternLength = sizeof(PATTERN_SHUTDOWN);
      patternTime = 400; // Slow blink
      break;
  }
  
  // Calculate pattern index based on time
  unsigned long elapsed = millis() % (patternLength * patternTime);
  uint8_t index = elapsed / patternTime;
  
  // Set LED based on pattern
  gpio.digitalWrite(LED_PIN, pattern[index]);
}

void applyPowerMode() {
  // Apply the selected power mode
  static bool modeApplied = false;
  
  if (!modeApplied) {
    uint8_t currentMode = gpio.getPowerMode();
    
    // Only change if not already in the desired mode
    if (currentMode != currentPowerMode) {
      switch (currentPowerMode) {
        case MAX7300_POWER_NORMAL:
          // If coming from shutdown, restore from EEPROM
          if (currentMode == MAX7300_POWER_SHUTDOWN) {
            Serial.println("Restoring configuration from EEPROM...");
            gpio.loadConfiguration(EEPROM_ADDR, true);
          } else {
            gpio.setPowerMode(MAX7300_POWER_NORMAL);
          }
          
          Serial.println("Normal mode activated. Full functionality enabled.");
          Serial.print("Power consumption: HIGH");
          break;
          
        case MAX7300_POWER_STANDBY:
          gpio.standby(true); // Preserve output states
          Serial.println("Standby mode activated. Pin states preserved.");
          Serial.print("Power consumption: MEDIUM");
          break;
          
        case MAX7300_POWER_SHUTDOWN:
          // Save configuration before shutdown
          Serial.println("Saving configuration to EEPROM...");
          gpio.saveConfiguration(EEPROM_ADDR);
          
          gpio.shutdown(); // Full shutdown mode
          Serial.println("Shutdown mode activated. All pins high-impedance.");
          Serial.print("Power consumption: LOW");
          break;
      }
      
      Serial.println(" (Check power meter for exact value)");
      modeApplied = true;
    }
  }
}
