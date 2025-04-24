/**
 * BatchOperations.ino - Example demonstrating efficient batch operations
 * 
 * This example shows how to use the batch operations features of the MAX7300 
 * library to efficiently configure and control multiple pins at once, resulting
 * in reduced I2C traffic and improved performance.
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect 8 LEDs to pins 8-15 (with appropriate resistors)
 * - Connect 8 buttons to pins 16-23 (with appropriate resistors)
 * 
 * Created by danel32, 2025-04-24
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Pin assignments
const uint8_t FIRST_LED_PIN = 8;     // First LED pin
const uint8_t LED_COUNT = 8;         // Number of LEDs
const uint8_t FIRST_BUTTON_PIN = 16; // First button pin  
const uint8_t BUTTON_COUNT = 8;      // Number of buttons

// Patterns for LED animations
const uint8_t PATTERNS[][8] = {
  {1, 0, 1, 0, 1, 0, 1, 0}, // Alternating
  {1, 1, 1, 1, 0, 0, 0, 0}, // Half and half
  {1, 1, 0, 0, 1, 1, 0, 0}, // Pairs
  {0, 1, 1, 1, 1, 1, 1, 0}, // Middle six
  {1, 0, 0, 0, 0, 0, 0, 1}  // Ends only
};
const uint8_t PATTERN_COUNT = 5;

// Buffer for button readings
uint8_t buttonStates[8];

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Batch Operations Example");
  
  // Initialize the MAX7300
  Serial.println("Initializing MAX7300...");
  if (gpio.begin() != MAX7300_OK) {
    Serial.print("Error: Could not initialize MAX7300! ");
    Serial.println(gpio.getErrorMessage());
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Benchmark start time for standard pin configuration
  unsigned long startTime = micros();
  
  // Configure LED pins as outputs using individual calls (standard method)
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    gpio.pinMode(FIRST_LED_PIN + i, MAX7300_PIN_OUTPUT);
  }
  
  unsigned long standardTime = micros() - startTime;
  
  // Benchmark start time for batch pin configuration
  startTime = micros();
  
  // Configure button pins as inputs using batch operation
  uint32_t buttonPinMask = 0;
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    buttonPinMask |= (1UL << (FIRST_BUTTON_PIN + i));
  }
  gpio.setMultiplePinMode(buttonPinMask, MAX7300_PIN_INPUT);
  
  unsigned long batchTime = micros() - startTime;
  
  // Print timing comparison
  Serial.println("\nPerformance Comparison:");
  Serial.print("Time to configure 8 pins individually: ");
  Serial.print(standardTime);
  Serial.println(" microseconds");
  
  Serial.print("Time to configure 8 pins in batch: ");
  Serial.print(batchTime);
  Serial.println(" microseconds");
  
  Serial.print("Improvement: ");
  Serial.print((float)standardTime / batchTime, 1);
  Serial.println("x faster\n");
  
  Serial.println("Press buttons to control LED patterns. Multiple buttons = combine patterns.");
}

void loop() {
  // Read all button states in a single efficient operation
  gpio.digitalReadRange(FIRST_BUTTON_PIN, FIRST_BUTTON_PIN + BUTTON_COUNT - 1, buttonStates);
  
  // Check if any button is pressed
  bool anyButtonPressed = false;
  for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
    if (buttonStates[i] & 0x01) {
      anyButtonPressed = true;
      break;
    }
  }
  
  // Prepare LED values
  uint8_t ledValues[LED_COUNT] = {0};
  
  if (anyButtonPressed) {
    // If buttons are pressed, combine selected patterns
    for (uint8_t i = 0; i < BUTTON_COUNT && i < PATTERN_COUNT; i++) {
      if (buttonStates[i] & 0x01) {
        // Overlay this pattern onto the LEDs
        for (uint8_t j = 0; j < LED_COUNT; j++) {
          ledValues[j] |= PATTERNS[i][j];
        }
      }
    }
  } else {
    // No buttons pressed - animate through patterns
    static uint8_t currentPattern = 0;
    static unsigned long lastPatternChange = 0;
    
    if (millis() - lastPatternChange > 1000) {
      lastPatternChange = millis();
      currentPattern = (currentPattern + 1) % PATTERN_COUNT;
      
      Serial.print("Displaying pattern #");
      Serial.println(currentPattern + 1);
    }
    
    // Copy the current pattern to the LED values
    for (uint8_t i = 0; i < LED_COUNT; i++) {
      ledValues[i] = PATTERNS[currentPattern][i];
    }
  }
  
  // Update all LEDs at once with a single I2C transaction
  gpio.digitalWriteRange(FIRST_LED_PIN, FIRST_LED_PIN + LED_COUNT - 1, ledValues);
  
  // Print button states periodically
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 500 && anyButtonPressed) {
    lastPrintTime = millis();
    
    Serial.print("Button states: ");
    for (int i = 0; i < BUTTON_COUNT; i++) {
      Serial.print((buttonStates[i] & 0x01) ? "1" : "0");
    }
    Serial.println();
  }
  
  // Small delay to avoid excessive I2C traffic
  delay(50);
}
