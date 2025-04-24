
/**
 * AdvancedFeatures.ino - Example showcasing advanced features of the MAX7300 library
 * 
 * This example demonstrates more complex features:
 * - Reading/writing multiple pins at once
 * - Direct register access
 * - Error handling
 * - Power management (sleep/wake)
 * 
 * Hardware setup:
 * - Connect MAX7300 to Arduino via I2C (SDA, SCL)
 * - Connect LEDs to pins 4-11 (with appropriate resistors)
 * - Connect buttons/switches to pins 12-19 (with appropriate resistors)
 * 
 * Created by danel32, 2023-04-23
 * Released under MIT License
 */

#include <Wire.h>
#include "MAX7300.h"

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

// Pin assignments
const uint8_t FIRST_LED_PIN = 4;       // First LED pin
const uint8_t FIRST_BUTTON_PIN = 12;    // First button pin
const uint8_t NUM_PINS = 8;            // Number of LEDs and buttons

// Patterns for LED animations
const uint32_t LED_PATTERNS[] = {
  0x55,  // 01010101
  0xAA,  // 10101010
  0x0F,  // 00001111
  0xF0,  // 11110000
  0xFF,  // 11111111
  0x00   // 00000000
};
const uint8_t NUM_PATTERNS = sizeof(LED_PATTERNS) / sizeof(LED_PATTERNS[0]);

// Function prototypes
void displayErrorMessage(int8_t errorCode);
void printRegisterValue(uint8_t reg);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MAX7300 Advanced Features Example");
  
  // Wait for serial monitor to open
  delay(1000);
  
  // Initialize the MAX7300 with error handling
  Serial.println("Initializing MAX7300...");
  
  int8_t result = gpio.begin();
  if (result != MAX7300_OK) {
    Serial.print("Initialization failed! Error code: ");
    displayErrorMessage(result);
    while (1); // Stop execution if initialization fails
  }
  
  Serial.println("MAX7300 initialized successfully!");
  
  // Print device information
  Serial.println("\n--- Device Information ---");
  printRegisterValue(MAX7300_REG_CONFIG);
  
  // Configure multiple pins at once using bitmasks
  uint32_t ledMask = 0;
  uint32_t buttonMask = 0;
  
  // Create bit masks for LEDs and buttons
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    ledMask |= (1UL << (FIRST_LED_PIN + i));
    buttonMask |= (1UL << (FIRST_BUTTON_PIN + i));
  }
  
  // Configure LEDs as outputs and buttons as inputs in batch operations
  Serial.println("\n--- Configuring multiple pins at once ---");
  
  result = gpio.setMultiplePinMode(ledMask, MAX7300_PIN_OUTPUT);
  if (result != MAX7300_OK) {
    Serial.print("Failed to configure LED pins! Error: ");
    displayErrorMessage(result);
  } else {
    Serial.println("LED pins configured successfully!");
  }
  
  result = gpio.setMultiplePinMode(buttonMask, MAX7300_PIN_INPUT);
  if (result != MAX7300_OK) {
    Serial.print("Failed to configure button pins! Error: ");
    displayErrorMessage(result);
  } else {
    Serial.println("Button pins configured successfully!");
  }
  
  // Show initial pin configurations
  Serial.println("\n--- Pin Configurations ---");
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    Serial.print("LED pin ");
    Serial.print(FIRST_LED_PIN + i);
    Serial.print(" mode: ");
    Serial.println(gpio.getPinMode(FIRST_LED_PIN + i));
    
    Serial.print("Button pin ");
    Serial.print(FIRST_BUTTON_PIN + i);
    Serial.print(" mode: ");
    Serial.println(gpio.getPinMode(FIRST_BUTTON_PIN + i));
  }
  
  Serial.println("\nStarting advanced operation demonstration...");
}

void loop() {
  // Demonstrate different features
  
  // 1. Read all buttons at once
  Serial.println("\n--- Reading all pins at once ---");
  uint32_t allPins = gpio.digitalReadAll();
  
  Serial.print("All pin states (binary): ");
  for (int i = MAX7300_MAX_PIN; i >= 0; i--) {
    Serial.print((allPins & (1UL << i)) ? '1' : '0');
    if (i % 4 == 0) Serial.print(' '); // Space every 4 bits for readability
  }
  Serial.println();
  
  // Extract button states
  uint32_t buttonStates = 0;
  for (uint8_t i = 0; i < NUM_PINS; i++) {
    if (allPins & (1UL << (FIRST_BUTTON_PIN + i))) {
      buttonStates |= (1UL << i);
    }
  }
  
  Serial.print("Button states (binary): ");
  for (int i = NUM_PINS - 1; i >= 0; i--) {
    Serial.print((buttonStates & (1UL << i)) ? '1' : '0');
  }
  Serial.println();
  
  // 2. Display LED patterns
  Serial.println("\n--- Displaying LED patterns ---");
  for (uint8_t p = 0; p < NUM_PATTERNS; p++) {
    Serial.print("Pattern ");
    Serial.print(p);
    Serial.print(": ");
    
    for (int i = 7; i >= 0; i--) {
      Serial.print((LED_PATTERNS[p] & (1UL << i)) ? '1' : '0');
    }
    Serial.println();
    
    // Create a 32-bit pattern with our 8-bit pattern in the right position
    uint32_t fullPattern = 0;
    for (uint8_t i = 0; i < NUM_PINS; i++) {
      if (LED_PATTERNS[p] & (1UL << i)) {
        fullPattern |= (1UL << (FIRST_LED_PIN + i));
      }
    }
    
    // Write pattern to all LEDs at once
    // Only modify the pins we care about using the mask
    uint32_t mask = 0;
    for (uint8_t i = 0; i < NUM_PINS; i++) {
      mask |= (1UL << (FIRST_LED_PIN + i));
    }
    
    gpio.digitalWriteAll(fullPattern, mask);
    delay(500);
  }
  
  // 3. Demonstrate power management
  Serial.println("\n--- Power Management Demo ---");
  
  Serial.println("Putting device into shutdown mode...");
  gpio.disable();
  delay(1000);
  
  Serial.println("Waking device up...");
  gpio.enable();
  delay(1000);
  
  // 4. Demonstrate direct register access (advanced users)
  Serial.println("\n--- Direct Register Access ---");
  
  for (uint8_t reg = MAX7300_REG_CONFIG; reg <= MAX7300_REG_PIN_CONFIG_BASE + 6; reg++) {
    printRegisterValue(reg);
  }
  
  Serial.println("\nEnd of demonstration cycle. Repeating in 3 seconds...");
  delay(3000);
}

// Helper function to display error messages
void displayErrorMessage(int8_t errorCode) {
  switch (errorCode) {
    case MAX7300_OK:
      Serial.println("No error");
      break;
    case MAX7300_ERR_WRONG_DEVICE:
      Serial.println("Wrong device detected");
      break;
    case MAX7300_ERR_COMM_FAIL:
      Serial.println("Communication failure");
      break;
    case MAX7300_ERR_INVALID_PIN:
      Serial.println("Invalid pin number");
      break;
    case MAX7300_ERR_INVALID_MODE:
      Serial.println("Invalid pin mode");
      break;
    default:
      Serial.print("Unknown error: ");
      Serial.println(errorCode);
  }
}

// Helper function to print register values
void printRegisterValue(uint8_t reg) {
  int16_t value = gpio.readRegister(reg);
  
  if (value < 0) {
    Serial.print("Failed to read register 0x");
    Serial.print(reg, HEX);
    Serial.print(": ");
    displayErrorMessage(value);
    return;
  }
  
  Serial.print("Register 0x");
  if (reg < 0x10) Serial.print("0"); // Pad with leading zero for single-digit hex values
  Serial.print(reg, HEX);
  Serial.print(": 0x");
  if (value < 0x10) Serial.print("0"); // Pad with leading zero for single-digit hex values
  Serial.print(value, HEX);
  Serial.print(" (Binary: ");
  
  // Print binary representation
  for (int i = 7; i >= 0; i--) {
    Serial.print((value & (1 << i)) ? '1' : '0');
    if (i == 4) Serial.print(' '); // Space in middle for readability
  }
  
  Serial.println(")");
}
