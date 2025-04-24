
/**
 * MAX7300 - I2C Port Expander Library
 * 
 * A comprehensive Arduino library for controlling the MAX7300 I2C GPIO Port Expander.
 * This library provides a user-friendly interface for all MAX7300 features including:
 * - GPIO configuration (input, output, high-impedance)
 * - Individual and batch pin operations
 * - Power management (normal/shutdown modes)
 * - Transition detection functionality
 * 
 * @author danel32
 * @version 1.0.0
 * @license MIT
 */

#ifndef MAX7300_h
#define MAX7300_h

#include <Arduino.h>
#include <Wire.h>

// Device address constants
#define MAX7300_BASE_ADDR 0x40 // Base I2C address (A2=A1=A0=0)
#define MAX7300_ADDR_MASK 0x0E // Address bits mask (A2, A1, A0)
#define MAX7300_ADDR_SHIFT 1   // Address bits start at bit position 1

// Register Addresses
#define MAX7300_REG_NO_OP         0x00
#define MAX7300_REG_CONFIG        0x04
#define MAX7300_REG_TRANSITION    0x06
#define MAX7300_REG_MASK          0x07
#define MAX7300_REG_PIN_CONFIG_BASE 0x09 // Configuration registers (0x09-0x0F)
#define MAX7300_REG_PIN_STATE_BASE  0x20 // Pin state registers (0x20-0x3F)

// Configuration Register Bits
#define MAX7300_CONFIG_SHUTDOWN   0x00
#define MAX7300_CONFIG_NORMAL     0x01
#define MAX7300_CONFIG_TRANSITION_ENABLE 0x02

// Power Management Modes
#define MAX7300_POWER_NORMAL      0x00
#define MAX7300_POWER_STANDBY     0x01
#define MAX7300_POWER_SHUTDOWN    0x02

// Pin Configuration Values
#define MAX7300_PIN_HIGHZ         0x00 // High-impedance input (default)
#define MAX7300_PIN_OUTPUT        0x01 // Output
#define MAX7300_PIN_INPUT         0x02 // Input
#define MAX7300_PIN_INPUT_PULLUP  0x03 // Input with internal pull-up

// Transition Detection 
#define MAX7300_TRANSITION_NONE   0x00 // No transition detection
#define MAX7300_TRANSITION_LOW    0x01 // Low level detection
#define MAX7300_TRANSITION_HIGH   0x02 // High level detection
#define MAX7300_TRANSITION_BOTH   0x03 // Both levels detection

// Pin-related constants
#define MAX7300_MIN_PIN           0
#define MAX7300_MAX_PIN           27
#define MAX7300_TOTAL_PINS        28
#define MAX7300_PINS_PER_REG      4

// Error codes
#define MAX7300_OK                 0
#define MAX7300_ERR_WRONG_DEVICE  -1
#define MAX7300_ERR_COMM_FAIL     -2
#define MAX7300_ERR_INVALID_PIN   -3
#define MAX7300_ERR_INVALID_MODE  -4
#define MAX7300_ERR_NOT_SUPPORTED -5
#define MAX7300_ERR_TIMEOUT       -6
#define MAX7300_ERR_EEPROM_FAIL   -7

// EEPROM storage constants
#define MAX7300_EEPROM_MAGIC      0xA537  // Magic number to verify config validity
#define MAX7300_EEPROM_VERSION    0x01    // Configuration version
#define MAX7300_EEPROM_SIZE       48      // Bytes needed for full configuration

/**
 * @brief Structure to hold a transition event
 */
struct MAX7300Event {
  uint8_t pin;     // The pin that triggered the event (4-27)
  uint8_t level;   // Current pin level (0 or 1)
};

/**
 * @brief Advanced interrupt callback type with event information
 */
typedef void (*MAX7300EventCallback)(MAX7300Event event);

/**
 * @brief Simple interrupt callback type with changed pins bitmask
 */
typedef void (*MAX7300InterruptCallback)(uint32_t changedPins);

/**
 * @brief Configuration structure for persistent settings
 */
struct MAX7300Config {
  uint16_t magic;                  // Magic number to verify config validity
  uint8_t version;                 // Configuration version
  uint8_t powerMode;               // Power management mode
  uint8_t pinModes[7];             // Pin configuration registers (0x09-0x0F)
  uint8_t defaultOutputs[28];      // Default output states for each pin
  uint8_t transitionMask;          // Transition detection mask
  uint8_t transitionTypes;         // Transition types (high/low)
  uint8_t checksum;                // Simple checksum for data integrity
};

class MAX7300 {
  public:
    /**
     * Constructor
     * 
     * @param addr The I2C address of the MAX7300 (determined by A2, A1, A0 pins)
     * @param wire Optional Wire interface for platforms with multiple I2C buses
     */
    MAX7300(uint8_t addr = MAX7300_BASE_ADDR, TwoWire *wire = &Wire);
    
    /**
     * Initialize the MAX7300 device
     * 
     * @param resetPins If true, all pins are set to high-impedance mode upon initialization
     * @param i2cSpeed Optional I2C clock frequency in Hz (default: 400000 Hz)
     * @return Error code (0 for success)
     */
    int8_t begin(bool resetPins = true, uint32_t i2cSpeed = 400000);
    
    /**
     * Reset the device to default state
     * 
     * @param hardReset If true, performs a full software reset
     * @return Error code (0 for success)
     */
    int8_t reset(bool hardReset = false);
    
    /**
     * Set device to normal operation mode
     * 
     * @return Error code (0 for success)
     */
    int8_t enable();
    
    /**
     * Set device to shutdown/low-power mode
     * 
     * @return Error code (0 for success)
     */
    int8_t disable();
    
    /**
     * Check if the device is in normal operation mode
     * 
     * @return true if device is enabled (normal mode), false if disabled (shutdown mode)
     */
    bool isEnabled();

    /**
     * Configure a pin's mode (input, output, high-impedance)
     * 
     * @param pin Pin number (0-27)
     * @param mode Mode (MAX7300_PIN_HIGHZ, MAX7300_PIN_OUTPUT, MAX7300_PIN_INPUT, MAX7300_PIN_INPUT_PULLUP)
     * @return Error code (0 for success)
     */
    int8_t pinMode(uint8_t pin, uint8_t mode);
    
    /**
     * Get the current mode of a pin
     * 
     * @param pin Pin number (0-27)
     * @return Pin mode or negative error code
     */
    int8_t getPinMode(uint8_t pin);
    
    /**
     * Configure multiple pins at once with the same mode
     * 
     * @param pinMask Bit mask of pins to configure (bit 0 = pin 0, etc.)
     * @param mode Mode to set for all selected pins
     * @return Error code (0 for success)
     */
    int8_t setMultiplePinMode(uint32_t pinMask, uint8_t mode);
    
    /**
     * Configure a range of consecutive pins with the same mode (optimized I2C operation)
     * 
     * @param startPin First pin in the range
     * @param endPin Last pin in the range (inclusive)
     * @param mode Mode to set for all pins in the range
     * @return Error code (0 for success)
     */
    int8_t setPinModeRange(uint8_t startPin, uint8_t endPin, uint8_t mode);
    
    /**
     * Read from a GPIO pin
     * 
     * @param pin Pin number (0-27)
     * @return Pin state (0 or 1) or negative error code
     */
    int8_t digitalRead(uint8_t pin);
    
    /**
     * Write to a GPIO pin
     * 
     * @param pin Pin number (0-27)
     * @param value Value to write (0 or 1)
     * @return Error code (0 for success)
     */
    int8_t digitalWrite(uint8_t pin, uint8_t value);
    
    /**
     * Read all pin states and return as a 32-bit value
     * 
     * @return 32-bit value where bits 0-27 represent pins 0-27
     */
    uint32_t digitalReadAll();
    
    /**
     * Write values to multiple pins at once
     * 
     * @param values 32-bit value where bits 0-27 represent pins 0-27
     * @param mask 32-bit mask where 1 means update that pin
     * @return Error code (0 for success)
     */
    int8_t digitalWriteAll(uint32_t values, uint32_t mask = 0xFFFFFFFF);
    
    /**
     * Read a range of consecutive pins efficiently (using auto-incrementing registers)
     * 
     * @param startPin First pin in the range
     * @param endPin Last pin in the range (inclusive)
     * @param values Pointer to array where values will be stored
     * @return Error code (0 for success)
     */
    int8_t digitalReadRange(uint8_t startPin, uint8_t endPin, uint8_t* values);
    
    /**
     * Write values to a range of consecutive pins efficiently
     * 
     * @param startPin First pin in the range
     * @param endPin Last pin in the range (inclusive)
     * @param values Array of values to write
     * @return Error code (0 for success)
     */
    int8_t digitalWriteRange(uint8_t startPin, uint8_t endPin, const uint8_t* values);
    
    /**
     * Enable transition detection functionality
     * 
     * @return Error code (0 for success)
     */
    int8_t enableTransitionDetection();
    
    /**
     * Disable transition detection functionality
     * 
     * @return Error code (0 for success)
     */
    int8_t disableTransitionDetection();
    
    /**
     * Check if transition detection is enabled
     * 
     * @return true if enabled, false if disabled
     */
    bool isTransitionDetectionEnabled();
    
    /**
     * Configure transition detection for a specific pin
     * 
     * @param pin Pin number (4-27, pins 0-3 don't support transition detection)
     * @param mode Detection mode (NONE, LOW, HIGH, or BOTH)
     * @return Error code (0 for success)
     */
    int8_t configureTransitionDetection(uint8_t pin, uint8_t mode);
    
    /**
     * Configure transition detection for multiple pins at once
     * 
     * @param pinMask Bit mask of pins to configure (bits 4-27 only)
     * @param mode Detection mode for all selected pins
     * @return Error code (0 for success)
     */
    int8_t configureMultipleTransitionDetection(uint32_t pinMask, uint8_t mode);
    
    /**
     * Get the transition detection configuration for a pin
     * 
     * @param pin Pin number (4-27)
     * @return Transition mode or negative error code
     */
    int8_t getTransitionDetection(uint8_t pin);
    
    /**
     * Check if a transition has been detected
     * 
     * @return true if a transition was detected, false otherwise
     */
    bool transitionDetected();
    
    /**
     * Read the transition event register to determine which pins triggered an event
     * 
     * @return 32-bit value where bits represent which pins had transitions
     */
    uint32_t getTransitionEvents();
    
    /**
     * Clear transition events
     * 
     * @return Error code (0 for success)
     */
    int8_t clearTransitionEvents();
    
    /**
     * Set up interrupt handling via callback function
     * The provided function will be called when a transition is detected
     * 
     * @param interruptPin Arduino pin connected to MAX7300 INT output
     * @param callback Function to call when interrupt occurs
     * @return Error code (0 for success)
     */
    int8_t attachInterrupt(uint8_t interruptPin, MAX7300InterruptCallback callback);
    
    /**
     * Remove interrupt handler
     * 
     * @return Error code (0 for success)
     */
    int8_t detachInterrupt();
    
    /**
     * Check interrupt status manually (useful in polling mode)
     * 
     * @return true if interrupt occurred, false otherwise
     */
    bool checkInterrupt();
    
    /**
     * Direct register access for advanced users
     * 
     * @param reg Register address
     * @return Register value or negative error code
     */
    int16_t readRegister(uint8_t reg);
    
    /**
     * Direct register write for advanced users
     * 
     * @param reg Register address
     * @param value Value to write
     * @return Error code (0 for success)
     */
    int8_t writeRegister(uint8_t reg, uint8_t value);
    
    /**
     * Read multiple consecutive registers efficiently
     * 
     * @param startReg First register address
     * @param values Pointer to array where values will be stored
     * @param count Number of registers to read
     * @return Error code (0 for success)
     */
    int8_t readRegisters(uint8_t startReg, uint8_t* values, uint8_t count);
    
    /**
     * Write to multiple consecutive registers efficiently
     * 
     * @param startReg First register address
     * @param values Array of values to write
     * @param count Number of registers to write
     * @return Error code (0 for success)
     */
    int8_t writeRegisters(uint8_t startReg, const uint8_t* values, uint8_t count);
    
    /**
     * Get the last error that occurred
     * 
     * @return Error code
     */
    int8_t getLastError() const;
    
    /**
     * Set I2C communication timeout
     * 
     * @param timeout Timeout in milliseconds
     */
    void setTimeout(uint16_t timeout);
    
    /**
     * Get current I2C timeout setting
     * 
     * @return Timeout in milliseconds
     */
    uint16_t getTimeout() const;
    
    /**
     * Get detailed error message for the last error
     * 
     * @return Error message string
     */
    const char* getErrorMessage() const;
    
    /**
     * @brief Set the power mode of the device
     * 
     * @param mode Power mode (MAX7300_POWER_NORMAL, MAX7300_POWER_STANDBY, MAX7300_POWER_SHUTDOWN)
     * @return Error code (0 for success)
     */
    int8_t setPowerMode(uint8_t mode);
    
    /**
     * @brief Get the current power mode of the device
     * 
     * @return Current power mode
     */
    uint8_t getPowerMode();
    
    /**
     * @brief Enter standby mode (reduced power while maintaining pin states)
     * This mode preserves pin configuration but reduces power consumption
     * 
     * @param preserveOutputs If true, current output states are preserved when returning to normal mode
     * @return Error code (0 for success)
     */
    int8_t standby(bool preserveOutputs = true);
    
    /**
     * @brief Enter shutdown mode (minimum power consumption)
     * This mode resets all pins to high-impedance inputs and minimizes power
     * 
     * @return Error code (0 for success)
     */
    int8_t shutdown();
    
    /**
     * @brief Wake the device from standby or shutdown mode
     * 
     * @param restoreState If true, restore previous pin states
     * @return Error code (0 for success)
     */
    int8_t wakeup(bool restoreState = true);
    
    /**
     * @brief Get the raw transition flag from the configuration register
     * This is more efficient than getTransitionEvents() as it only reads one register
     * 
     * @return true if transition flag is set, false otherwise
     */
    bool getRawTransitionFlag();
    
    /**
     * @brief Set up event-based interrupt handling
     * This provides more detailed information about the interrupt events
     * 
     * @param interruptPin Arduino pin connected to MAX7300 INT output
     * @param callback Function to call when interrupt occurs
     * @return Error code (0 for success)
     */
    int8_t attachEventInterrupt(uint8_t interruptPin, MAX7300EventCallback callback);
    
    /**
     * @brief Poll for transition events with detailed information
     * Use this in loop() when not using interrupts
     * 
     * @param event Pointer to MAX7300Event structure to fill with event details
     * @return true if an event was detected, false otherwise
     */
    bool pollEvent(MAX7300Event* event);
    
    /**
     * @brief Save current configuration to Arduino EEPROM
     * 
     * @param eepromAddress Starting address in EEPROM
     * @return Error code (0 for success)
     */
    int8_t saveConfiguration(uint16_t eepromAddress = 0);
    
    /**
     * @brief Load configuration from Arduino EEPROM
     * 
     * @param eepromAddress Starting address in EEPROM
     * @param applyConfig If true, apply the loaded configuration immediately
     * @return Error code (0 for success)
     */
    int8_t loadConfiguration(uint16_t eepromAddress = 0, bool applyConfig = true);
    
    /**
     * @brief Save current configuration to external EEPROM or flash
     * 
     * @param writeCallback Function to call to write data
     * @return Error code (0 for success)
     */
    int8_t saveConfigurationExt(bool (*writeCallback)(uint16_t address, uint8_t* data, uint16_t length));
    
    /**
     * @brief Load configuration from external EEPROM or flash
     * 
     * @param readCallback Function to call to read data
     * @param applyConfig If true, apply the loaded configuration immediately
     * @return Error code (0 for success)
     */
    int8_t loadConfigurationExt(bool (*readCallback)(uint16_t address, uint8_t* data, uint16_t length), bool applyConfig = true);
    
  private:
    TwoWire *_wire;        // I2C interface
    uint8_t _address;      // I2C address
    int8_t _lastError;     // Last error code
    uint16_t _timeout;     // I2C timeout in milliseconds
    bool _initialized;     // Whether the device has been initialized
    volatile bool _interruptOccurred; // Flag set by ISR
    uint8_t _interruptPin; // Arduino pin for interrupt
    MAX7300InterruptCallback _interruptCallback; // User-provided callback function
    MAX7300EventCallback _eventCallback;         // Event-based callback function
    uint8_t _powerMode;    // Current power mode
    
    // Pin configuration cache to reduce I2C traffic
    uint8_t _pinConfigCache[7]; // Cache for config registers 0x09-0x0F
    bool _pinConfigCacheValid;  // Whether the cache is valid
    
    // Output state cache for power management
    uint8_t _outputStateCache[MAX7300_TOTAL_PINS]; // Cache for output states
    bool _outputStateCacheValid;                   // Whether the cache is valid

    // Current configuration structure
    MAX7300Config _config;  // Current configuration
    
    // Helper methods
    bool validPin(uint8_t pin);
    bool validMode(uint8_t mode);
    bool validTransitionMode(uint8_t mode);
    bool validPowerMode(uint8_t mode);
    uint8_t getConfigRegister(uint8_t pin);
    uint8_t getPinShift(uint8_t pin);
    int8_t updatePinConfigCache();
    int8_t updateOutputStateCache();
    int8_t writePinConfigRegister(uint8_t reg, uint8_t value);
    int8_t setPinMode(uint8_t pin, uint8_t mode, bool updateCache);
    uint8_t calculateChecksum(uint8_t* data, uint8_t length);
    int8_t applyConfiguration(const MAX7300Config& config);
    bool getEventDetails(MAX7300Event* event);
    
    // Static function for interrupt handling
    static void handleInterrupt(void* instance);
};

#endif

