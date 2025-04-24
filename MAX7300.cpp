/**
 * MAX7300 - I2C Port Expander Library Implementation
 */

#include "MAX7300.h"

// Interrupt handler map (used for static interrupt callback)
static MAX7300* _instanceMap[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
static uint8_t _instanceCount = 0;

// Static interrupt handler that redirects to the appropriate instance
void interruptHandler() {
  for (uint8_t i = 0; i < _instanceCount; i++) {
    if (_instanceMap[i] && digitalRead(_instanceMap[i]->_interruptPin) == LOW) {
      _instanceMap[i]->_interruptOccurred = true;
      if (_instanceMap[i]->_interruptCallback) {
        _instanceMap[i]->_interruptCallback(_instanceMap[i]->getTransitionEvents());
      } else if (_instanceMap[i]->_eventCallback) {
        MAX7300Event event;
        if (_instanceMap[i]->getEventDetails(&event)) {
          _instanceMap[i]->_eventCallback(event);
        }
      }
    }
  }
}

MAX7300::MAX7300(uint8_t addr, TwoWire *wire) {
  // Store the I2C address and Wire interface
  _address = addr;
  _wire = wire;
  _lastError = MAX7300_OK;
  _timeout = 1000; // 1 second default timeout
  _initialized = false;
  _pinConfigCacheValid = false;
  _outputStateCacheValid = false;
  _interruptOccurred = false;
  _interruptPin = 0xFF; // Invalid pin to indicate no interrupt configured
  _interruptCallback = nullptr;
  _eventCallback = nullptr;
  _powerMode = MAX7300_POWER_NORMAL;
  
  // Initialize cache
  for (int i = 0; i < 7; i++) {
    _pinConfigCache[i] = 0;
  }
  
  for (int i = 0; i < MAX7300_TOTAL_PINS; i++) {
    _outputStateCache[i] = 0;
  }
  
  // Initialize configuration structure
  memset(&_config, 0, sizeof(_config));
  _config.magic = MAX7300_EEPROM_MAGIC;
  _config.version = MAX7300_EEPROM_VERSION;
}

int8_t MAX7300::begin(bool resetPins, uint32_t i2cSpeed) {
  // Initialize I2C communication
  _wire->begin();
  
  // Set I2C clock speed if specified
  if (i2cSpeed > 0) {
    _wire->setClock(i2cSpeed);
  }
  
  // Check if the device is responding
  _wire->beginTransmission(_address);
  if (_wire->endTransmission() != 0) {
    _lastError = MAX7300_ERR_COMM_FAIL;
    return _lastError;
  }
  
  // Reset device if requested
  if (resetPins) {
    // Reset all pins to high-impedance state (default)
    for (uint8_t reg = MAX7300_REG_PIN_CONFIG_BASE; reg <= MAX7300_REG_PIN_CONFIG_BASE + 6; reg++) {
      if (writeRegister(reg, 0x00) != MAX7300_OK) {
        return _lastError;
      }
    }
  }
  
  // Initialize the configuration cache
  if (updatePinConfigCache() != MAX7300_OK) {
    return _lastError;
  }
  
  // Set device to normal operating mode
  int8_t result = enable();
  if (result == MAX7300_OK) {
    _initialized = true;
    _powerMode = MAX7300_POWER_NORMAL;
  }
  
  return result;
}

int8_t MAX7300::reset(bool hardReset) {
  // Return to default configuration
  if (hardReset) {
    // Set shutdown mode first
    if (disable() != MAX7300_OK) {
      return _lastError;
    }
    
    // Reset all configuration registers
    for (uint8_t reg = MAX7300_REG_CONFIG; reg <= MAX7300_REG_PIN_CONFIG_BASE + 6; reg++) {
      if (writeRegister(reg, 0x00) != MAX7300_OK) {
        return _lastError;
      }
    }
    
    // Invalidate cache
    _pinConfigCacheValid = false;
    _outputStateCacheValid = false;
    _powerMode = MAX7300_POWER_SHUTDOWN;
  }
  
  // Set all pins to high-impedance state
  for (uint8_t pin = MAX7300_MIN_PIN; pin <= MAX7300_MAX_PIN; pin++) {
    if (pinMode(pin, MAX7300_PIN_HIGHZ) != MAX7300_OK) {
      return _lastError;
    }
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::enable() {
  // Set normal operating mode (bit 0 = 1)
  int8_t result = writeRegister(MAX7300_REG_CONFIG, MAX7300_CONFIG_NORMAL);
  if (result == MAX7300_OK) {
    _powerMode = MAX7300_POWER_NORMAL;
  }
  return result;
}

int8_t MAX7300::disable() {
  // Set shutdown mode (bit 0 = 0)
  int8_t result = writeRegister(MAX7300_REG_CONFIG, MAX7300_CONFIG_SHUTDOWN);
  if (result == MAX7300_OK) {
    _powerMode = MAX7300_POWER_SHUTDOWN;
  }
  return result;
}

bool MAX7300::isEnabled() {
  int16_t config = readRegister(MAX7300_REG_CONFIG);
  return (config >= 0) && (config & MAX7300_CONFIG_NORMAL);
}

int8_t MAX7300::pinMode(uint8_t pin, uint8_t mode) {
  return setPinMode(pin, mode, true);
}

int8_t MAX7300::setPinMode(uint8_t pin, uint8_t mode, bool updateCache) {
  // Validate pin and mode
  if (!validPin(pin)) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  if (!validMode(mode)) {
    _lastError = MAX7300_ERR_INVALID_MODE;
    return _lastError;
  }
  
  // Special case: pins 0-3 don't support input modes in some chip variants
  // But we'll allow it, as it depends on the specific model
  
  // Get the configuration register and bit positions for this pin
  uint8_t reg = getConfigRegister(pin);
  uint8_t shift = getPinShift(pin);
  
  // First, ensure the cache is valid
  if (!_pinConfigCacheValid && updatePinConfigCache() != MAX7300_OK) {
    return _lastError;
  }
  
  // Update the pin configuration in our cache
  uint8_t currentConfig = _pinConfigCache[reg - MAX7300_REG_PIN_CONFIG_BASE];
  uint8_t newConfig = (currentConfig & ~(0x03 << shift)) | (mode << shift);
  
  // Only write if the configuration actually changed
  if (currentConfig != newConfig) {
    // Write the updated configuration to the device
    int8_t result = writePinConfigRegister(reg, newConfig);
    
    // Update cache if successful and requested
    if (result == MAX7300_OK && updateCache) {
      _pinConfigCache[reg - MAX7300_REG_PIN_CONFIG_BASE] = newConfig;
    }
    
    return result;
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::getPinMode(uint8_t pin) {
  if (!validPin(pin)) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  // Get register and bit position
  uint8_t reg = getConfigRegister(pin);
  uint8_t shift = getPinShift(pin);
  
  // Ensure cache is valid
  if (!_pinConfigCacheValid && updatePinConfigCache() != MAX7300_OK) {
    return _lastError;
  }
  
  // Read from cache
  uint8_t config = _pinConfigCache[reg - MAX7300_REG_PIN_CONFIG_BASE];
  return (config >> shift) & 0x03;
}

int8_t MAX7300::setMultiplePinMode(uint32_t pinMask, uint8_t mode) {
  if (!validMode(mode)) {
    _lastError = MAX7300_ERR_INVALID_MODE;
    return _lastError;
  }
  
  // Ensure cache is valid
  if (!_pinConfigCacheValid && updatePinConfigCache() != MAX7300_OK) {
    return _lastError;
  }

  // Create modified configurations
  uint8_t newConfigs[7];
  for (uint8_t i = 0; i < 7; i++) {
    newConfigs[i] = _pinConfigCache[i];
  }
  
  // Update configuration registers for each pin in the mask
  for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
    if (pinMask & (1UL << pin)) {
      uint8_t reg = getConfigRegister(pin);
      uint8_t regIndex = reg - MAX7300_REG_PIN_CONFIG_BASE;
      uint8_t shift = getPinShift(pin);
      
      // Update configuration in our working copy
      newConfigs[regIndex] = (newConfigs[regIndex] & ~(0x03 << shift)) | (mode << shift);
    }
  }
  
  // Now write only the registers that have changed
  for (uint8_t i = 0; i < 7; i++) {
    if (newConfigs[i] != _pinConfigCache[i]) {
      int8_t result = writePinConfigRegister(MAX7300_REG_PIN_CONFIG_BASE + i, newConfigs[i]);
      if (result != MAX7300_OK) {
        return result;
      }
      
      // Update cache
      _pinConfigCache[i] = newConfigs[i];
    }
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::setPinModeRange(uint8_t startPin, uint8_t endPin, uint8_t mode) {
  // Validate parameters
  if (!validPin(startPin) || !validPin(endPin) || startPin > endPin) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  if (!validMode(mode)) {
    _lastError = MAX7300_ERR_INVALID_MODE;
    return _lastError;
  }
  
  // Build a mask for the specified pin range
  uint32_t mask = 0;
  for (uint8_t pin = startPin; pin <= endPin; pin++) {
    mask |= (1UL << pin);
  }
  
  // Use the optimized multiple pin mode function
  return setMultiplePinMode(mask, mode);
}

int8_t MAX7300::digitalRead(uint8_t pin) {
  if (!validPin(pin)) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  // Get pin data from the appropriate register
  int16_t regVal = readRegister(MAX7300_REG_PIN_STATE_BASE + pin);
  if (regVal < 0) {
    return regVal; // Return error code
  }
  
  return regVal & 0x01;
}

int8_t MAX7300::digitalWrite(uint8_t pin, uint8_t value) {
  if (!validPin(pin)) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  // Make sure the pin is configured as an output
  int8_t mode = getPinMode(pin);
  if (mode < 0) {
    return mode; // Return error code
  }
  
  if (mode != MAX7300_PIN_OUTPUT) {
    // Auto-configure as output
    int8_t result = pinMode(pin, MAX7300_PIN_OUTPUT);
    if (result != MAX7300_OK) {
      return result;
    }
  }
  
  // Write the value to the pin
  return writeRegister(MAX7300_REG_PIN_STATE_BASE + pin, value ? 0x01 : 0x00);
}

uint32_t MAX7300::digitalReadAll() {
  uint32_t result = 0;
  
  // Read all pin state registers (pins 0-27)
  uint8_t states[MAX7300_MAX_PIN + 1];
  int8_t err = readRegisters(MAX7300_REG_PIN_STATE_BASE, states, MAX7300_MAX_PIN + 1);
  
  if (err != MAX7300_OK) {
    _lastError = err;
    return 0;
  }
  
  // Construct the 32-bit result
  for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
    if (states[pin] & 0x01) {
      result |= (1UL << pin);
    }
  }
  
  return result;
}

int8_t MAX7300::digitalWriteAll(uint32_t values, uint32_t mask) {
  // Optimize the case where mask covers consecutive pins:
  // Check if mask is a consecutive set of 1s
  uint32_t tempMask = mask;
  uint8_t firstBit = 0xFF;
  uint8_t lastBit = 0;
  
  // Find first and last set bit
  for (uint8_t i = 0; i <= MAX7300_MAX_PIN; i++) {
    if (tempMask & (1UL << i)) {
      if (firstBit == 0xFF) firstBit = i;
      lastBit = i;
      tempMask &= ~(1UL << i); // Clear this bit
    }
  }
  
  // If mask is a consecutive range and tempMask is now 0, use the range method
  if (tempMask == 0 && firstBit != 0xFF) {
    uint8_t buffer[lastBit - firstBit + 1];
    
    // Prepare the values
    for (uint8_t i = 0; i < (lastBit - firstBit + 1); i++) {
      buffer[i] = (values & (1UL << (firstBit + i))) ? 0x01 : 0x00;
    }
    
    // Write the values using the range method
    return digitalWriteRange(firstBit, lastBit, buffer);
  }
  
  // If not a consecutive range, update each pin as needed
  for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
    if (mask & (1UL << pin)) {
      int8_t result = digitalWrite(pin, (values & (1UL << pin)) ? 1 : 0);
      if (result != MAX7300_OK) {
        return result;
      }
    }
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::digitalReadRange(uint8_t startPin, uint8_t endPin, uint8_t* values) {
  // Validate parameters
  if (!validPin(startPin) || !validPin(endPin) || startPin > endPin) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  if (!values) {
    _lastError = MAX7300_ERR_INVALID_PIN;  // Using this error code for null pointer
    return _lastError;
  }
  
  // Use register reading to read a consecutive block
  return readRegisters(MAX7300_REG_PIN_STATE_BASE + startPin, values, endPin - startPin + 1);
}

int8_t MAX7300::digitalWriteRange(uint8_t startPin, uint8_t endPin, const uint8_t* values) {
  // Validate parameters
  if (!validPin(startPin) || !validPin(endPin) || startPin > endPin) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  if (!values) {
    _lastError = MAX7300_ERR_INVALID_PIN;  // Using this error code for null pointer
    return _lastError;
  }
  
  // Make sure all pins are configured as outputs
  for (uint8_t pin = startPin; pin <= endPin; pin++) {
    int8_t mode = getPinMode(pin);
    if (mode < 0) {
      return mode; // Return error code
    }
    
    if (mode != MAX7300_PIN_OUTPUT) {
      // Auto-configure as output
      int8_t result = pinMode(pin, MAX7300_PIN_OUTPUT);
      if (result != MAX7300_OK) {
        return result;
      }
    }
  }
  
  // Use register writing to write a consecutive block
  return writeRegisters(MAX7300_REG_PIN_STATE_BASE + startPin, values, endPin - startPin + 1);
}

int8_t MAX7300::enableTransitionDetection() {
  int16_t config = readRegister(MAX7300_REG_CONFIG);
  if (config < 0) {
    return _lastError;
  }
  
  return writeRegister(MAX7300_REG_CONFIG, config | MAX7300_CONFIG_TRANSITION_ENABLE);
}

int8_t MAX7300::disableTransitionDetection() {
  int16_t config = readRegister(MAX7300_REG_CONFIG);
  if (config < 0) {
    return _lastError;
  }
  
  return writeRegister(MAX7300_REG_CONFIG, config & ~MAX7300_CONFIG_TRANSITION_ENABLE);
}

bool MAX7300::isTransitionDetectionEnabled() {
  int16_t config = readRegister(MAX7300_REG_CONFIG);
  return (config >= 0) && (config & MAX7300_CONFIG_TRANSITION_ENABLE);
}

int8_t MAX7300::configureTransitionDetection(uint8_t pin, uint8_t mode) {
  // Transition detection only works for pins 4-27
  if (pin < 4 || pin > MAX7300_MAX_PIN) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  if (!validTransitionMode(mode)) {
    _lastError = MAX7300_ERR_INVALID_MODE;
    return _lastError;
  }
  
  // Read current mask register
  int16_t mask = readRegister(MAX7300_REG_MASK);
  if (mask < 0) {
    return _lastError;
  }
  
  // Calculate bit position for this pin
  uint8_t bit = pin - 4;
  
  // Update mask based on the desired mode
  uint8_t newMask;
  switch (mode) {
    case MAX7300_TRANSITION_NONE:
      newMask = mask & ~(1 << bit);
      break;
    case MAX7300_TRANSITION_LOW:
    case MAX7300_TRANSITION_HIGH:
    case MAX7300_TRANSITION_BOTH:
      newMask = mask | (1 << bit);
      break;
    default:
      _lastError = MAX7300_ERR_INVALID_MODE;
      return _lastError;
  }
  
  // Write updated mask
  int8_t result = writeRegister(MAX7300_REG_MASK, newMask);
  if (result != MAX7300_OK) {
    return result;
  }
  
  // For transition type (high/low/both), we need to set the transition register
  if (mode != MAX7300_TRANSITION_NONE) {
    // Read current transition register
    int16_t trans = readRegister(MAX7300_REG_TRANSITION);
    if (trans < 0) {
      return _lastError;
    }
    
    // Set/clear bit based on mode (bit set = high transition, bit clear = low transition)
    if (mode == MAX7300_TRANSITION_HIGH || mode == MAX7300_TRANSITION_BOTH) {
      trans |= (1 << bit);
    } else {
      trans &= ~(1 << bit);
    }
    
    // Write updated transition register
    result = writeRegister(MAX7300_REG_TRANSITION, trans);
  }
  
  return result;
}

int8_t MAX7300::configureMultipleTransitionDetection(uint32_t pinMask, uint8_t mode) {
  if (!validTransitionMode(mode)) {
    _lastError = MAX7300_ERR_INVALID_MODE;
    return _lastError;
  }
  
  // Only pins 4-27 support transition detection
  pinMask &= 0x0FFFFF0; // Mask out pins 0-3 and 28-31
  
  // Read current mask and transition registers
  int16_t maskReg = readRegister(MAX7300_REG_MASK);
  if (maskReg < 0) {
    return _lastError;
  }
  
  int16_t transReg = readRegister(MAX7300_REG_TRANSITION);
  if (transReg < 0) {
    return _lastError;
  }
  
  // Update mask register based on mode
  uint8_t newMask = maskReg;
  uint8_t newTrans = transReg;
  
  for (uint8_t pin = 4; pin <= MAX7300_MAX_PIN; pin++) {
    if (pinMask & (1UL << pin)) {
      uint8_t bit = pin - 4;
      
      switch (mode) {
        case MAX7300_TRANSITION_NONE:
          newMask &= ~(1 << bit);
          break;
        case MAX7300_TRANSITION_LOW:
          newMask |= (1 << bit);
          newTrans &= ~(1 << bit); // Clear bit for low transition
          break;
        case MAX7300_TRANSITION_HIGH:
          newMask |= (1 << bit);
          newTrans |= (1 << bit);  // Set bit for high transition
          break;
        case MAX7300_TRANSITION_BOTH:
          newMask |= (1 << bit);
          // For "both" we use high transition setting, application needs to check both states
          newTrans |= (1 << bit);
          break;
      }
    }
  }
  
  // Now write only the registers that have changed
  int8_t result = MAX7300_OK;
  if (newMask != maskReg) {
    result = writeRegister(MAX7300_REG_MASK, newMask);
    if (result != MAX7300_OK) {
      return result;
    }
  }
  
  if (mode != MAX7300_TRANSITION_NONE && newTrans != transReg) {
    result = writeRegister(MAX7300_REG_TRANSITION, newTrans);
  }
  
  return result;
}

int8_t MAX7300::getTransitionDetection(uint8_t pin) {
  // Transition detection only works for pins 4-27
  if (pin < 4 || pin > MAX7300_MAX_PIN) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  uint8_t bit = pin - 4;
  
  // Read mask register to see if this pin has transition detection enabled
  int16_t mask = readRegister(MAX7300_REG_MASK);
  if (mask < 0) {
    return _lastError;
  }
  
  if (!(mask & (1 << bit))) {
    // Transition detection disabled for this pin
    return MAX7300_TRANSITION_NONE;
  }
  
  // Read transition register to determine type
  int16_t trans = readRegister(MAX7300_REG_TRANSITION);
  if (trans < 0) {
    return _lastError;
  }
  
  // Determine mode based on transition bit
  if (trans & (1 << bit)) {
    return MAX7300_TRANSITION_HIGH;
  } else {
    return MAX7300_TRANSITION_LOW;
  }
  
  // Note: MAX7300 doesn't directly support BOTH mode, it's emulated in software
}

bool MAX7300::transitionDetected() {
  // Read config register - bit 1 indicates a transition was detected
  int16_t config = readRegister(MAX7300_REG_CONFIG);
  return (config >= 0) && (config & 0x02);
}

uint32_t MAX7300::getTransitionEvents() {
  uint32_t events = 0;
  
  // Need to read pin states to determine which pins had transitions
  // Transition flag in config tells us there was a transition, but not which pin
  
  // First check if there was a transition at all
  if (!transitionDetected()) {
    return 0;
  }
  
  // To determine which pins had transitions, we need to:
  // 1. Read the current pin states
  // 2. Compare with the expected states (based on transition configuration)
  
  // Read all pin states
  uint32_t currentStates = digitalReadAll();
  
  // Read mask register to see which pins can trigger transitions
  int16_t mask = readRegister(MAX7300_REG_MASK);
  if (mask < 0) {
    return 0;
  }
  
  // Read transition register to know which level triggers each pin
  int16_t trans = readRegister(MAX7300_REG_TRANSITION);
  if (trans < 0) {
    return 0;
  }
  
  // For each pin with transition detection enabled
  for (uint8_t pin = 4; pin <= MAX7300_MAX_PIN; pin++) {
    uint8_t bit = pin - 4;
    
    // If this pin has transition detection enabled
    if (mask & (1 << bit)) {
      bool pinHigh = currentStates & (1UL << pin);
      bool highTrigger = trans & (1 << bit);
      
      // If the pin state matches its trigger condition, it likely caused the transition
      if ((pinHigh && highTrigger) || (!pinHigh && !highTrigger)) {
        events |= (1UL << pin);
      }
    }
  }
  
  return events;
}

int8_t MAX7300::clearTransitionEvents() {
  // Reading the configuration clears the transition flag
  return (readRegister(MAX7300_REG_CONFIG) >= 0) ? MAX7300_OK : _lastError;
}

int8_t MAX7300::attachInterrupt(uint8_t interruptPin, MAX7300InterruptCallback callback) {
  // Validate parameters
  if (interruptPin >= NUM_DIGITAL_PINS) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  if (callback == nullptr) {
    _lastError = MAX7300_ERR_INVALID_MODE; // Reuse this error for null callback
    return _lastError;
  }
  
  // Store parameters
  _interruptPin = interruptPin;
  _interruptCallback = callback;
  _interruptOccurred = false;
  
  // Enable transition detection
  int8_t result = enableTransitionDetection();
  if (result != MAX7300_OK) {
    return result;
  }
  
  // Setup Arduino interrupt
  pinMode(_interruptPin, INPUT_PULLUP);
  
  // Add this instance to the map
  if (_instanceCount < 5) {
    _instanceMap[_instanceCount++] = this;
    ::attachInterrupt(digitalPinToInterrupt(_interruptPin), interruptHandler, FALLING);
  } else {
    _lastError = MAX7300_ERR_NOT_SUPPORTED; // Too many instances
    return _lastError;
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::detachInterrupt() {
  if (_interruptPin != 0xFF) {
    // Detach hardware interrupt
    ::detachInterrupt(digitalPinToInterrupt(_interruptPin));
    
    // Remove this instance from the map
    for (uint8_t i = 0; i < _instanceCount; i++) {
      if (_instanceMap[i] == this) {
        // Shift remaining instances down
        for (uint8_t j = i; j < _instanceCount - 1; j++) {
          _instanceMap[j] = _instanceMap[j + 1];
        }
        _instanceMap[_instanceCount - 1] = nullptr;
        _instanceCount--;
        break;
      }
    }
    
    // Reset interrupt state
    _interruptPin = 0xFF;
    _interruptCallback = nullptr;
    _interruptOccurred = false;
    
    // Disable transition detection if not needed for other functions
    disableTransitionDetection();
  }
  
  return MAX7300_OK;
}

bool MAX7300::checkInterrupt() {
  if (_interruptOccurred) {
    _interruptOccurred = false;
    return true;
  }
  
  return false;
}

int8_t MAX7300::attachEventInterrupt(uint8_t interruptPin, MAX7300EventCallback callback) {
  // Validate parameters
  if (interruptPin >= NUM_DIGITAL_PINS) {
    _lastError = MAX7300_ERR_INVALID_PIN;
    return _lastError;
  }
  
  if (callback == nullptr) {
    _lastError = MAX7300_ERR_INVALID_MODE; // Reuse this error for null callback
    return _lastError;
  }
  
  // Store parameters
  _interruptPin = interruptPin;
  _eventCallback = callback;
  _interruptCallback = nullptr; // Use event callback instead of basic callback
  _interruptOccurred = false;
  
  // Enable transition detection
  int8_t result = enableTransitionDetection();
  if (result != MAX7300_OK) {
    return result;
  }
  
  // Setup Arduino interrupt
  pinMode(_interruptPin, INPUT_PULLUP);
  
  // Add this instance to the map
  if (_instanceCount < 5) {
    _instanceMap[_instanceCount++] = this;
    ::attachInterrupt(digitalPinToInterrupt(_interruptPin), interruptHandler, FALLING);
  } else {
    _lastError = MAX7300_ERR_NOT_SUPPORTED; // Too many instances
    return _lastError;
  }
  
  return MAX7300_OK;
}

bool MAX7300::pollEvent(MAX7300Event* event) {
  if (!event) {
    _lastError = MAX7300_ERR_INVALID_PIN; // Reuse this error for null pointer
    return false;
  }
  
  // Check if transition flag is set
  if (!getRawTransitionFlag()) {
    return false;
  }
  
  // Get details of the event
  bool result = getEventDetails(event);
  
  // Clear the transition flag
  clearTransitionEvents();
  
  return result;
}

bool MAX7300::getRawTransitionFlag() {
  // Read the transition flag directly from the configuration register
  // This is much more efficient than checking all pins when you just need to know
  // if any transition occurred
  int16_t config = readRegister(MAX7300_REG_CONFIG);
  return (config >= 0) && (config & 0x02);
}

bool MAX7300::getEventDetails(MAX7300Event* event) {
  if (!event) {
    return false;
  }
  
  // Read the transition mask register to see which pins can trigger transitions
  int16_t mask = readRegister(MAX7300_REG_MASK);
  if (mask < 0) {
    return false;
  }
  
  // Read the transition register to know which level triggers each pin
  int16_t trans = readRegister(MAX7300_REG_TRANSITION);
  if (trans < 0) {
    return false;
  }
  
  // Read all pin states
  uint32_t currentStates = digitalReadAll();
  
  // Find the first pin with a transition
  for (uint8_t pin = 4; pin <= MAX7300_MAX_PIN; pin++) {
    uint8_t bit = pin - 4;
    
    // If this pin has transition detection enabled
    if (mask & (1 << bit)) {
      bool pinHigh = currentStates & (1UL << pin);
      bool highTrigger = trans & (1 << bit);
      
      // If the pin state matches its trigger condition, it likely caused the transition
      if ((pinHigh && highTrigger) || (!pinHigh && !highTrigger)) {
        event->pin = pin;
        event->level = pinHigh ? 1 : 0;
        return true;
      }
    }
  }
  
  return false;
}

int8_t MAX7300::setPowerMode(uint8_t mode) {
  if (!validPowerMode(mode)) {
    _lastError = MAX7300_ERR_INVALID_MODE;
    return _lastError;
  }
  
  if (mode == _powerMode) {
    return MAX7300_OK; // Already in requested mode
  }
  
  switch (mode) {
    case MAX7300_POWER_NORMAL:
      return wakeup(true);
    
    case MAX7300_POWER_STANDBY:
      return standby(true);
    
    case MAX7300_POWER_SHUTDOWN:
      return shutdown();
      
    default:
      _lastError = MAX7300_ERR_INVALID_MODE;
      return _lastError;
  }
}

uint8_t MAX7300::getPowerMode() {
  return _powerMode;
}

int8_t MAX7300::standby(bool preserveOutputs) {
  // Cache current output states if needed
  if (preserveOutputs && !_outputStateCacheValid) {
    if (updateOutputStateCache() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  // Disable transition detection to save power
  if (isTransitionDetectionEnabled()) {
    if (disableTransitionDetection() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  // Keep device enabled but minimize current consumption
  // In standby mode, we keep the chip enabled but reduce functionality
  
  int16_t config = readRegister(MAX7300_REG_CONFIG);
  if (config < 0) {
    return _lastError;
  }
  
  // Ensure the normal bit is set and transition detection is disabled
  int8_t result = writeRegister(MAX7300_REG_CONFIG, MAX7300_CONFIG_NORMAL);
  if (result == MAX7300_OK) {
    _powerMode = MAX7300_POWER_STANDBY;
  }
  
  return result;
}

int8_t MAX7300::shutdown() {
  // Cache all pin configurations and output states before shutdown
  if (!_pinConfigCacheValid) {
    if (updatePinConfigCache() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  if (!_outputStateCacheValid) {
    if (updateOutputStateCache() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  // Set device to shutdown mode (bit 0 = 0)
  int8_t result = writeRegister(MAX7300_REG_CONFIG, MAX7300_CONFIG_SHUTDOWN);
  if (result == MAX7300_OK) {
    _powerMode = MAX7300_POWER_SHUTDOWN;
  }
  
  return result;
}

int8_t MAX7300::wakeup(bool restoreState) {
  // Set device to normal operating mode first
  int8_t result = writeRegister(MAX7300_REG_CONFIG, MAX7300_CONFIG_NORMAL);
  if (result != MAX7300_OK) {
    return result;
  }
  
  // If requested and we have valid cached state, restore it
  if (restoreState) {
    // Restore pin configurations if cache is valid
    if (_pinConfigCacheValid) {
      for (uint8_t i = 0; i < 7; i++) {
        if (writeRegister(MAX7300_REG_PIN_CONFIG_BASE + i, _pinConfigCache[i]) != MAX7300_OK) {
          return _lastError;
        }
      }
    }
    
    // Restore output states if cache is valid
    if (_outputStateCacheValid) {
      // First identify which pins are configured as outputs
      uint32_t outputMask = 0;
      
      for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
        if (getPinMode(pin) == MAX7300_PIN_OUTPUT) {
          outputMask |= (1UL << pin);
        }
      }
      
      // Then restore their states
      if (outputMask != 0) {
        uint32_t values = 0;
        
        for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
          if (outputMask & (1UL << pin)) {
            if (_outputStateCache[pin]) {
              values |= (1UL << pin);
            }
          }
        }
        
        if (digitalWriteAll(values, outputMask) != MAX7300_OK) {
          return _lastError;
        }
      }
    }
  }
  
  _powerMode = MAX7300_POWER_NORMAL;
  return MAX7300_OK;
}

int8_t MAX7300::saveConfiguration(uint16_t eepromAddress) {
  // Prepare configuration structure
  _config.magic = MAX7300_EEPROM_MAGIC;
  _config.version = MAX7300_EEPROM_VERSION;
  _config.powerMode = _powerMode;
  
  // Update configuration caches if needed
  if (!_pinConfigCacheValid) {
    if (updatePinConfigCache() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  if (!_outputStateCacheValid) {
    if (updateOutputStateCache() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  // Copy pin configuration registers
  for (uint8_t i = 0; i < 7; i++) {
    _config.pinModes[i] = _pinConfigCache[i];
  }
  
  // Copy output states
  for (uint8_t i = 0; i < MAX7300_TOTAL_PINS; i++) {
    _config.defaultOutputs[i] = _outputStateCache[i];
  }
  
  // Get transition detection configuration
  int16_t mask = readRegister(MAX7300_REG_MASK);
  if (mask < 0) {
    return _lastError;
  }
  _config.transitionMask = (uint8_t)mask;
  
  int16_t trans = readRegister(MAX7300_REG_TRANSITION);
  if (trans < 0) {
    return _lastError;
  }
  _config.transitionTypes = (uint8_t)trans;
  
  // Calculate checksum
  _config.checksum = calculateChecksum((uint8_t*)&_config, sizeof(_config) - 1);
  
  // Write to EEPROM
  for (uint16_t i = 0; i < sizeof(_config); i++) {
    EEPROM.write(eepromAddress + i, ((uint8_t*)&_config)[i]);
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::loadConfiguration(uint16_t eepromAddress, bool applyConfig) {
  // Read from EEPROM
  for (uint16_t i = 0; i < sizeof(_config); i++) {
    ((uint8_t*)&_config)[i] = EEPROM.read(eepromAddress + i);
  }
  
  // Verify magic number and version
  if (_config.magic != MAX7300_EEPROM_MAGIC) {
    _lastError = MAX7300_ERR_EEPROM_FAIL;
    return _lastError;
  }
  
  // Verify checksum
  uint8_t checksum = calculateChecksum((uint8_t*)&_config, sizeof(_config) - 1);
  if (checksum != _config.checksum) {
    _lastError = MAX7300_ERR_EEPROM_FAIL;
    return _lastError;
  }
  
  // Apply configuration if requested
  if (applyConfig) {
    return applyConfiguration(_config);
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::saveConfigurationExt(bool (*writeCallback)(uint16_t address, uint8_t* data, uint16_t length)) {
  if (!writeCallback) {
    _lastError = MAX7300_ERR_INVALID_MODE; // Reuse error for null callback
    return _lastError;
  }
  
  // Prepare configuration same as with internal EEPROM
  _config.magic = MAX7300_EEPROM_MAGIC;
  _config.version = MAX7300_EEPROM_VERSION;
  _config.powerMode = _powerMode;
  
  // Update configuration caches if needed
  if (!_pinConfigCacheValid) {
    if (updatePinConfigCache() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  if (!_outputStateCacheValid) {
    if (updateOutputStateCache() != MAX7300_OK) {
      return _lastError;
    }
  }
  
  // Copy pin configuration registers
  for (uint8_t i = 0; i < 7; i++) {
    _config.pinModes[i] = _pinConfigCache[i];
  }
  
  // Copy output states
  for (uint8_t i = 0; i < MAX7300_TOTAL_PINS; i++) {
    _config.defaultOutputs[i] = _outputStateCache[i];
  }
  
  // Get transition detection configuration
  int16_t mask = readRegister(MAX7300_REG_MASK);
  if (mask < 0) {
    return _lastError;
  }
  _config.transitionMask = (uint8_t)mask;
  
  int16_t trans = readRegister(MAX7300_REG_TRANSITION);
  if (trans < 0) {
    return _lastError;
  }
  _config.transitionTypes = (uint8_t)trans;
  
  // Calculate checksum
  _config.checksum = calculateChecksum((uint8_t*)&_config, sizeof(_config) - 1);
  
  // Call the write callback
  if (!writeCallback(0, (uint8_t*)&_config, sizeof(_config))) {
    _lastError = MAX7300_ERR_EEPROM_FAIL;
    return _lastError;
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::loadConfigurationExt(bool (*readCallback)(uint16_t address, uint8_t* data, uint16_t length), bool applyConfig) {
  if (!readCallback) {
    _lastError = MAX7300_ERR_INVALID_MODE; // Reuse error for null callback
    return _lastError;
  }
  
  // Call the read callback
  if (!readCallback(0, (uint8_t*)&_config, sizeof(_config))) {
    _lastError = MAX7300_ERR_EEPROM_FAIL;
    return _lastError;
  }
  
  // Verify magic number and version
  if (_config.magic != MAX7300_EEPROM_MAGIC) {
    _lastError = MAX7300_ERR_EEPROM_FAIL;
    return _lastError;
  }
  
  // Verify checksum
  uint8_t checksum = calculateChecksum((uint8_t*)&_config, sizeof(_config) - 1);
  if (checksum != _config.checksum) {
    _lastError = MAX7300_ERR_EEPROM_FAIL;
    return _lastError;
  }
  
  // Apply configuration if requested
  if (applyConfig) {
    return applyConfiguration(_config);
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::applyConfiguration(const MAX7300Config& config) {
  // Reset to known state
  if (reset(false) != MAX7300_OK) {
    return _lastError;
  }
  
  // Apply power mode first
  if (setPowerMode(config.powerMode) != MAX7300_OK) {
    return _lastError;
  }
  
  // Apply pin configurations
  for (uint8_t i = 0; i < 7; i++) {
    if (writeRegister(MAX7300_REG_PIN_CONFIG_BASE + i, config.pinModes[i]) != MAX7300_OK) {
      return _lastError;
    }
  }
  
  // Apply output states for pins configured as outputs
  for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
    uint8_t reg = getConfigRegister(pin);
    uint8_t shift = getPinShift(pin);
    uint8_t mode = (config.pinModes[reg - MAX7300_REG_PIN_CONFIG_BASE] >> shift) & 0x03;
    
    if (mode == MAX7300_PIN_OUTPUT) {
      if (digitalWrite(pin, config.defaultOutputs[pin]) != MAX7300_OK) {
        return _lastError;
      }
    }
  }
  
  // Apply transition detection configuration
  if (writeRegister(MAX7300_REG_MASK, config.transitionMask) != MAX7300_OK) {
    return _lastError;
  }
  
  if (writeRegister(MAX7300_REG_TRANSITION, config.transitionTypes) != MAX7300_OK) {
    return _lastError;
  }
  
  // Update internal caches
  _pinConfigCacheValid = true;
  for (uint8_t i = 0; i < 7; i++) {
    _pinConfigCache[i] = config.pinModes[i];
  }
  
  _outputStateCacheValid = true;
  for (uint8_t i = 0; i < MAX7300_TOTAL_PINS; i++) {
    _outputStateCache[i] = config.defaultOutputs[i];
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::updateOutputStateCache() {
  // For each pin configured as an output, read its current state
  for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
    int8_t mode = getPinMode(pin);
    if (mode < 0) {
      return mode; // Return error code
    }
    
    if (mode == MAX7300_PIN_OUTPUT) {
      int8_t state = digitalRead(pin);
      if (state < 0) {
        return state; // Return error code
      }
      
      _outputStateCache[pin] = (state > 0) ? 1 : 0;
    } else {
      _outputStateCache[pin] = 0; // Default for non-outputs
    }
  }
  
  _outputStateCacheValid = true;
  return MAX7300_OK;
}

uint8_t MAX7300::calculateChecksum(uint8_t* data, uint8_t length) {
  uint8_t checksum = 0;
  
  for (uint8_t i = 0; i < length; i++) {
    checksum += data[i];
  }
  
  return ~checksum; // One's complement
}

bool MAX7300::validPin(uint8_t pin) {
  return pin <= MAX7300_MAX_PIN;
}

bool MAX7300::validMode(uint8_t mode) {
  return mode <= MAX7300_PIN_INPUT_PULLUP;
}

bool MAX7300::validTransitionMode(uint8_t mode) {
  return mode <= MAX7300_TRANSITION_BOTH;
}

uint8_t MAX7300::getConfigRegister(uint8_t pin) {
  return MAX7300_REG_PIN_CONFIG_BASE + (pin / MAX7300_PINS_PER_REG);
}

uint8_t MAX7300::getPinShift(uint8_t pin) {
  return 2 * (pin % MAX7300_PINS_PER_REG);
}

int8_t MAX7300::updatePinConfigCache() {
  // Read all configuration registers into cache
  int8_t result = readRegisters(MAX7300_REG_PIN_CONFIG_BASE, _pinConfigCache, 7);
  if (result == MAX7300_OK) {
    _pinConfigCacheValid = true;
  } else {
    _pinConfigCacheValid = false;
  }
  
  return result;
}

int8_t MAX7300::writePinConfigRegister(uint8_t reg, uint8_t value) {
  int8_t result = writeRegister(reg, value);
  
  // If write was successful, update cache
  if (result == MAX7300_OK && reg >= MAX7300_REG_PIN_CONFIG_BASE && reg <= MAX7300_REG_PIN_CONFIG_BASE + 6) {
    _pinConfigCache[reg - MAX7300_REG_PIN_CONFIG_BASE] = value;
  }
  
  return result;
}

void MAX7300::handleInterrupt(void* instance) {
  MAX7300* obj = static_cast<MAX7300*>(instance);
  obj->_interruptOccurred = true;
  
  if (obj->_interruptCallback) {
    obj->_interruptCallback(obj->getTransitionEvents());
  } else if (obj->_eventCallback) {
    MAX7300Event event;
    if (obj->getEventDetails(&event)) {
      obj->_eventCallback(event);
    }
  }
}

bool MAX7300::validPowerMode(uint8_t mode) {
  return (mode == MAX7300_POWER_NORMAL || 
          mode == MAX7300_POWER_STANDBY || 
          mode == MAX7300_POWER_SHUTDOWN);
}

const char* MAX7300::getErrorMessage() const {
  switch (_lastError) {
    case MAX7300_OK:
      return "Success";
    case MAX7300_ERR_WRONG_DEVICE:
      return "Wrong device detected";
    case MAX7300_ERR_COMM_FAIL:
      return "Communication failure";
    case MAX7300_ERR_INVALID_PIN:
      return "Invalid pin number";
    case MAX7300_ERR_INVALID_MODE:
      return "Invalid mode specified";
    case MAX7300_ERR_NOT_SUPPORTED:
      return "Operation not supported";
    case MAX7300_ERR_TIMEOUT:
      return "Communication timeout";
    case MAX7300_ERR_EEPROM_FAIL:
      return "EEPROM configuration error";
    default:
      return "Unknown error";
  }
}
