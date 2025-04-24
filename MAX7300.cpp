
/**
 * MAX7300 - I2C Port Expander Library Implementation
 */

#include "MAX7300.h"

MAX7300::MAX7300(uint8_t addr, TwoWire *wire) {
  // Store the I2C address and Wire interface
  _address = addr;
  _wire = wire;
  _lastError = MAX7300_OK;
  _timeout = 1000; // 1 second default timeout
  _initialized = false;
  _pinConfigCacheValid = false;
  
  // Initialize cache
  for (int i = 0; i < 7; i++) {
    _pinConfigCache[i] = 0;
  }
}

int8_t MAX7300::begin(bool resetPins) {
  _wire->begin();
  
  // Check if the device is responding
  _wire->beginTransmission(_address);
  if (_wire->endTransmission() != 0) {
    _lastError = MAX7300_ERR_COMM_FAIL;
    return _lastError;
  }
  
  if (resetPins) {
    // Reset all pins to high-impedance state (default)
    for (uint8_t reg = MAX7300_REG_PIN_CONFIG_BASE; reg <= MAX7300_REG_PIN_CONFIG_BASE + 6; reg++) {
      if (writeRegister(reg, 0x00) != MAX7300_OK) {
        return _lastError;
      }
    }
  }
  
  // Initialize the configuration cache
  updatePinConfigCache();
  
  // Set device to normal operating mode
  int8_t result = enable();
  if (result == MAX7300_OK) {
    _initialized = true;
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
  return writeRegister(MAX7300_REG_CONFIG, MAX7300_CONFIG_NORMAL);
}

int8_t MAX7300::disable() {
  // Set shutdown mode (bit 0 = 0)
  return writeRegister(MAX7300_REG_CONFIG, MAX7300_CONFIG_SHUTDOWN);
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
  
  // Write the updated configuration to the device
  int8_t result = writePinConfigRegister(reg, newConfig);
  
  // Update cache if successful and requested
  if (result == MAX7300_OK && updateCache) {
    _pinConfigCache[reg - MAX7300_REG_PIN_CONFIG_BASE] = newConfig;
  }
  
  return result;
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
  
  // Update configuration for each pin in the mask
  for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
    if (pinMask & (1UL << pin)) {
      // Use setPinMode with updateCache=false to avoid excessive I2C traffic
      int8_t result = setPinMode(pin, mode, false);
      if (result != MAX7300_OK) {
        return result;
      }
    }
  }
  
  // Update the cache once at the end
  return updatePinConfigCache();
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
  for (uint8_t pin = 0; pin <= MAX7300_MAX_PIN; pin++) {
    int16_t state = readRegister(MAX7300_REG_PIN_STATE_BASE + pin);
    if (state < 0) {
      // Error occurred, return 0
      _lastError = state;
      return 0;
    }
    
    if (state & 0x01) {
      result |= (1UL << pin);
    }
  }
  
  return result;
}

int8_t MAX7300::digitalWriteAll(uint32_t values, uint32_t mask) {
  // Update each pin as needed
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
  
  // Note: MAX7300 doesn't directly support BOTH mode, it would have to be emulated in software
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

int16_t MAX7300::readRegister(uint8_t reg) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission() != 0) {
    _lastError = MAX7300_ERR_COMM_FAIL;
    return _lastError;
  }
  
  // Request 1 byte from device
  if (_wire->requestFrom(_address, (uint8_t)1) != 1) {
    _lastError = MAX7300_ERR_COMM_FAIL;
    return _lastError;
  }
  
  uint8_t value = _wire->read();
  return value;
}

int8_t MAX7300::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  
  if (_wire->endTransmission() != 0) {
    _lastError = MAX7300_ERR_COMM_FAIL;
    return _lastError;
  }
  
  return MAX7300_OK;
}

int8_t MAX7300::getLastError() const {
  return _lastError;
}

void MAX7300::setTimeout(uint16_t timeout) {
  _timeout = timeout;
  _wire->setTimeout(timeout);
}

uint16_t MAX7300::getTimeout() const {
  return _timeout;
}

// Private helper methods
bool MAX7300::validPin(uint8_t pin) {
  return pin <= MAX7300_MAX_PIN;
}

bool MAX7300::validMode(uint8_t mode) {
  return mode <= MAX7300_PIN_INPUT_PULLUP;
}

uint8_t MAX7300::getConfigRegister(uint8_t pin) {
  return MAX7300_REG_PIN_CONFIG_BASE + (pin / MAX7300_PINS_PER_REG);
}

uint8_t MAX7300::getPinShift(uint8_t pin) {
  return 2 * (pin % MAX7300_PINS_PER_REG);
}

int8_t MAX7300::updatePinConfigCache() {
  // Read all configuration registers into cache
  for (uint8_t i = 0; i < 7; i++) {
    int16_t value = readRegister(MAX7300_REG_PIN_CONFIG_BASE + i);
    if (value < 0) {
      _pinConfigCacheValid = false;
      return _lastError;
    }
    _pinConfigCache[i] = value;
  }
  
  _pinConfigCacheValid = true;
  return MAX7300_OK;
}

int8_t MAX7300::writePinConfigRegister(uint8_t reg, uint8_t value) {
  int8_t result = writeRegister(reg, value);
  
  // If write was successful, update cache
  if (result == MAX7300_OK && reg >= MAX7300_REG_PIN_CONFIG_BASE && reg <= MAX7300_REG_PIN_CONFIG_BASE + 6) {
    _pinConfigCache[reg - MAX7300_REG_PIN_CONFIG_BASE] = value;
  }
  
  return result;
}
