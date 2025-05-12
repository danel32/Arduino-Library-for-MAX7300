# MAX7300 I2C GPIO Port Expander Library

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive, professional-grade Arduino library for the MAX7300 I²C GPIO port expander chip manufactured by Analog Devices (formerly Maxim Integrated). This library provides complete access to all features of the MAX7300 with an intuitive, user-friendly API.

## Features

- **Complete Access to All Functionality**:
  - Individual pin configuration (input, output, high-impedance)
  - Individual and batch pin read/write operations
  - Full transition detection support
  - Power management (normal/shutdown modes)

- **High Performance**:
  - Efficient register caching to minimize I2C traffic
  - Optimized read/write operations

- **Developer Friendly**:
  - Clear, consistent API
  - Comprehensive error reporting and handling
  - Detailed documentation and examples
  - Input validation to prevent errors

- **Configurable**:
  - Multiple chip support with different I2C addresses
  - Compatible with all Arduino platforms
  - Support for multiple I2C buses

## Hardware Overview

The MAX7300 is a 28-port I²C-interfaced GPIO expander that allows microcontrollers to control up to 28 additional general-purpose input/output (GPIO) pins through an I²C serial interface. Key features of the MAX7300:

- 28 I/O ports configurable as inputs or outputs
- 2.5V to 5.5V operating voltage range
- 400kHz I²C-compatible interface
- Transition detection capability on pins 4-27
- Low power consumption (<1µA in shutdown mode)
- Configurable I²C address (4 possible addresses)
- Small 24-pin QSOP or 36-pin TQFN packages

## Installation

### Using the Arduino Library Manager

1. Open the Arduino IDE
2. Go to Sketch > Include Library > Manage Libraries...
3. Search for "MAX7300"
4. Click "Install"

### Manual Installation

1. Download the [latest release](https://github.com/danel32/Arduino-Library-for-MAX7300)
2. Extract the ZIP file
3. Copy the folder to your Arduino libraries directory
   - Windows: Documents/Arduino/libraries/
   - Mac/Linux: ~/Documents/Arduino/libraries/
4. Restart the Arduino IDE

## Wiring

Connect your MAX7300 to your Arduino or compatible board:

| MAX7300 Pin | Arduino Pin | Description |
|-------------|-------------|-------------|
| VCC         | 3.3V/5V*    | Power supply (2.5V to 5.5V) |
| GND         | GND         | Ground |
| SDA         | SDA†        | I2C data line |
| SCL         | SCL†        | I2C clock line |
| A0          | GND/VCC     | Address select bit 0 |
| A1          | GND/VCC     | Address select bit 1 |
| A2          | GND/VCC     | Address select bit 2 |

\* Make sure to use a voltage compatible with your microcontroller's I/O levels.  
† Use the appropriate I2C pins for your Arduino board:
- Arduino Uno/Nano: A4 (SDA), A5 (SCL)
- Arduino Mega: 20 (SDA), 21 (SCL)
- Arduino Leonardo/Micro: 2 (SDA), 3 (SCL)
- Arduino ESP8266/ESP32: Check your board's documentation

## Quick Start

```cpp
#include <Wire.h>
#include <MAX7300.h>

// Create a MAX7300 object with default address (0x40)
MAX7300 gpio;

void setup() {
  Serial.begin(115200);
  
  // Initialize the MAX7300
  if (gpio.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300");
    while (1);
  }
  
  // Configure pins
  gpio.pinMode(10, MAX7300_PIN_OUTPUT);     // Set pin 10 as output
  gpio.pinMode(11, MAX7300_PIN_INPUT);      // Set pin 11 as input
  
  // Set output pin high
  gpio.digitalWrite(10, HIGH);
}

void loop() {
  // Read input pin
  int value = gpio.digitalRead(11);
  Serial.print("Pin 11 value: ");
  Serial.println(value);
  
  delay(1000);
}
```

## Addressing

The MAX7300's I²C address is determined by the A0, A1, and A2 pins:

| A2 | A1 | A0 | 7-bit Address |
|----|----|----|---------------|
| L  | L  | L  | 0x40          |
| L  | L  | H  | 0x42          |
| L  | H  | L  | 0x44          |
| L  | H  | H  | 0x46          |
| H  | L  | L  | 0x48          |
| H  | L  | H  | 0x4A          |
| H  | H  | L  | 0x4C          |
| H  | H  | H  | 0x4E          |

To use a specific address:

```cpp
// Create MAX7300 with address 0x44 (A2=L, A1=H, A0=L)
MAX7300 gpio(0x44);
```

## API Reference

### Initialization

#### `MAX7300(uint8_t addr = MAX7300_BASE_ADDR, TwoWire *wire = &Wire)`
Constructor for creating a MAX7300 object.

Parameters:
- `addr`: I2C address of the device (default: 0x40)
- `wire`: TwoWire instance for platforms with multiple I2C buses (default: Wire)

#### `int8_t begin(bool resetPins = true)`
Initialize the MAX7300 device.

Parameters:
- `resetPins`: If true, resets all pins to high-impedance mode (default: true)

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `int8_t reset(bool hardReset = false)`
Reset the device to default state.

Parameters:
- `hardReset`: If true, performs a full software reset (default: false)

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

### Power Management

#### `int8_t enable()`
Set device to normal operation mode.

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `int8_t disable()`
Set device to shutdown/low-power mode.

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `bool isEnabled()`
Check if the device is in normal operation mode.

Returns:
- `true` if device is enabled (normal mode), `false` if disabled (shutdown mode)

### Pin Configuration

#### `int8_t pinMode(uint8_t pin, uint8_t mode)`
Configure a pin's mode (input, output, high-impedance).

Parameters:
- `pin`: Pin number (0-27)
- `mode`: Mode (MAX7300_PIN_HIGHZ, MAX7300_PIN_OUTPUT, MAX7300_PIN_INPUT, or MAX7300_PIN_INPUT_PULLUP)

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `int8_t getPinMode(uint8_t pin)`
Get the current mode of a pin.

Parameters:
- `pin`: Pin number (0-27)

Returns:
- Pin mode or negative error code

#### `int8_t setMultiplePinMode(uint32_t pinMask, uint8_t mode)`
Configure multiple pins at once with the same mode.

Parameters:
- `pinMask`: Bit mask of pins to configure (bit 0 = pin 0, etc.)
- `mode`: Mode to set for all selected pins

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

### Digital I/O

#### `int8_t digitalRead(uint8_t pin)`
Read from a GPIO pin.

Parameters:
- `pin`: Pin number (0-27)

Returns:
- Pin state (0 or 1) or negative error code

#### `int8_t digitalWrite(uint8_t pin, uint8_t value)`
Write to a GPIO pin.

Parameters:
- `pin`: Pin number (0-27)
- `value`: Value to write (0 or 1)

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `uint32_t digitalReadAll()`
Read all pin states and return as a 32-bit value.

Returns:
- 32-bit value where bits 0-27 represent pins 0-27

#### `int8_t digitalWriteAll(uint32_t values, uint32_t mask = 0xFFFFFFFF)`
Write values to multiple pins at once.

Parameters:
- `values`: 32-bit value where bits 0-27 represent pins 0-27
- `mask`: 32-bit mask where 1 means update that pin (default: all pins)

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

### Transition Detection

#### `int8_t enableTransitionDetection()`
Enable transition detection functionality.

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `int8_t disableTransitionDetection()`
Disable transition detection functionality.

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `bool isTransitionDetectionEnabled()`
Check if transition detection is enabled.

Returns:
- `true` if enabled, `false` if disabled

#### `int8_t configureTransitionDetection(uint8_t pin, uint8_t mode)`
Configure transition detection for a specific pin.

Parameters:
- `pin`: Pin number (4-27, pins 0-3 don't support transition detection)
- `mode`: Detection mode (MAX7300_TRANSITION_NONE, MAX7300_TRANSITION_LOW, MAX7300_TRANSITION_HIGH, or MAX7300_TRANSITION_BOTH)

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

#### `int8_t getTransitionDetection(uint8_t pin)`
Get the transition detection configuration for a pin.

Parameters:
- `pin`: Pin number (4-27)

Returns:
- Transition mode or negative error code

#### `bool transitionDetected()`
Check if a transition has been detected.

Returns:
- `true` if a transition was detected, `false` otherwise

#### `uint32_t getTransitionEvents()`
Read the transition event register to determine which pins triggered an event.

Returns:
- 32-bit value where bits represent which pins had transitions

#### `int8_t clearTransitionEvents()`
Clear transition events.

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

### Advanced Functions

#### `int16_t readRegister(uint8_t reg)`
Direct register access for advanced users.

Parameters:
- `reg`: Register address

Returns:
- Register value or negative error code

#### `int8_t writeRegister(uint8_t reg, uint8_t value)`
Direct register write for advanced users.

Parameters:
- `reg`: Register address
- `value`: Value to write

Returns:
- `MAX7300_OK` (0) on success, or a negative error code

### Error Handling

#### `int8_t getLastError() const`
Get the last error that occurred.

Returns:
- Error code

#### `void setTimeout(uint16_t timeout)`
Set I2C communication timeout.

Parameters:
- `timeout`: Timeout in milliseconds

#### `uint16_t getTimeout() const`
Get current I2C timeout setting.

Returns:
- Timeout in milliseconds

## Examples

### Basic GPIO
```cpp
#include <Wire.h>
#include <MAX7300.h>

MAX7300 gpio;

void setup() {
  Serial.begin(115200);
  
  if (gpio.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300");
    while (1);
  }
  
  gpio.pinMode(10, MAX7300_PIN_OUTPUT);  // LED pin
  gpio.pinMode(11, MAX7300_PIN_INPUT);   // Button pin
}

void loop() {
  // Read button state
  int buttonState = gpio.digitalRead(11);
  
  // Control LED based on button state
  gpio.digitalWrite(10, buttonState);
  
  delay(10);  // Short delay for debouncing
}
```

### Multiple Pins
```cpp
#include <Wire.h>
#include <MAX7300.h>

MAX7300 gpio;

void setup() {
  Serial.begin(115200);
  
  if (gpio.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300");
    while (1);
  }
  
  // Configure pins 8-15 as outputs (for LEDs)
  uint32_t ledPins = 0xFF00;  // Pins 8-15
  gpio.setMultiplePinMode(ledPins, MAX7300_PIN_OUTPUT);
  
  // Configure pins 16-23 as inputs (for buttons)
  uint32_t buttonPins = 0xFF0000;  // Pins 16-23
  gpio.setMultiplePinMode(buttonPins, MAX7300_PIN_INPUT);
}

void loop() {
  // Read all buttons at once
  uint32_t buttonStates = gpio.digitalReadAll() & 0xFF0000;
  
  // Light LEDs based on button states (shifted to match LED pins)
  uint32_t ledStates = buttonStates >> 8;
  gpio.digitalWriteAll(ledStates, 0xFF00);
  
  delay(10);
}
```

### Transition Detection
```cpp
#include <Wire.h>
#include <MAX7300.h>

MAX7300 gpio;

void setup() {
  Serial.begin(115200);
  
  if (gpio.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300");
    while (1);
  }
  
  // Configure pin 5 as input
  gpio.pinMode(5, MAX7300_PIN_INPUT);
  
  // Enable transition detection for pin 5
  gpio.enableTransitionDetection();
  gpio.configureTransitionDetection(5, MAX7300_TRANSITION_BOTH);
  
  Serial.println("Waiting for transitions on pin 5...");
}

void loop() {
  // Check if a transition occurred
  if (gpio.transitionDetected()) {
    // Get which pins had transitions
    uint32_t events = gpio.getTransitionEvents();
    
    if (events & (1UL << 5)) {
      int state = gpio.digitalRead(5);
      Serial.print("Pin 5 changed to: ");
      Serial.println(state);
    }
    
    // Clear the transition flag
    gpio.clearTransitionEvents();
  }
  
  delay(10);
}
```

### Multiple MAX7300 Devices
```cpp
#include <Wire.h>
#include <MAX7300.h>

// Create two MAX7300 objects with different addresses
MAX7300 gpio1(0x40);  // A0=0, A1=0, A2=0
MAX7300 gpio2(0x42);  // A0=1, A1=0, A2=0

void setup() {
  Serial.begin(115200);
  
  if (gpio1.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300 #1");
    while (1);
  }
  
  if (gpio2.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300 #2");
    while (1);
  }
  
  // Configure pins on first device
  gpio1.pinMode(0, MAX7300_PIN_OUTPUT);
  gpio1.pinMode(1, MAX7300_PIN_OUTPUT);
  
  // Configure pins on second device
  gpio2.pinMode(0, MAX7300_PIN_INPUT);
  gpio2.pinMode(1, MAX7300_PIN_INPUT);
}

void loop() {
  // Read inputs from second device
  int input1 = gpio2.digitalRead(0);
  int input2 = gpio2.digitalRead(1);
  
  // Write values to first device
  gpio1.digitalWrite(0, input1);
  gpio1.digitalWrite(1, input2);
  
  delay(10);
}
```

## Error Codes

| Code | Constant | Description |
|------|----------|-------------|
| 0 | MAX7300_OK | Success |
| -1 | MAX7300_ERR_WRONG_DEVICE | Wrong device detected |
| -2 | MAX7300_ERR_COMM_FAIL | Communication failure |
| -3 | MAX7300_ERR_INVALID_PIN | Invalid pin number |
| -4 | MAX7300_ERR_INVALID_MODE | Invalid pin mode |

## Advanced Usage

### Direct Register Access

For advanced users who need direct access to the MAX7300 registers:

```cpp
// Read Configuration Register (0x04)
int16_t configReg = gpio.readRegister(0x04);
if (configReg >= 0) {
  Serial.print("Config Register: 0x");
  Serial.println(configReg, HEX);
}

// Write to Configuration Register (0x04)
gpio.writeRegister(0x04, 0x01);  // Normal operation mode
```

### Custom Error Handling

```cpp
void performOperation() {
  int8_t result = gpio.pinMode(10, MAX7300_PIN_OUTPUT);
  
  if (result != MAX7300_OK) {
    Serial.print("Error occurred: ");
    switch (result) {
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
        Serial.println("Unknown error");
        break;
    }
  }
}
```

### Using with ESP32/ESP8266 Multiple I2C Buses

```cpp
#include <Wire.h>
#include <MAX7300.h>

TwoWire secondBus = TwoWire(1);  // Create a second I2C instance (for ESP32)
MAX7300 gpio(&secondBus);        // Use the second I2C bus

void setup() {
  Serial.begin(115200);
  
  // Initialize the second I2C bus on custom pins (ESP32 example)
  secondBus.begin(SDA_PIN, SCL_PIN, 400000);
  
  if (gpio.begin() != MAX7300_OK) {
    Serial.println("Error: Could not initialize MAX7300");
    while (1);
  }
  
  // Rest of your setup code
}
```

## License

This library is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Troubleshooting

### Common Issues and Solutions

1. **No Communication**
   - Check wiring and I2C address
   - Verify pull-up resistors on SDA/SCL lines (typically 4.7kΩ)
   - Try a slower I2C speed (lower than 400kHz)

2. **Inconsistent Readings**
   - Check for proper power supply decoupling capacitors
   - Ensure the I2C bus isn't too long or noisy
   - Verify that all chips share a common ground reference

3. **Pin Configuration Issues**
   - Remember that pins start in high-impedance mode after power-up
   - Make sure to enable the device with `gpio.enable()`
   - Pins 0-3 have special functions on some variants of the chip

4. **Transition Detection Not Working**
   - Only pins 4-27 support transition detection
   - Make sure to enable transition detection with `enableTransitionDetection()`
   - Clear transition events with `clearTransitionEvents()`

## Contributing

Contributions to improve the library are welcome. Please follow these steps:

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Credits

Library developed by danel32 /erbolite@gmail.com/.

## Version History

- 1.0.0 (2023-04-23): Initial release
  - Complete implementation of all MAX7300 features
  - Extensive documentation and examples
  - Error handling and input validation

---

If you find this library helpful, please consider starring the repository on GitHub!
