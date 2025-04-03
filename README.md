# AD7327 Arduino Library

An Arduino library for the Analog Devices AD7327 12-bit plus sign SAR ADC. This library provides a complete interface to all functionality of the AD7327, including true bipolar input, software-selectable input ranges, channel sequencer, and power management features.

## Features

* Full support for the AD7327's 8 analog input channels
* Configurable input modes:
  * 8 single-ended inputs
  * 4 true differential input pairs
  * 4 pseudo differential inputs
  * 7 pseudo differential inputs
* Configurable input ranges for each channel:
  * ±10V
  * ±5V
  * ±2.5V
  * 0V to +10V
* Selectable internal 2.5V reference or external reference
* Configurable output coding (two's complement or straight binary)
* Channel sequencer with programmable and consecutive modes
* Power management modes:
  * Normal mode
  * Autostandby mode
  * Autoshutdown mode
  * Full shutdown mode
* Temperature indicator support

## Hardware Requirements

* Arduino board with SPI support
* AD7327 ADC
* Appropriate power supplies for VDD, VSS, and VCC
* If using internal reference, a 680nF capacitor on REFIN/OUT pin

## Connections

* SCLK: Connect to SPI SCK
* DIN: Connect to SPI MOSI
* DOUT: Connect to SPI MISO
* CS: Connect to a digital output pin on Arduino
* VDD: Connect to positive supply (+12V to +16.5V recommended)
* VSS: Connect to negative supply (-12V to -16.5V recommended)
* VCC: Connect to +2.7V to +5.25V supply
* VDRIVE: Connect to Arduino logic level (3.3V or 5V)
* AGND/DGND: Connect to ground
* REFIN/OUT: If using internal reference, add 680nF capacitor to AGND
* VIN0-VIN7: Analog inputs, connect as needed based on configuration

## Installation

1. Download the library as a ZIP file
2. In the Arduino IDE, select Sketch > Include Library > Add .ZIP Library...
3. Select the downloaded ZIP file
4. The library will be installed and examples will be available

## Usage

### Basic Example

```cpp
#include <SPI.h>
#include "AD7327.h"

// Define pin connections
const int CS_PIN = 10;

// Create ADC object
AD7327 adc(CS_PIN);

void setup() {
  Serial.begin(115200);
  
  // Initialize the ADC
  adc.begin();
  
  // Configure the ADC
  adc.setInputMode(AD7327::Mode::SINGLE_ENDED);
  adc.setReference(AD7327::Reference::INTERNAL);
  adc.setCoding(AD7327::Coding::TWOS_COMPLEMENT);
}

void loop() {
  // Read from channel 0
  int16_t result = adc.readChannel(0);
  
  // Convert to voltage
  float voltage = adc.resultToVoltage(result, 0);
  
  Serial.print("Channel 0: ");
  Serial.print(voltage, 4);
  Serial.println(" V");
  
  delay(1000);
}
```

## Examples

The library includes several example sketches:

* **AD7327_BasicReading**: Simple example showing how to read from all channels
* **AD7327_DifferentialMode**: Demonstrates using the ADC in true differential mode
* **AD7327_Sequencer**: Shows how to use the channel sequencer
* **AD7327_PowerManagement**: Demonstrates the different power modes

## Documentation

The library is fully documented with doxygen-style comments. Refer to the header file for detailed documentation of all available functions and options.

## License

This library is released under the MIT License.

