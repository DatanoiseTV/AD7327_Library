/**
 * @file AD7327_DifferentialMode.ino
 * @brief Example for using the AD7327 ADC in differential mode
 * @author Datanoise
 * @date 2025-04-03
 * 
 * This example demonstrates using the AD7327 in true differential mode,
 * where the 8 input channels are configured as 4 true differential pairs.
 * 
 * Hardware connections:
 * - Connect SCLK to Arduino SPI SCK
 * - Connect DOUT to Arduino SPI MISO
 * - Connect DIN to Arduino SPI MOSI
 * - Connect CS to Arduino pin 10 (or change as needed)
 * - Connect VDD to +15V (or appropriate supply)
 * - Connect VSS to -15V (or appropriate supply)
 * - Connect VCC to +5V
 * - Connect VDRIVE to Arduino logic level (+5V or +3.3V)
 * - Connect AGND and DGND to GND
 * - Differential input pairs: (VIN0,VIN1), (VIN2,VIN3), (VIN4,VIN5), (VIN6,VIN7)
 * - If using internal reference, add a 680nF capacitor on REFIN/OUT to AGND
 */

#include <SPI.h>
#include "AD7327.h"

// Pin definitions
const int CS_PIN = 10;
const int RESET_PIN = -1;  // Set to actual pin if hardware reset is connected

// Create AD7327 object
AD7327 adc(CS_PIN, &SPI1);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB port only)
  }
  
  Serial.println("AD7327 ADC Differential Mode Example");
  
  // Initialize the ADC
  if (!adc.begin()) {
    Serial.println("Failed to initialize AD7327!");
    while (1);
  }
  
  // Configure the ADC for true differential mode
  adc.setInputMode(AD7327::Mode::TRUE_DIFFERENTIAL);
  adc.setReference(AD7327::Reference::INTERNAL); // Use internal 2.5V reference
  adc.setCoding(AD7327::Coding::TWOS_COMPLEMENT); // Use two's complement coding
  
  // Set ranges for differential pairs
  adc.setRange(0, AD7327::Range::RANGE_10V);  // Pair 0 (VIN0,VIN1): ±10V
  adc.setRange(2, AD7327::Range::RANGE_5V);   // Pair 1 (VIN2,VIN3): ±5V
  adc.setRange(4, AD7327::Range::RANGE_2_5V); // Pair 2 (VIN4,VIN5): ±2.5V
  adc.setRange(6, AD7327::Range::RANGE_10V);  // Pair 3 (VIN6,VIN7): ±10V
  
  Serial.println("ADC configured for differential mode");
  Serial.println();
  Serial.println("Differential Pair | Raw Value | Voltage (V)");
  Serial.println("------------------|-----------|------------");
}

void loop() {
  // Read from all 4 differential pairs
  // Note that in differential mode, we access pairs via channels 0, 2, 4, 6
  int differentialChannels[] = {0, 2, 4, 6};
  
  for (int i = 0; i < 4; i++) {
    int channel = differentialChannels[i];
    
    // Read the raw conversion result
    int16_t rawValue = adc.readChannel(channel);
    
    // Convert to voltage
    float voltage = adc.resultToVoltage(rawValue, channel);
    
    // Print results
    Serial.print("      ");
    Serial.print(i);
    Serial.print(" (VIN");
    Serial.print(channel);
    Serial.print(",VIN");
    Serial.print(channel + 1);
    Serial.print(")    |   ");
    Serial.print(rawValue);
    Serial.print("   |   ");
    Serial.print(voltage, 4);
    Serial.println(" V");
  }
  
  Serial.println();
  delay(1000);  // Wait 1 second between readings
}
