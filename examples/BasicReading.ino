/**
 * @file AD7327_BasicReading.ino
 * @brief Basic example for reading data from AD7327 ADC
 * @author Datanoise
 * @date 2025-04-03
 * 
 * This example reads from all 8 channels of the AD7327 in single-ended mode
 * and prints the results to the serial monitor.
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
 * - Connect analog inputs as needed
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
  
  Serial.println("AD7327 ADC Basic Reading Example");
  
  // Initialize the ADC
  if (!adc.begin()) {
    Serial.println("Failed to initialize AD7327!");
    while (1);
  }
  
  // Configure the ADC
  adc.setInputMode(AD7327::Mode::SINGLE_ENDED);  // 8 single-ended inputs
  adc.setReference(AD7327::Reference::INTERNAL); // Use internal 2.5V reference
  adc.setCoding(AD7327::Coding::TWOS_COMPLEMENT); // Use two's complement coding
  
  // Set different ranges for different channels as an example
  adc.setRange(0, AD7327::Range::RANGE_10V);     // ±10V for channel 0
  adc.setRange(1, AD7327::Range::RANGE_5V);      // ±5V for channel 1
  adc.setRange(2, AD7327::Range::RANGE_2_5V);    // ±2.5V for channel 2
  adc.setRange(3, AD7327::Range::RANGE_0_10V);   // 0V to +10V for channel 3
  
  // Channels 4-7 default to ±10V
  
  Serial.println("ADC configured successfully");
  Serial.println();
  Serial.println("Channel | Raw Value | Voltage (V)");
  Serial.println("--------|-----------|------------");
}

void loop() {
  // Read from all 8 channels
  for (int i = 0; i < 8; i++) {
    // Read the raw conversion result
    int16_t rawValue = adc.readChannel(i);
    
    // Convert to voltage
    float voltage = adc.resultToVoltage(rawValue, i);
    
    // Print results
    Serial.print("   ");
    Serial.print(i);
    Serial.print("    |   ");
    Serial.print(rawValue);
    Serial.print("   |   ");
    Serial.print(voltage, 4);
    Serial.println(" V");
  }
  
  // Also read the temperature
  float temp = adc.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temp, 1);
  Serial.println(" °C");
  
  Serial.println();
  delay(1000);  // Wait 1 second between readings
}
