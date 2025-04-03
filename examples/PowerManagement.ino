/**
 * @file AD7327_PowerManagement.ino
 * @brief Example for using the AD7327 ADC power management features
 * @author Your Name
 * @date 2025-04-03
 * 
 * This example demonstrates the different power modes of the AD7327,
 * including normal mode, autostandby mode, autoshutdown mode, and
 * full shutdown mode.
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
  
  Serial.println("AD7327 ADC Power Management Example");
  
  // Initialize the ADC
  if (!adc.begin()) {
    Serial.println("Failed to initialize AD7327!");
    while (1);
  }
  
  // Configure the ADC
  adc.setInputMode(AD7327::Mode::SINGLE_ENDED);  // 8 single-ended inputs
  adc.setReference(AD7327::Reference::INTERNAL); // Use internal 2.5V reference
  adc.setCoding(AD7327::Coding::TWOS_COMPLEMENT); // Use two's complement coding
  
  // All channels default to ±10V range
  
  Serial.println("ADC configured successfully");
  Serial.println();
}

void loop() {
  // Demonstrate Normal Mode
  demoNormalMode();
  
  // Demonstrate AutoStandby Mode
  demoAutoStandbyMode();
  
  // Demonstrate AutoShutdown Mode
  demoAutoShutdownMode();
  
  // Demonstrate Full Shutdown Mode
  demoFullShutdownMode();
}

void demoNormalMode() {
  Serial.println("=============================================");
  Serial.println("Power Mode: NORMAL");
  Serial.println("All circuitry powered up at all times");
  Serial.println("=============================================");
  
  // Set to normal mode
  adc.setPowerMode(AD7327::PowerMode::NORMAL);
  
  // Take a few readings
  takeSampleReadings();
  
  Serial.println();
}

void demoAutoStandbyMode() {
  Serial.println("=============================================");
  Serial.println("Power Mode: AUTOSTANDBY");
  Serial.println("Most circuitry powers down between conversions");
  Serial.println("Internal reference remains powered");
  Serial.println("=============================================");
  
  // Set to autostandby mode
  adc.setPowerMode(AD7327::PowerMode::AUTOSTANDBY);
  
  // Take a few readings
  takeSampleReadings();
  
  Serial.println();
}

void demoAutoShutdownMode() {
  Serial.println("=============================================");
  Serial.println("Power Mode: AUTOSHUTDOWN");
  Serial.println("All circuitry powers down between conversions");
  Serial.println("Longer power-up time required");
  Serial.println("=============================================");
  
  // Set to autoshutdown mode
  adc.setPowerMode(AD7327::PowerMode::AUTOSHUTDOWN);
  
  // Take a few readings (with longer delays to account for power-up time)
  for (int i = 0; i < 3; i++) {
    // Read from channel 0
    int16_t rawValue = adc.readChannel(0);
    float voltage = adc.resultToVoltage(rawValue, 0);
    
    Serial.print("Channel 0: ");
    Serial.print(rawValue);
    Serial.print(" (");
    Serial.print(voltage, 4);
    Serial.println(" V)");
    
    delay(250);  // Longer delay to demonstrate slower conversions
  }
  
  Serial.println();
}

void demoFullShutdownMode() {
  Serial.println("=============================================");
  Serial.println("Power Mode: FULL_SHUTDOWN");
  Serial.println("Complete shutdown, no conversions possible");
  Serial.println("Must be explicitly woken up");
  Serial.println("=============================================");
  
  // Set to full shutdown mode
  adc.setPowerMode(AD7327::PowerMode::FULL_SHUTDOWN);
  
  Serial.println("ADC is now in full shutdown mode");
  Serial.println("No conversions are possible");
  delay(2000);
  
  Serial.println("Waking up from full shutdown...");
  
  // Wake up by setting to normal mode
  adc.setPowerMode(AD7327::PowerMode::NORMAL);
  
  // Take one reading to verify we're awake
  int16_t rawValue = adc.readChannel(0);
  float voltage = adc.resultToVoltage(rawValue, 0);
  
  Serial.print("Channel 0: ");
  Serial.print(rawValue);
  Serial.print(" (");
  Serial.print(voltage, 4);
  Serial.println(" V)");
  
  Serial.println();
  delay(2000);  // Pause before repeating the demo
}

void takeSampleReadings() {
  // Take 3 sample readings from channel 0
  for (int i = 0; i < 3; i++) {
    int16_t rawValue = adc.readChannel(0);
    float voltage = adc.resultToVoltage(rawValue, 0);
    
    Serial.print("Channel 0: ");
    Serial.print(rawValue);
    Serial.print(" (");
    Serial.print(voltage, 4);
    Serial.println(" V)");
    
    delay(100);
  }
}
