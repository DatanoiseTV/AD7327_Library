/**
 * @file AD7327_Sequencer.ino
 * @brief Example for using the AD7327 ADC's sequencer functionality
 * @author Datanoise
 * @date 2025-04-03
 * 
 * This example demonstrates using the AD7327's channel sequencer to
 * automatically cycle through a sequence of channels and perform
 * conversions.
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
  
  Serial.println("AD7327 ADC Sequencer Example");
  
  // Initialize the ADC
  if (!adc.begin()) {
    Serial.println("Failed to initialize AD7327!");
    while (1);
  }
  
  // Configure the ADC
  adc.setInputMode(AD7327::Mode::SINGLE_ENDED);  // 8 single-ended inputs
  adc.setReference(AD7327::Reference::INTERNAL); // Use internal 2.5V reference
  adc.setCoding(AD7327::Coding::TWOS_COMPLEMENT); // Use two's complement coding
  
  // Set ranges for all channels
  AD7327::Range ranges[8] = {
    AD7327::Range::RANGE_0_10V,  // Channel 0: ±10V
    AD7327::Range::RANGE_0_10V,   // Channel 1: ±5V
    AD7327::Range::RANGE_0_10V, // Channel 2: ±2.5V
    AD7327::Range::RANGE_0_10V,// Channel 3: 0V to +10V
    AD7327::Range::RANGE_0_10V,  // Channel 4: ±10V
    AD7327::Range::RANGE_0_10V,  // Channel 5: ±10V
    AD7327::Range::RANGE_0_10V,  // Channel 6: ±10V
    AD7327::Range::RANGE_0_10V   // Channel 7: ±10V
  };
  adc.setAllRanges(ranges);
  
  Serial.println("ADC configured successfully");
  Serial.println();
  
  // Example 1: Programmed Sequence
  demoSequencer();
  
  // Example 2: Consecutive Sequence
  demoConsecutiveSequence();
}

void loop() {
  // Main loop is empty as demos are run in setup
  delay(1000);
}

void demoSequencer() {
  Serial.println("=============================================");
  Serial.println("Example 1: Programmed Sequence");
  Serial.println("Channels 0, 2, 4, and 6 in sequence");
  Serial.println("=============================================");
  
  // Set up a channel sequence with channels 0, 2, 4, and 6
  uint8_t channelMask = (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6);
  adc.setSequence(channelMask);
  
  // Enable the sequencer in programmed mode
  adc.configureSequencer(AD7327::SequencerMode::PROGRAMMED);
  
  // Read and display 10 conversions from the sequence
  Serial.println("Channel | Raw Value | Voltage (V)");
  Serial.println("--------|-----------|------------");
  
  for (int i = 0; i < 10; i++) {
    uint8_t channel;
    int16_t rawValue = adc.readNextInSequence(&channel);
    float voltage = adc.resultToVoltage(rawValue, channel);
    
    Serial.print("   ");
    Serial.print(channel);
    Serial.print("    |   ");
    Serial.print(rawValue);
    Serial.print("   |   ");
    Serial.print(voltage, 4);
    Serial.println(" V");
    
    delay(100);  // Short delay between conversions
  }
  
  // Disable the sequencer
  adc.configureSequencer(AD7327::SequencerMode::DISABLED);
  
  Serial.println();
}

void demoConsecutiveSequence() {
  Serial.println("=============================================");
  Serial.println("Example 2: Consecutive Sequence");
  Serial.println("All channels from 0 to 5 in sequence");
  Serial.println("=============================================");
  
  // Enable the sequencer in consecutive mode, ending at channel 5
  adc.configureSequencer(AD7327::SequencerMode::CONSECUTIVE, 5);
  
  // Read and display 12 conversions from the sequence
  // (We should see channels 0-5 twice)
  Serial.println("Channel | Raw Value | Voltage (V)");
  Serial.println("--------|-----------|------------");
  
  for (int i = 0; i < 12; i++) {
    uint8_t channel;
    int16_t rawValue = adc.readNextInSequence(&channel);
    float voltage = adc.resultToVoltage(rawValue, channel);
    
    Serial.print("   ");
    Serial.print(channel);
    Serial.print("    |   ");
    Serial.print(rawValue);
    Serial.print("   |   ");
    Serial.print(voltage, 4);
    Serial.println(" V");
    
    delay(100);  // Short delay between conversions
  }
  
  // Disable the sequencer
  adc.configureSequencer(AD7327::SequencerMode::DISABLED);
  
  Serial.println();
}
