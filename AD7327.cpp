/**
 * @file AD7327.cpp
 * @brief Implementation file for the AD7327 12-bit plus sign ADC library
 * @author DaatanoiseTV
 * @version 1.1.0
 * @date 2025-04-03
 */

 #include "AD7327.h"

 // Constants for register access
 #define AD7327_WRITE_BIT         (1 << 15)
 #define AD7327_REG_SEL1_BIT      (1 << 14)
 #define AD7327_REG_SEL2_BIT      (1 << 13)
 
 // Register identifiers (combination of write bit and register select bits)
 #define AD7327_CONTROL_REG       (AD7327_WRITE_BIT | 0)
 #define AD7327_RANGE1_REG        (AD7327_WRITE_BIT | AD7327_REG_SEL2_BIT)
 #define AD7327_RANGE2_REG        (AD7327_WRITE_BIT | AD7327_REG_SEL1_BIT)
 #define AD7327_SEQUENCE_REG      (AD7327_WRITE_BIT | AD7327_REG_SEL1_BIT | AD7327_REG_SEL2_BIT)
 
 // Control register bit positions
 #define AD7327_CTRL_ADDR_SHIFT   10
 #define AD7327_CTRL_ADDR_MASK    (0x7 << AD7327_CTRL_ADDR_SHIFT)
 #define AD7327_CTRL_MODE_SHIFT   8
 #define AD7327_CTRL_MODE_MASK    (0x3 << AD7327_CTRL_MODE_SHIFT)
 #define AD7327_CTRL_PM_SHIFT     6
 #define AD7327_CTRL_PM_MASK      (0x3 << AD7327_CTRL_PM_SHIFT)
 #define AD7327_CTRL_CODING_BIT   (1 << 5)
 #define AD7327_CTRL_REF_BIT      (1 << 4)
 #define AD7327_CTRL_SEQ_SHIFT    2
 #define AD7327_CTRL_SEQ_MASK     (0x3 << AD7327_CTRL_SEQ_SHIFT)
 #define AD7327_CTRL_ZERO_BIT     (1 << 1)
 
 // Range register bit positions (for both Range1 and Range2)
 #define AD7327_RANGE_SHIFT(ch)   ((ch % 4) * 2)
 #define AD7327_RANGE_MASK(ch)    (0x3 << AD7327_RANGE_SHIFT(ch))
 
 // Conversion result bit positions
 #define AD7327_RESULT_SIGN_BIT   (1 << 12)
 #define AD7327_RESULT_MASK       0x1FFF  // 13 bits total (sign + 12 bits)
 #define AD7327_CHANNEL_ID_MASK   0x7000
 #define AD7327_CHANNEL_ID_SHIFT  12
 
 // Maximum value for a 12-bit + sign ADC
 #define AD7327_MAX_VALUE         0x0FFF  // 4095 (12 bits)
 
 // Power-up and timing constants
 #define AD7327_POWERUP_TIME_US   500     // 500 µs power-up time from full shutdown
 
 /**
  * Constructor for AD7327 ADC
  */
 AD7327::AD7327(int csPin, SPIClass* spiInstance, int resetPin) : 
     _csPin(csPin), 
     _resetPin(resetPin),
     _spiSettings(10000000, MSBFIRST, SPI_MODE0),  // 10 MHz, MSB first, Mode 0
     _spi(spiInstance),
     _currentMode(Mode::SINGLE_ENDED),
     _powerMode(PowerMode::NORMAL),
     _reference(Reference::EXTERNAL),  // Default is external reference
     _coding(Coding::TWOS_COMPLEMENT),
     _sequencerMode(SequencerMode::DISABLED),
     _controlRegValue(0),
     _range1RegValue(0),
     _range2RegValue(0),
     _sequenceRegValue(0)
 {
     // Initialize default ranges to ±10V for all channels
     for (int i = 0; i < 8; i++) {
         _channelRanges[i] = Range::RANGE_10V;
     }
 }
 
 /**
  * Initialize the AD7327 ADC
  */
 bool AD7327::begin() {
     // Initialize SPI (leave this to the user if using custom SPI instance)
     // _spi->begin();
     
     // Configure chip select pin
     pinMode(_csPin, OUTPUT);
     digitalWrite(_csPin, HIGH);  // Deselect chip
     
     // Configure reset pin if available
     if (_resetPin >= 0) {
         pinMode(_resetPin, OUTPUT);
         digitalWrite(_resetPin, HIGH);
     }
     
     // Reset the device
     reset();
     
     // Default configuration
     setInputMode(_currentMode);  // Default is single-ended mode
     setAllRanges(_channelRanges);  // Default is ±10V for all channels
     setReference(_reference);     // Default is external reference
     setCoding(_coding);           // Default is twos complement
     setPowerMode(_powerMode);     // Default is normal mode
     
     return true;
 }
 
 /**
  * Reset the AD7327 ADC
  */
 void AD7327::reset() {
     // Hardware reset if available
     if (_resetPin >= 0) {
         digitalWrite(_resetPin, LOW);
         delayMicroseconds(100);  // Brief pulse
         digitalWrite(_resetPin, HIGH);
         delayMicroseconds(100);  // Wait for stable operation
     }
     
     // Restore default register values
     _controlRegValue = 0;
     _range1RegValue = 0;  // All channels default to ±10V range
     _range2RegValue = 0;
     _sequenceRegValue = 0;
     
     // Write default values to all registers
     writeControlRegister();
     writeRange1Register();
     writeRange2Register();
     writeSequenceRegister();
     
     // Device needs time to stabilize after reset
     delay(5);
 }
 
 /**
  * Set the analog input mode
  */
 void AD7327::setInputMode(Mode mode) {
     _currentMode = mode;
     
     // Update control register with new mode bits
     _controlRegValue &= ~AD7327_CTRL_MODE_MASK;
     _controlRegValue |= (static_cast<uint16_t>(mode) << AD7327_CTRL_MODE_SHIFT);
     
     writeControlRegister();
 }
 
 /**
  * Set the input range for a specific channel
  */
 void AD7327::setRange(uint8_t channel, Range range) {
     if (channel > 7) {
         return;  // Invalid channel
     }
     
     // Store the range for this channel
     _channelRanges[channel] = range;
     
     // Update the appropriate range register
     if (channel < 4) {
         // Channels 0-3 are in Range Register 1
         _range1RegValue &= ~AD7327_RANGE_MASK(channel);
         _range1RegValue |= (static_cast<uint16_t>(range) << AD7327_RANGE_SHIFT(channel));
         writeRange1Register();
     } else {
         // Channels 4-7 are in Range Register 2
         _range2RegValue &= ~AD7327_RANGE_MASK(channel - 4);
         _range2RegValue |= (static_cast<uint16_t>(range) << AD7327_RANGE_SHIFT(channel - 4));
         writeRange2Register();
     }
 }
 
 /**
  * Configure all ranges at once
  */
 void AD7327::setAllRanges(Range ranges[8]) {
     // Copy all ranges to internal storage
     for (int i = 0; i < 8; i++) {
         _channelRanges[i] = ranges[i];
     }
     
     // Build Range Register 1 (channels 0-3)
     _range1RegValue = 0;
     for (int i = 0; i < 4; i++) {
         _range1RegValue |= (static_cast<uint16_t>(ranges[i]) << AD7327_RANGE_SHIFT(i));
     }
     
     // Build Range Register 2 (channels 4-7)
     _range2RegValue = 0;
     for (int i = 0; i < 4; i++) {
         _range2RegValue |= (static_cast<uint16_t>(ranges[i + 4]) << AD7327_RANGE_SHIFT(i));
     }
     
     // Write to both range registers
     writeRange1Register();
     writeRange2Register();
 }
 
 /**
  * Set the reference source
  */
 void AD7327::setReference(Reference ref) {
     _reference = ref;
     
     // Update control register
     if (ref == Reference::INTERNAL) {
         _controlRegValue |= AD7327_CTRL_REF_BIT;
     } else {
         _controlRegValue &= ~AD7327_CTRL_REF_BIT;
     }
     
     writeControlRegister();
     
     // If switching to internal reference, give it time to stabilize
     if (ref == Reference::INTERNAL) {
         delay(1);  // 1 ms should be enough for most cases
     }
 }
 
 /**
  * Set the output coding format
  */
 void AD7327::setCoding(Coding coding) {
     _coding = coding;
     
     // Update control register
     if (coding == Coding::STRAIGHT_BINARY) {
         _controlRegValue |= AD7327_CTRL_CODING_BIT;
     } else {
         _controlRegValue &= ~AD7327_CTRL_CODING_BIT;
     }
     
     writeControlRegister();
 }
 
 /**
  * Set the power mode
  */
 void AD7327::setPowerMode(PowerMode mode) {
     _powerMode = mode;
     
     // Update control register with new power mode bits
     _controlRegValue &= ~AD7327_CTRL_PM_MASK;
     _controlRegValue |= (static_cast<uint16_t>(mode) << AD7327_CTRL_PM_SHIFT);
     
     writeControlRegister();
     
     // If coming out of full shutdown, wait for power-up time
     if (mode == PowerMode::NORMAL && 
         (mode == PowerMode::FULL_SHUTDOWN || mode == PowerMode::AUTOSHUTDOWN)) {
         delayMicroseconds(AD7327_POWERUP_TIME_US);
     }
 }
 
 /**
  * Configure the sequencer
  */
 bool AD7327::configureSequencer(SequencerMode mode, uint8_t finalChannel) {
     if (finalChannel > 7) {
         return false;  // Invalid channel
     }
     
     _sequencerMode = mode;
     
     // Update control register with new sequencer mode bits
     _controlRegValue &= ~AD7327_CTRL_SEQ_MASK;
     _controlRegValue |= (static_cast<uint16_t>(mode) << AD7327_CTRL_SEQ_SHIFT);
     
     // For consecutive sequence mode, set the final channel
     if (mode == SequencerMode::CONSECUTIVE) {
         _controlRegValue &= ~AD7327_CTRL_ADDR_MASK;
         _controlRegValue |= (finalChannel << AD7327_CTRL_ADDR_SHIFT);
     }
     
     writeControlRegister();
     return true;
 }
 
 /**
  * Set sequence of channels to be converted
  */
 bool AD7327::setSequence(uint8_t channelMask) {
     // Store the sequence mask (lower 8 bits)
     _sequenceRegValue = channelMask & 0xFF;
     
     // Write to sequence register
     writeSequenceRegister();
     
     return true;
 }
 
 /**
  * Read a single conversion result from a specific channel
  */
 int16_t AD7327::readChannel(uint8_t channel) {
     if (channel > 7) {
         return 0;  // Invalid channel
     }
     
     // If sequencer is enabled, disable it first
     SequencerMode prevMode = _sequencerMode;
     if (prevMode != SequencerMode::DISABLED && prevMode != SequencerMode::DISABLED_ALT) {
         configureSequencer(SequencerMode::DISABLED);
     }
     
     // Set the channel address in control register
     _controlRegValue &= ~AD7327_CTRL_ADDR_MASK;
     _controlRegValue |= (channel << AD7327_CTRL_ADDR_SHIFT);
     writeControlRegister();
     
     // Perform conversion and read result
     int16_t result = readConversion();
     
     // Restore previous sequencer mode if needed
     if (prevMode != SequencerMode::DISABLED && prevMode != SequencerMode::DISABLED_ALT) {
         configureSequencer(prevMode);
     }
     
     return result;
 }
 
 /**
  * Read the next result from the current sequence
  */
 int16_t AD7327::readNextInSequence(uint8_t* channelOut) {
     // Perform conversion and read result
     uint16_t rawResult = transfer16(0);  // Send dummy data to receive conversion
     
     // Extract the channel ID if requested
     if (channelOut != nullptr) {
         *channelOut = readChannelID(rawResult);
     }
     
     // Return the conversion result
     return extractResult(rawResult);
 }
 
 /**
  * Convert a raw result to a voltage based on the channel's range setting
  */
 float AD7327::resultToVoltage(int16_t result, uint8_t channel) {
     if (channel > 7) {
         return 0.0f;  // Invalid channel
     }
     
     // Get the range for this channel
     Range range = _channelRanges[channel];
     
     // Convert based on range and coding
     float voltage = 0.0f;
     float refVoltage = 2.5f;  // Internal reference is 2.5V
     
     switch (range) {
         case Range::RANGE_10V:
             voltage = (result * 10.0f) / AD7327_MAX_VALUE;
             break;
         case Range::RANGE_5V:
             voltage = (result * 5.0f) / AD7327_MAX_VALUE;
             break;
         case Range::RANGE_2_5V:
             voltage = (result * 2.5f) / AD7327_MAX_VALUE;
             break;
         case Range::RANGE_0_10V:
             // For 0-10V range, result is unipolar
             if (_coding == Coding::STRAIGHT_BINARY) {
                 voltage = (result * 10.0f) / (2 * AD7327_MAX_VALUE);
             } else {
                 // Handle twos complement for unipolar range
                 voltage = ((result + AD7327_MAX_VALUE) * 10.0f) / (2 * AD7327_MAX_VALUE);
             }
             break;
     }
     
     return voltage;
 }
 
 /**
  * Read the temperature from the on-chip temperature indicator
  */
 float AD7327::readTemperature() {
     // Configure for pseudo differential mode (required for temperature sensor)
     Mode prevMode = _currentMode;
     setInputMode(Mode::PSEUDO_DIFFERENTIAL_7);
     
     // Select temperature channel (channel 7 in mode 3)
     _controlRegValue &= ~AD7327_CTRL_ADDR_MASK;
     _controlRegValue |= (7 << AD7327_CTRL_ADDR_SHIFT);
     writeControlRegister();
     
     // Perform conversion at a lower rate for temperature sensing
     delayMicroseconds(100);  // Extra delay for temperature sensing
     int16_t result = readConversion();
     
     // Convert result to temperature (based on datasheet calibration)
     // Note: This is an approximation - refer to datasheet for exact conversion details
     float temp = 25.0f - (result - 4380) / 14.0f;  // Approximate based on datasheet graph
     
     // Restore previous mode
     setInputMode(prevMode);
     
     return temp;
 }
 
 /**
  * Write to the control register
  */
 void AD7327::writeControlRegister() {
     uint16_t data = AD7327_CONTROL_REG | (_controlRegValue & 0x0FFF);
     transfer16(data);
 }
 
 /**
  * Write to Range Register 1
  */
 void AD7327::writeRange1Register() {
     uint16_t data = AD7327_RANGE1_REG | (_range1RegValue & 0x00FF);
     transfer16(data);
 }
 
 /**
  * Write to Range Register 2
  */
 void AD7327::writeRange2Register() {
     uint16_t data = AD7327_RANGE2_REG | (_range2RegValue & 0x00FF);
     transfer16(data);
 }
 
 /**
  * Write to the sequence register
  */
 void AD7327::writeSequenceRegister() {
     uint16_t data = AD7327_SEQUENCE_REG | (_sequenceRegValue & 0x00FF);
     transfer16(data);
 }
 
 /**
  * Perform a conversion and read the result
  */
 int16_t AD7327::readConversion() {
     uint16_t rawResult = transfer16(0);  // Send dummy data to receive conversion
     delayMicroseconds(5);
     return extractResult(rawResult);
 }
 
 /**
  * Extract the channel ID from a raw result
  */
 uint8_t AD7327::readChannelID(uint16_t rawResult) {
     return (rawResult & AD7327_CHANNEL_ID_MASK) >> AD7327_CHANNEL_ID_SHIFT;
 }
 
 /**
  * Extract the conversion result from a raw value
  */
 int16_t AD7327::extractResult(uint16_t rawResult) {
     int16_t result = rawResult & AD7327_RESULT_MASK;
     
     // If using twos complement, sign-extend the 13-bit value to 16 bits
     if (_coding == Coding::TWOS_COMPLEMENT && (result & AD7327_RESULT_SIGN_BIT)) {
         result |= 0xE000;  // Set the upper 3 bits
     }
     
     return result;
 }
 
 /**
  * Send and receive 16 bits over SPI
  */
 uint16_t AD7327::transfer16(uint16_t data) {
     _spi->beginTransaction(_spiSettings);
     digitalWrite(_csPin, LOW);
     
     // Send 16 bits MSB first
     uint16_t received = 0;
     received = (_spi->transfer(data >> 8) << 8);
     received |= _spi->transfer(data & 0xFF);
     
     digitalWrite(_csPin, HIGH);
     _spi->endTransaction();
     
     return received;
 }