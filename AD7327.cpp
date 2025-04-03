 /**
  * @file AD7327.cpp
  * @brief Implementation file for the AD7327 12-bit plus sign ADC library
  * @author DaatanoiseTV / Corrected by AI Assistant
  * @version 1.2.0
  * @date 2023-10-27 // Corrected Date
  */

  #include "AD7327.h"

  // Constants for register access (Corrected based on Table 8)
  #define AD7327_WRITE_BIT         (1 << 15) // 0x8000
  #define AD7327_REG_SEL1_BIT      (1 << 14) // 0x4000
  #define AD7327_REG_SEL2_BIT      (1 << 13) // 0x2000
 
  // Register write command identifiers (MSBs for DIN)
  #define AD7327_CMD_CONTROL_REG   (AD7327_WRITE_BIT | 0                   | 0                   ) // 0x8000
  #define AD7327_CMD_RANGE1_REG    (AD7327_WRITE_BIT | 0                   | AD7327_REG_SEL2_BIT ) // 0xA000
  #define AD7327_CMD_RANGE2_REG    (AD7327_WRITE_BIT | AD7327_REG_SEL1_BIT | 0                   ) // 0xC000
  #define AD7327_CMD_SEQUENCE_REG  (AD7327_WRITE_BIT | AD7327_REG_SEL1_BIT | AD7327_REG_SEL2_BIT ) // 0xE000
  #define AD7327_CMD_NO_WRITE      (0) // Use to read data without writing registers
 
  // Control register bit positions/masks
  #define AD7327_CTRL_ADDR_SHIFT   10
  #define AD7327_CTRL_ADDR_MASK    (0x7 << AD7327_CTRL_ADDR_SHIFT) // Bits 12, 11, 10
  #define AD7327_CTRL_MODE_SHIFT   8
  #define AD7327_CTRL_MODE_MASK    (0x3 << AD7327_CTRL_MODE_SHIFT) // Bits 9, 8
  #define AD7327_CTRL_PM_SHIFT     6
  #define AD7327_CTRL_PM_MASK      (0x3 << AD7327_CTRL_PM_SHIFT)   // Bits 7, 6
  #define AD7327_CTRL_CODING_BIT   (1 << 5)                       // Bit 5
  #define AD7327_CTRL_REF_BIT      (1 << 4)                       // Bit 4
  #define AD7327_CTRL_SEQ_SHIFT    2
  #define AD7327_CTRL_SEQ_MASK     (0x3 << AD7327_CTRL_SEQ_SHIFT)  // Bits 3, 2
  #define AD7327_CTRL_ZERO_MASK    (1 << 1)                       // Bit 1 (Should be 0)
  // Bit 0 is unused (0)
 
  // Range register bit positions (2 bits per channel)
  #define AD7327_RANGE_SHIFT(ch_mod_4) ((ch_mod_4) * 2) // Shift for channel 0..3 within its register
  #define AD7327_RANGE_MASK(ch_mod_4)  (0x3 << AD7327_RANGE_SHIFT(ch_mod_4))
 
  // Sequence register bit positions (1 bit per channel)
  #define AD7327_SEQ_CH_BIT(ch)    (1 << ch) // Bit 0 for Ch0, Bit 7 for Ch7
 
  // Conversion result bit interpretation
  #define AD7327_DOUT_CHANNEL_ID_MASK  0xE000 // Bits 15, 14, 13
  #define AD7327_DOUT_CHANNEL_ID_SHIFT 13
  #define AD7327_DOUT_SIGN_BIT         (1 << 12) // Bit 12 is the sign bit
  #define AD7327_DOUT_VALUE_MASK       0x0FFF // Bits 11..0 are magnitude (12 bits)
  #define AD7327_RESULT_FULL_MASK      0x1FFF // Sign + Magnitude (13 bits)
 
  // Constants for voltage conversion
  #define AD7327_BIPOLAR_CODES   8192.0f // 2^13 (for +/- ranges)
  #define AD7327_UNIPOLAR_CODES  4096.0f // 2^12 (for 0-X range, straight binary)
 
  // Power-up and timing constants
  #define AD7327_POWERUP_TIME_US   500     // 500 µs power-up time from full shutdown/autoshutdown
  #define AD7327_STANDBY_WAKE_US   1       // Very short wake from standby (dominated by SPI)
  #define AD7327_INTERNAL_REF_STABLE_MS 1 // Time for internal ref to stabilize
 
  // Constructor
  AD7327::AD7327(int csPin, SPIClass* spiInstance, int resetPin) :
      _csPin(csPin),
      _resetPin(resetPin),
      _spiSettings(10000000, MSBFIRST, SPI_MODE0), // Max 10 MHz, MSB First, Mode 0
      _spi(spiInstance),
      _currentMode(Mode::SINGLE_ENDED),
      _powerMode(PowerMode::NORMAL),
      _previousPowerMode(PowerMode::FULL_SHUTDOWN), // Assume starts powered down for delay logic
      _reference(Reference::EXTERNAL),
      _coding(Coding::TWOS_COMPLEMENT),
      _sequencerMode(SequencerMode::DISABLED),
      _refVoltage(2.5f), // Default assumption if external ref selected initially
      _controlRegValue(0),
      _range1RegValue(0),
      _range2RegValue(0),
      _sequenceRegValue(0)
  {
      // Initialize default ranges (±10V) in shadow registers
      for (int i = 0; i < 8; i++) {
          _channelRanges[i] = Range::RANGE_10V;
          // The actual register values (_range1RegValue, _range2RegValue) remain 0
          // as RANGE_10V corresponds to 0b00.
      }
  }
 
  // Initialize the AD7327 ADC
  bool AD7327::begin() {
      // Check if SPI instance is valid
      if (_spi == nullptr) {
         return false;
      }
 
      // Let user handle _spi->begin() externally if needed
 
      // Configure chip select pin
      pinMode(_csPin, OUTPUT);
      digitalWrite(_csPin, HIGH); // Deselect chip
 
      // Configure reset pin if available
      if (_resetPin >= 0) {
          pinMode(_resetPin, OUTPUT);
          digitalWrite(_resetPin, HIGH);
      }
 
      // Reset the device to ensure known state
      reset(); // This writes the default shadow register values
 
      // Apply initial configuration (redundant if defaults are kept, but safe)
      // Note: reset() already wrote the defaults. This is just for explicit setting.
      // setInputMode(_currentMode);
      // setAllRanges(_channelRanges);
      // setReference(_reference, _refVoltage);
      // setCoding(_coding);
      // setPowerMode(_powerMode); // Power mode handled by reset/wake logic
 
      return true;
  }
 
  // Reset the AD7327 ADC
  void AD7327::reset() {
      // Hardware reset if available
      if (_resetPin >= 0) {
          digitalWrite(_resetPin, LOW);
          delayMicroseconds(10); // Datasheet doesn't specify pulse width, 10us is safe
          digitalWrite(_resetPin, HIGH);
          delayMicroseconds(10); // Wait for stable operation
      }
 
      // Restore default shadow register values explicitly
      _controlRegValue = 0; // Defaults: Addr=0, Mode=SE, PM=Normal, Coding=2sComp, Ref=Ext, Seq=Disabled
      _range1RegValue = 0;  // All channels default to ±10V range (0b00)
      _range2RegValue = 0;
      _sequenceRegValue = 0; // No channels in sequence
 
      // Restore default internal state
      _currentMode = Mode::SINGLE_ENDED;
      _powerMode = PowerMode::NORMAL;
      _previousPowerMode = PowerMode::FULL_SHUTDOWN; // Assume we woke from shutdown
      _reference = Reference::EXTERNAL;
      _coding = Coding::TWOS_COMPLEMENT;
      _sequencerMode = SequencerMode::DISABLED;
      _refVoltage = 2.5f; // Default assumption
      for (int i = 0; i < 8; i++) {
          _channelRanges[i] = Range::RANGE_10V;
      }
 
      // Write default values to all registers
      // Need to ensure power mode allows writing
      _controlRegValue &= ~AD7327_CTRL_PM_MASK; // Force normal mode temporarily
      _controlRegValue |= (static_cast<uint16_t>(PowerMode::NORMAL) << AD7327_CTRL_PM_SHIFT);
      writeControlRegister(); // Write defaults with normal power
      delayMicroseconds(AD7327_POWERUP_TIME_US); // Ensure device is fully awake
 
      writeRange1Register();
      writeRange2Register();
      writeSequenceRegister();
 
      // Set the desired initial power mode *after* reset configuration
      setPowerMode(PowerMode::NORMAL); // Or user's desired default
  }
 
 
  // --- Configuration Setters ---
 
  void AD7327::setInputMode(Mode mode) {
      _currentMode = mode;
      _controlRegValue &= ~AD7327_CTRL_MODE_MASK;
      _controlRegValue |= (static_cast<uint16_t>(mode) << AD7327_CTRL_MODE_SHIFT);
      writeControlRegister();
  }
 
  void AD7327::setRange(uint8_t channel, Range range) {
      if (channel > 7) return;
 
      _channelRanges[channel] = range;
      uint8_t channel_in_reg = channel % 4;
      uint16_t range_bits = static_cast<uint16_t>(range);
 
      if (channel < 4) { // Range Register 1 (Channels 0-3)
          _range1RegValue &= ~AD7327_RANGE_MASK(channel_in_reg); // Clear bits
          _range1RegValue |= (range_bits << AD7327_RANGE_SHIFT(channel_in_reg)); // Set bits
          writeRange1Register();
      } else { // Range Register 2 (Channels 4-7)
          _range2RegValue &= ~AD7327_RANGE_MASK(channel_in_reg); // Clear bits
          _range2RegValue |= (range_bits << AD7327_RANGE_SHIFT(channel_in_reg)); // Set bits
          writeRange2Register();
      }
  }
 
  void AD7327::setAllRanges(Range ranges[8]) {
      _range1RegValue = 0;
      _range2RegValue = 0;
      for (int i = 0; i < 8; i++) {
          _channelRanges[i] = ranges[i];
          uint8_t channel_in_reg = i % 4;
          uint16_t range_bits = static_cast<uint16_t>(ranges[i]);
          if (i < 4) {
              _range1RegValue |= (range_bits << AD7327_RANGE_SHIFT(channel_in_reg));
          } else {
              _range2RegValue |= (range_bits << AD7327_RANGE_SHIFT(channel_in_reg));
          }
      }
      writeRange1Register();
      writeRange2Register();
  }
 
  void AD7327::setReference(Reference ref, float externalVoltage) {
      _reference = ref;
      if (ref == Reference::INTERNAL) {
          _controlRegValue |= AD7327_CTRL_REF_BIT;
          _refVoltage = 2.5f; // Internal reference is 2.5V
      } else {
          _controlRegValue &= ~AD7327_CTRL_REF_BIT;
          _refVoltage = externalVoltage; // Store provided external voltage
      }
      writeControlRegister();
 
      // If switching to internal reference, give it time to stabilize
      if (ref == Reference::INTERNAL) {
          delay(AD7327_INTERNAL_REF_STABLE_MS);
      }
  }
 
  void AD7327::setCoding(Coding coding) {
      _coding = coding;
      if (coding == Coding::STRAIGHT_BINARY) {
          _controlRegValue |= AD7327_CTRL_CODING_BIT;
      } else {
          _controlRegValue &= ~AD7327_CTRL_CODING_BIT;
      }
      writeControlRegister();
  }
 
  void AD7327::setPowerMode(PowerMode mode) {
      // Store previous mode *before* changing shadow register
      _previousPowerMode = _powerMode;
      _powerMode = mode;
 
      _controlRegValue &= ~AD7327_CTRL_PM_MASK;
      _controlRegValue |= (static_cast<uint16_t>(mode) << AD7327_CTRL_PM_SHIFT);
      writeControlRegister(); // This write takes effect on 15th SCLK edge
 
      // If waking up from a low power mode, add necessary delay *after* the write
      // The actual wake-up starts on the CS rising edge following the write command.
      // However, we need to ensure enough time passes *before* the *next* CS falling edge.
      if (_powerMode == PowerMode::NORMAL) {
         if (_previousPowerMode == PowerMode::FULL_SHUTDOWN || _previousPowerMode == PowerMode::AUTOSHUTDOWN) {
              delayMicroseconds(AD7327_POWERUP_TIME_US);
          } else if (_previousPowerMode == PowerMode::AUTOSTANDBY) {
              delayMicroseconds(AD7327_STANDBY_WAKE_US); // Usually negligible
          }
      }
  }
 
  bool AD7327::configureSequencer(SequencerMode mode, uint8_t finalChannel) {
      if (finalChannel > 7) return false;
 
      _sequencerMode = mode;
      _controlRegValue &= ~AD7327_CTRL_SEQ_MASK;
      _controlRegValue |= (static_cast<uint16_t>(mode) << AD7327_CTRL_SEQ_SHIFT);
 
      // For consecutive sequence mode, set the final channel in address bits
      if (mode == SequencerMode::CONSECUTIVE) {
          _controlRegValue &= ~AD7327_CTRL_ADDR_MASK;
          _controlRegValue |= (finalChannel << AD7327_CTRL_ADDR_SHIFT);
      }
      // For other modes, address bits select single channel if Seq disabled,
      // or are ignored if Programmed sequence is active.
 
      writeControlRegister();
      return true;
  }
 
  bool AD7327::setSequence(uint8_t channelMask) {
      _sequenceRegValue = channelMask & 0xFF; // Lower 8 bits for 8 channels
      writeSequenceRegister();
      return true;
  }
 
  // --- Reading Data ---
 
  /**
   * @brief Read a single conversion result from a specific channel.
   * Assumes the sequencer is NOT active or manages it temporarily.
   * @param channel Channel to read (0-7)
   * @return int16_t Conversion result (13-bit signed value)
   */
  int16_t AD7327::readChannel(uint8_t channel) {
      if (channel > 7) return 0; // Invalid channel
 
      // --- Prepare Control Register for this read ---
      // Ensure sequencer is disabled for single channel read
      _controlRegValue &= ~AD7327_CTRL_SEQ_MASK;
      _controlRegValue |= (static_cast<uint16_t>(SequencerMode::DISABLED) << AD7327_CTRL_SEQ_SHIFT);
 
      // Set the channel address
      _controlRegValue &= ~AD7327_CTRL_ADDR_MASK;
      _controlRegValue |= (channel << AD7327_CTRL_ADDR_SHIFT);
 
      // --- Perform the two required transactions ---
      // Transaction 1: Write control register to select channel & start conversion. Ignore DOUT.
      startConversionAndReadPrevious(AD7327_CMD_CONTROL_REG | (_controlRegValue & 0x1FFF));
 
      // Transaction 2: Write dummy data (No Write command) to retrieve the result.
      uint16_t rawResult = startConversionAndReadPrevious(AD7327_CMD_NO_WRITE);
 
      // Extract and return the 13-bit result
      return extractResult(rawResult);
  }
 
  /**
   * @brief Reads the next result when operating in a sequence mode.
   * Call this repeatedly to cycle through the sequence.
   * The result returned is from the *previous* channel in the sequence.
   * @param channelOut Optional pointer to store the channel number the result belongs to.
   * @return int16_t Conversion result (13-bit signed value)
   */
  int16_t AD7327::readNextInSequence(uint8_t* channelOut) {
      // In sequence mode, each transaction reads the previous result
      // and automatically starts the next conversion in the sequence.
      // We just need to send a "No Write" command to clock out the data.
      uint16_t rawResult = startConversionAndReadPrevious(AD7327_CMD_NO_WRITE);
 
      if (channelOut != nullptr) {
          *channelOut = extractChannelID(rawResult);
      }
 
      return extractResult(rawResult);
  }
 
 
  /**
   * @brief Convert a raw 13-bit result to voltage based on channel's range and coding.
   * @param result The 13-bit signed result from extractResult().
   * @param channel The channel number (0-7) the result is from.
   * @return float Voltage in volts.
   */
  float AD7327::resultToVoltage(int16_t result, uint8_t channel) {
      if (channel > 7) return 0.0f;
 
      Range range = _channelRanges[channel];
      float span = 0.0f;
      float offset = 0.0f;
      float codes = AD7327_BIPOLAR_CODES; // Default for bipolar
 
      switch (range) {
          case Range::RANGE_10V: // ±10V
              span = 20.0f;
              offset = -10.0f;
              codes = AD7327_BIPOLAR_CODES;
              break;
          case Range::RANGE_5V: // ±5V
              span = 10.0f;
              offset = -5.0f;
              codes = AD7327_BIPOLAR_CODES;
              break;
          case Range::RANGE_2_5V: // ±2.5V
              span = 5.0f;
              offset = -2.5f;
              codes = AD7327_BIPOLAR_CODES;
              break;
          case Range::RANGE_0_10V: // 0V to +10V
              span = 10.0f;
              offset = 0.0f;
              if (_coding == Coding::STRAIGHT_BINARY) {
                  codes = AD7327_UNIPOLAR_CODES;
                  // For straight binary unipolar, the raw 'result' from extractResult
                  // (which does sign extension for 2's comp) needs adjustment.
                  // We only want the lower 12 bits as unsigned.
                  uint16_t unsigned_result = result & 0x0FFF;
                  return (unsigned_result / codes) * span * (10.0f / (2.5f * 4)) + offset; // (Vref=2.5, range 0-10V => 4*Vref)
                 // return ( (float)unsigned_result / codes) * span + offset; // Simpler calculation? check datasheet Vref scaling
 
              } else { // Twos Complement
                  codes = AD7327_BIPOLAR_CODES;
                  // Result is -4096 to +4095 centered at 5V
                   return ( (float)result / codes) * span + (span / 2.0f) + offset; // Center + offset
              }
              break;
      }
 
      // Adjust span based on actual reference voltage vs. nominal 2.5V internal
      span *= (_refVoltage / 2.5f);
      offset *= (_refVoltage / 2.5f); // Offset scales too for bipolar
 
      // General bipolar calculation (and unipolar Twos Comp)
      // Voltage = (Result / Total Codes) * Span + Offset
      // Need to map the signed result (-4096 to +4095) to the voltage range
      // Midpoint code is near 0. Full scale positive is +4095, Full scale negative is -4096
      // Positive voltage = (result / 4095.0f) * (span / 2.0f) + MidpointVoltage
      // Negative voltage = (result / 4096.0f) * (span / 2.0f) + MidpointVoltage
      // Simplified: map -4096..+4095 to Offset .. Offset+Span
 
      return ( ((float)result + (codes/2.0f)) / codes ) * span + offset; // Map signed code to span
  }
 
 
  /**
   * @brief Read the temperature from the on-chip indicator.
   * Note: Requires specific setup and may be less accurate without calibration.
   * Throughput should be reduced when reading temperature (e.g., < 50 kSPS).
   * @return float Temperature in degrees Celsius (approximate).
   */
  float AD7327::readTemperature() {
      // --- Store previous state ---
      Mode previousMode = _currentMode;
      uint16_t previousControl = _controlRegValue;
 
      // --- Configure for Temperature Reading ---
      // Mode must be Pseudo Differential with VIN- = VIN7
      setInputMode(Mode::PSEUDO_DIFFERENTIAL_7);
      // Address must be 7
      _controlRegValue &= ~AD7327_CTRL_ADDR_MASK;
      _controlRegValue |= (7 << AD7327_CTRL_ADDR_SHIFT);
      // Ensure sequencer is disabled
      _controlRegValue &= ~AD7327_CTRL_SEQ_MASK;
      _controlRegValue |= (static_cast<uint16_t>(SequencerMode::DISABLED) << AD7327_CTRL_SEQ_SHIFT);
 
      // --- Perform Reads ---
      // Transaction 1: Write control register to select temp sensor & start conversion. Ignore DOUT.
      startConversionAndReadPrevious(AD7327_CMD_CONTROL_REG | (_controlRegValue & 0x1FFF));
 
      // Add extra delay recommended for temperature sensor settling/conversion
      delayMicroseconds(30); // Datasheet suggests lower throughput (~30kSPS)
 
      // Transaction 2: Write dummy data to retrieve the temperature result.
      uint16_t rawResult = startConversionAndReadPrevious(AD7327_CMD_NO_WRITE);
 
      // Extract the 13-bit result
      int16_t tempCode = extractResult(rawResult);
 
      // --- Restore previous state ---
      // Restore control register value (implicitly restores mode, addr, seq)
      _controlRegValue = previousControl;
      writeControlRegister();
      _currentMode = previousMode; // Restore internal state variable
 
 
      // --- Convert code to temperature ---
      // Formula derived from datasheet Figure 46/47 needs range context.
      // Assuming default ±10V range for the temp channel if not set otherwise.
      // Figure 46 (±10V, Vref=2.5V internal): Code ~4380 @ 25°C. Slope approx -14 LSB/°C.
      // Figure 47 (±2.5V, Vref=2.5V internal): Code ~5250 @ 25°C? Slope looks steeper.
      // Using Figure 46 formula as a basis:
      float temperature = 25.0f - ( (float)tempCode - 4380.0f ) / 14.0f; // Approximate
 
      return temperature;
  }
 
 
  // --- Internal Helper Methods ---
 
  // Write Control Register (ensures Bit 1 is 0)
  void AD7327::writeControlRegister() {
      uint16_t data_payload = _controlRegValue & 0x1FFF; // Get lower 13 bits
      data_payload &= ~AD7327_CTRL_ZERO_MASK; // Ensure ZERO bit (Bit 1) is 0
      startConversionAndReadPrevious(AD7327_CMD_CONTROL_REG | data_payload);
  }
 
  // Write Range Register 1
  void AD7327::writeRange1Register() {
      // Range registers only use lower 8 bits for data payload
      startConversionAndReadPrevious(AD7327_CMD_RANGE1_REG | (_range1RegValue & 0x00FF));
  }
 
  // Write Range Register 2
  void AD7327::writeRange2Register() {
      startConversionAndReadPrevious(AD7327_CMD_RANGE2_REG | (_range2RegValue & 0x00FF));
  }
 
  // Write Sequence Register
  void AD7327::writeSequenceRegister() {
      startConversionAndReadPrevious(AD7327_CMD_SEQUENCE_REG | (_sequenceRegValue & 0x00FF));
  }
 
  /**
   * @brief Performs a 16-bit SPI transfer. Sends command, reads previous result.
   * This is the core function that handles the ADC's pipeline.
   * @param command The 16-bit command to send (includes register address/write bit).
   * @return uint16_t The 16-bit data read from DOUT (result from previous cycle).
   */
  uint16_t AD7327::startConversionAndReadPrevious(uint16_t command) {
     return transfer16(command); // Just wrap the basic transfer for now
     // In a non-blocking scenario, this would queue the command
     // and retrieve the result from the *last* completed transfer.
  }
 
  // Basic SPI 16-bit transfer
  uint16_t AD7327::transfer16(uint16_t data) {
     uint16_t received = 0;
     _spi->beginTransaction(_spiSettings);
     digitalWrite(_csPin, LOW);
     // Datasheet Figure 2 timing: t2 (CS to SCLK setup) >= 20ns for Vcc=5V
     // delayMicroseconds(1); // Small delay might be needed for some MCUs/speeds
 
     received = _spi->transfer16(data); // Use hardware transfer16 if available
 
     // If transfer16 isn't available or reliable:
     // received = (_spi->transfer(data >> 8) << 8);
     // received |= _spi->transfer(data & 0xFF);
 
     digitalWrite(_csPin, HIGH);
     _spi->endTransaction();
     return received;
  }
 
  // Extract Channel ID from DOUT raw data
  uint8_t AD7327::extractChannelID(uint16_t rawResult) {
      // Channel ID is in bits 15, 14, 13 according to Figure 53
      return (rawResult & AD7327_DOUT_CHANNEL_ID_MASK) >> AD7327_DOUT_CHANNEL_ID_SHIFT;
  }
 
  // Extract 13-bit conversion result, handle sign extension
  int16_t AD7327::extractResult(uint16_t rawResult) {
      // Result is Sign (Bit 12) + Value (Bits 11..0)
      int16_t result = rawResult & AD7327_RESULT_FULL_MASK; // Mask out Channel ID bits (15,14,13)
 
      // Sign extend if Twos Complement and negative (Bit 12 is set)
      if (_coding == Coding::TWOS_COMPLEMENT && (result & AD7327_DOUT_SIGN_BIT)) {
          result |= 0xE000; // Set upper 3 bits (15, 14, 13) to make it 16-bit negative
      }
 
      return result;
  }