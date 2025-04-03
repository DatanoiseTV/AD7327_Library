 /**
  * @file AD7327.h
  * @brief Header file for the AD7327 12-bit plus sign ADC library
  * @author DatanoiseTV / Corrected by AI Assistant
  * @version 1.2.0 
  * @date 2023-10-27 // Corrected Date
  *
  * This library provides an interface to the Analog Devices AD7327 12-bit plus sign
  * ADC with true bipolar input, 8 channels, and software-selectable input ranges.
  *
  * Supports custom SPI instances for multi-SPI port microcontrollers.
  */

  #ifndef AD7327_H
  #define AD7327_H
 
  #include <Arduino.h>
  #include <SPI.h>
 
  /**
   * @brief Class for interfacing with the AD7327 ADC
   */
  class AD7327 {
  public:
      // --- Enums remain the same ---
       /**
       * @brief Analog input mode definitions
       */
      enum class Mode {
          SINGLE_ENDED = 0b00,      // 8 single-ended inputs
          PSEUDO_DIFFERENTIAL_4 = 0b01, // 4 pseudo differential inputs (VIN- = VIN1, VIN3, VIN5, VIN7)
          TRUE_DIFFERENTIAL = 0b10, // 4 true differential input pairs (VIN0/1, VIN2/3, VIN4/5, VIN6/7) - Corrected Name
          PSEUDO_DIFFERENTIAL_7 = 0b11  // 7 pseudo differential inputs (VIN- = VIN7)
      };
 
      /**
       * @brief Input range definitions
       */
      enum class Range {
          RANGE_10V = 0b00,    // ±10V range
          RANGE_5V = 0b01,     // ±5V range
          RANGE_2_5V = 0b10,   // ±2.5V range
          RANGE_0_10V = 0b11   // 0V to +10V range
      };
 
      /**
       * @brief Coding format definitions
       */
      enum class Coding {
          TWOS_COMPLEMENT = 0, // Two's complement output coding (Default)
          STRAIGHT_BINARY = 1  // Straight binary output coding
      };
 
      /**
       * @brief Power mode definitions
       */
      enum class PowerMode {
          NORMAL = 0b00,       // Normal operation mode (Default)
          AUTOSTANDBY = 0b01,  // Auto-standby mode (reference remains powered)
          AUTOSHUTDOWN = 0b10, // Auto-shutdown mode
          FULL_SHUTDOWN = 0b11 // Full shutdown mode
      };
 
      /**
       * @brief Sequencer mode definitions
       */
      enum class SequencerMode {
          DISABLED = 0b00,     // Sequencer disabled (Default)
          PROGRAMMED = 0b01,   // Programmed sequence (Uses Sequence Register)
          CONSECUTIVE = 0b10,  // Consecutive channels (Channel 0 up to Addr Bits)
          DISABLED_ALT = 0b11  // Alternate way to disable sequencer
      };
 
      /**
       * @brief Reference source definitions
       */
      enum class Reference {
          EXTERNAL = 0,      // External reference (Default after power-up)
          INTERNAL = 1       // Internal 2.5V reference
      };
 
      // --- Constructor and begin() ---
      AD7327(int csPin, SPIClass* spiInstance = &SPI, int resetPin = -1);
      bool begin();
      void reset();
 
      // --- Configuration Setters ---
      void setInputMode(Mode mode);
      void setRange(uint8_t channel, Range range);
      void setAllRanges(Range ranges[8]);
      void setReference(Reference ref, float externalVoltage = 2.5f); // Added externalVoltage param
      void setCoding(Coding coding);
      void setPowerMode(PowerMode mode);
      bool configureSequencer(SequencerMode mode, uint8_t finalChannel = 7); // Default finalChannel to max
      bool setSequence(uint8_t channelMask);
 
      // --- Reading Data ---
      int16_t readChannel(uint8_t channel);
      int16_t readNextInSequence(uint8_t* channelOut = nullptr); // Reads result from previous cycle
      float resultToVoltage(int16_t result, uint8_t channel);
      float readTemperature(); // Returns temperature in Celsius
 
  protected:
      // SPI and pin management
      int _csPin;
      int _resetPin;
      SPISettings _spiSettings;
      SPIClass* _spi;
 
      // Current configuration state
      Mode _currentMode;
      Range _channelRanges[8];
      PowerMode _powerMode;
      PowerMode _previousPowerMode; // Added to track state for delays
      Reference _reference;
      Coding _coding;
      SequencerMode _sequencerMode;
      float _refVoltage; // Store the actual reference voltage being used
 
      // Shadow registers (store the intended state)
      uint16_t _controlRegValue;
      uint16_t _range1RegValue;
      uint16_t _range2RegValue;
      uint16_t _sequenceRegValue;
 
      // Internal helper methods
      void writeControlRegister();
      void writeRange1Register();
      void writeRange2Register();
      void writeSequenceRegister();
      uint16_t transfer16(uint16_t data);
      uint8_t extractChannelID(uint16_t rawResult); // Renamed for clarity
      int16_t extractResult(uint16_t rawResult);
 
      // Corrected method to start a conversion (by writing control reg)
      // and retrieve the result later.
      uint16_t startConversionAndReadPrevious(uint16_t command);
 
  private:
      // Prevent copying
      AD7327(const AD7327&);
      AD7327& operator=(const AD7327&);
  };
 
  #endif // AD7327_H