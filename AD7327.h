/**
 * @file AD7327.h
 * @brief Header file for the AD7327 12-bit plus sign ADC library
 * @author DatanoiseTV
 * @version 1.1.0
 * @date 2025-04-03
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
     /**
      * @brief Analog input mode definitions
      */
     enum class Mode {
         SINGLE_ENDED = 0b00,      // 8 single-ended inputs
         PSEUDO_DIFFERENTIAL_4 = 0b01, // 4 pseudo differential inputs
         TRUE_DIFFERENTIAL = 0b10, // 4 true differential input pairs
         PSEUDO_DIFFERENTIAL_7 = 0b11  // 7 pseudo differential inputs
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
         TWOS_COMPLEMENT = 0, // Two's complement output coding
         STRAIGHT_BINARY = 1  // Straight binary output coding
     };
 
     /**
      * @brief Power mode definitions
      */
     enum class PowerMode {
         NORMAL = 0b00,       // Normal operation mode
         AUTOSTANDBY = 0b01,  // Auto-standby mode (reference remains powered)
         AUTOSHUTDOWN = 0b10, // Auto-shutdown mode
         FULL_SHUTDOWN = 0b11 // Full shutdown mode
     };
 
     /**
      * @brief Sequencer mode definitions
      */
     enum class SequencerMode {
         DISABLED = 0b00,     // Sequencer disabled
         PROGRAMMED = 0b01,   // Programmed sequence
         CONSECUTIVE = 0b10,  // Consecutive channels
         DISABLED_ALT = 0b11  // Alternate way to disable sequencer
     };
 
     /**
      * @brief Reference source definitions
      */
     enum class Reference {
         EXTERNAL = 0,      // External reference (default after power-up)
         INTERNAL = 1       // Internal 2.5V reference
     };
 
     /**
      * @brief Constructor for AD7327 ADC
      * 
      * @param csPin Chip select pin
      * @param spiInstance Pointer to an SPI instance (default is &SPI)
      * @param resetPin Reset pin (optional, set to -1 if not used)
      */
     AD7327(int csPin, SPIClass* spiInstance = &SPI, int resetPin = -1);
 
     /**
      * @brief Initialize the AD7327 ADC
      * 
      * @return true if initialization succeeds, false otherwise
      */
     bool begin();
 
     /**
      * @brief Reset the AD7327 ADC
      * 
      * Performs a software reset if possible, or a hardware reset if the reset pin is connected
      */
     void reset();
 
     /**
      * @brief Set the analog input mode
      * 
      * @param mode The input mode (single-ended, differential, etc.)
      */
     void setInputMode(Mode mode);
 
     /**
      * @brief Set the input range for a specific channel
      * 
      * @param channel Channel number (0-7)
      * @param range Input range to set
      */
     void setRange(uint8_t channel, Range range);
 
     /**
      * @brief Configure all ranges at once
      * 
      * @param ranges Array of 8 Range values for each channel
      */
     void setAllRanges(Range ranges[8]);
 
     /**
      * @brief Set the reference source
      * 
      * @param ref Reference source (internal or external)
      */
     void setReference(Reference ref);
 
     /**
      * @brief Set the output coding format
      * 
      * @param coding Output coding format
      */
     void setCoding(Coding coding);
 
     /**
      * @brief Set the power mode
      * 
      * @param mode Power mode to set
      */
     void setPowerMode(PowerMode mode);
 
     /**
      * @brief Configure the sequencer
      * 
      * @param mode Sequencer mode
      * @param finalChannel Final channel in consecutive sequence (0-7)
      * @return true if configuration succeeds, false otherwise
      */
     bool configureSequencer(SequencerMode mode, uint8_t finalChannel = 0);
 
     /**
      * @brief Set sequence of channels to be converted
      * 
      * @param channelMask Bitmask of channels to include in sequence (bit 0 for channel 0, etc.)
      * @return true if setting succeeds, false otherwise
      */
     bool setSequence(uint8_t channelMask);
 
     /**
      * @brief Read a single conversion result
      * 
      * @param channel Channel to read (0-7)
      * @return int16_t Conversion result (13-bit signed value)
      */
     int16_t readChannel(uint8_t channel);
     
     /**
      * @brief Read the next result from the current sequence
      * 
      * @param channelOut Optional pointer to store the channel number
      * @return int16_t Conversion result (13-bit signed value)
      */
     int16_t readNextInSequence(uint8_t* channelOut = nullptr);
 
     /**
      * @brief Get the voltage from a conversion result
      * 
      * @param result Conversion result from readChannel or readNextInSequence
      * @param channel Channel the conversion was from
      * @return float Voltage in volts
      */
     float resultToVoltage(int16_t result, uint8_t channel);
 
     /**
      * @brief Read the temperature from the on-chip temperature indicator
      * 
      * @return float Temperature in degrees Celsius
      */
     float readTemperature();
 
 protected:
     // SPI and pin management
     int _csPin;
     int _resetPin;
     SPISettings _spiSettings;
     SPIClass* _spi;  // Pointer to SPI instance
 
     // Current configuration
     Mode _currentMode;
     Range _channelRanges[8];
     PowerMode _powerMode;
     Reference _reference;
     Coding _coding;
     SequencerMode _sequencerMode;
     
     // Register values
     uint16_t _controlRegValue;
     uint16_t _range1RegValue;
     uint16_t _range2RegValue;
     uint16_t _sequenceRegValue;
 
     // Private methods for register access
     void writeControlRegister();
     void writeRange1Register();
     void writeRange2Register();
     void writeSequenceRegister();
     int16_t readConversion();
     uint8_t readChannelID(uint16_t rawResult);
     int16_t extractResult(uint16_t rawResult);
 
     // SPI communication
     virtual uint16_t transfer16(uint16_t data);
 };
 
 #endif // AD7327_H