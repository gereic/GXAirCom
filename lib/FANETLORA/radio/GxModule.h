#ifndef _GXMODULE_H
#define _GXMODULE_H

#define GXMODULE_NC                                 (0xFF)

//#include "TypeDef.h"

#include <SPI.h>
#include <Arduino.h>


#define SX1276_MODE_SLEEP   0b00000000
#define SX1276_MODE_STANDBY 0b00000001
#define SX1276_MODE_TX 0b00000011
#define SX1276_MODE_FS_MODE_RX 0b00000100
#define SX1276_MODE_RX_CONTINUOUS 0b00000101
#define SX1276_MODE_CAD 0b00000111
#define SX127X_CRYSTAL_FREQ 32.0

#define GX_MODULE_DEBUG

#define GX_MODULE_DEBUG_PORT   Serial
#if defined(GX_MODULE_DEBUG)
  #define GX_MODULE_DEBUG_PRINT(...) { GX_MODULE_DEBUG_PORT.print(__VA_ARGS__); }
  #define GX_MODULE_DEBUG_PRINTLN(...) { GX_MODULE_DEBUG_PORT.println(__VA_ARGS__); }
#else
  #define GX_MODULE_DEBUG_PRINT(...) {}
  #define GX_MODULE_DEBUG_PRINTLN(...) {}
#endif

#if defined(GX_MODULE_VERBOSE)
  #define GX_MODULE_VERBOSE_PRINT(...) { GX_MODULE_DEBUG_PORT.print(__VA_ARGS__); }
  #define GX_MODULE_VERBOSE_PRINTLN(...) { GX_MODULE_DEBUG_PORT.println(__VA_ARGS__); }
#else
  #define GX_MODULE_VERBOSE_PRINT(...) {}
  #define GX_MODULE_VERBOSE_PRINTLN(...) {}
#endif

#define GXMODULE_ASSERT(STATEVAR) { if((STATEVAR) != ERR_NONE) { return(STATEVAR); } }

/*!
  \brief Internal only.
*/
#define	ERR_TX_TX_ONGOING                   					1
#define	ERR_TX_RX_ONGOING					                    2
#define	ERR_TX_FSK_ONGOING					                  3
#define ERR_NONE                                      0
#define ERR_UNKNOWN                                   -1
#define ERR_CHIP_NOT_FOUND                            -2
#define ERR_MEMORY_ALLOCATION_FAILED                  -3
#define ERR_PACKET_TOO_LONG                           -4
#define ERR_TX_TIMEOUT                                -5
#define ERR_RX_TIMEOUT                                -6
#define ERR_CRC_MISMATCH                              -7
#define ERR_INVALID_BIT_RANGE                         -11
#define ERR_SPI_WRITE_FAILED                          -16
#define ERR_SPI_CMD_TIMEOUT                           -705
#define ERR_SPI_CMD_INVALID                           -706
#define ERR_SPI_CMD_FAILED                            -707


/*!
  \class GxModule

  \brief Implements all common low-level SPI/UART/I2C methods to control the wireless module.
  Every module class contains one private instance of this class.
*/
class GxModule {
  public:

    /*!
      \brief SPI-based module constructor. Will use the default SPI interface automatically initialize it.

      \param cs Arduino pin to be used as chip select.

      \param irq Arduino pin to be used as interrupt/GPIO.

      \param rst Arduino pin to be used as hardware reset for the module.
    */
    GxModule(uint8_t cs, uint8_t irq, uint8_t rst);

    /*!
      \brief Extended SPI-based module constructor. Will use the default SPI interface automatically initialize it.

      \param cs Arduino pin to be used as chip select.

      \param irq Arduino pin to be used as interrupt/GPIO.

      \param rst Arduino pin to be used as hardware reset for the module.

      \param gpio Arduino pin to be used as additional interrupt/GPIO.
    */
    GxModule(uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio);

    /*!
      \brief SPI-based module constructor.

      \param cs Arduino pin to be used as chip select.

      \param irq Arduino pin to be used as interrupt/GPIO.

      \param rst Arduino pin to be used as hardware reset for the module.

      \param spi SPI interface to be used, can also use software SPI implementations.

      \param spiSettings SPI interface settings.
    */
    GxModule(uint8_t cs, uint8_t irq, uint8_t rst, SPIClass& spi, SPISettings spiSettings);

    /*!
      \brief Extended SPI-based module constructor.

      \param cs Arduino pin to be used as chip select.

      \param irq Arduino pin to be used as interrupt/GPIO.

      \param rst Arduino pin to be used as hardware reset for the module.

      \param gpio Arduino pin to be used as additional interrupt/GPIO.

      \param spi SPI interface to be used, can also use software SPI implementations.

      \param spiSettings SPI interface settings.
    */
    GxModule(uint8_t cs, uint8_t irq, uint8_t rst, uint8_t gpio, SPIClass& spi, SPISettings spiSettings);


    /*!
      \brief Copy constructor.

      \param mod Module instance to copy.
    */
    GxModule(const GxModule& mod);

    /*!
      \brief Overload for assignment operator.

      \param frame rvalue Module.
    */
    GxModule& operator=(const GxModule& mod);

    // public member variables


    /*!
      \brief Basic SPI read command. Defaults to 0x00.
    */
    uint8_t SPIreadCommand = 0b00000000;

    /*!
      \brief Basic SPI write command. Defaults to 0x80.
    */
    uint8_t SPIwriteCommand = 0b10000000;

    // basic methods

    /*!
      \brief Initialize low-level module control.

      \param interface Interface to be used on the module. See \ref shield_config for details.
    */
    void init(uint8_t interface);

    /*!
      \brief Terminate low-level module control.

      \param interface Interface to be terminated. See \ref shield_config for details.
    */
    void term(uint8_t interface);

    // SPI methods

    /*!
      \brief SPI read method that automatically masks unused bits. This method is the preferred SPI read mechanism.

      \param reg Address of SPI register to read.

      \param msb Most significant bit of the register variable. Bits above this one will be masked out.

      \param lsb Least significant bit of the register variable. Bits below this one will be masked out.

      \returns Masked register value or status code.
    */
    int16_t SPIgetRegValue(uint8_t reg, uint8_t msb = 7, uint8_t lsb = 0);

    /*!
      \brief Overwrite-safe SPI write method with verification. This method is the preferred SPI write mechanism.

      \param reg Address of SPI register to write.

      \param value Single byte value that will be written to the SPI register.

      \param msb Most significant bit of the register variable. Bits above this one will not be affected by the write operation.

      \param lsb Least significant bit of the register variable. Bits below this one will not be affected by the write operation.

      \param checkInterval Number of milliseconds between register writing and verification reading. Some registers need up to 10ms to process the change.

      \returns \ref status_codes
    */
    int16_t SPIsetRegValue(uint8_t reg, uint8_t value, uint8_t msb = 7, uint8_t lsb = 0, uint8_t checkInterval = 2);

    /*!
      \brief SPI burst read method.

      \param reg Address of SPI register to read.

      \param numBytes Number of bytes that will be read.

      \param inBytes Pointer to array that will hold the read data.
    */
    void SPIreadRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t* inBytes);

    /*!
      \brief SPI basic read method. Use of this method is reserved for special cases, SPIgetRegValue should be used instead.

      \param reg Address of SPI register to read.

      \returns Value that was read from register.
    */
    uint8_t SPIreadRegister(uint8_t reg);

    /*!
      \brief SPI burst write method.

      \param reg Address of SPI register to write.

      \param data Pointer to array that holds the data that will be written.

      \param numBytes Number of bytes that will be written.
    */
    void SPIwriteRegisterBurst(uint8_t reg, uint8_t* data, uint8_t numBytes);

    /*!
      \brief SPI basic write method. Use of this method is reserved for special cases, SPIsetRegValue should be used instead.

      \param reg Address of SPI register to write.

      \param data Value that will be written to the register.
    */
    void SPIwriteRegister(uint8_t reg, uint8_t data);

    /*!
      \brief SPI single transfer method.

      \param cmd SPI access command (read/write/burst/...).

      \param reg Address of SPI register to transfer to/from.

      \param dataOut Data that will be transfered from master to slave.

      \param dataIn Data that was transfered from slave to master.

      \param numBytes Number of bytes to transfer.
    */
    void SPItransfer(uint8_t cmd, uint8_t reg, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes);

    // pin number access methods

    /*!
      \brief Access method to get the pin number of SPI chip select.

      \returns Pin number of SPI chip select configured in the constructor.
    */
    uint8_t getCs() const { return(_cs); }

    /*!
      \brief Access method to get the pin number of interrupt/GPIO.

      \returns Pin number of interrupt/GPIO configured in the constructor.
    */
    uint8_t getIrq() const { return(_irq); }

    /*!
      \brief Access method to get the pin number of hardware reset pin.

      \returns Pin number of hardware reset pin configured in the constructor.
    */
    uint8_t getRst() const { return(_rst); }

    /*!
      \brief Access method to get the pin number of second interrupt/GPIO.

      \returns Pin number of second interrupt/GPIO configured in the constructor.
    */
    uint8_t getGpio() const { return(_rx); }

    /*!
      \brief Access method to get the pin number of UART Rx.

      \returns Pin number of UART Rx configured in the constructor.
    */
    uint8_t getRx() const { return(_rx); }

    /*!
      \brief Access method to get the pin number of UART Rx.

      \returns Pin number of UART Rx configured in the constructor.
    */
    uint8_t getTx() const { return(_tx); }

    /*!
      \brief Access method to get the SPI interface.

      \returns SPI interface configured in the constructor.
    */
    SPIClass* getSpi() const { return(_spi); }

    /*!
      \brief Access method to get the SPI interface settings.

      \returns SPI interface settings configured in the constructor.
    */
    SPISettings getSpiSettings() const { return(_spiSettings); }

    /*!
      \brief Some modules contain external RF switch controlled by two pins. This function gives RadioLib control over those two pins to automatically switch Rx and Tx state.
      When using automatic RF switch control, DO NOT change the pin mode of rxEn or txEn from Arduino sketch!

      \param rxEn RX enable pin.

      \param txEn TX enable pin.
    */
    void setRfSwitchPins(uint8_t rxEn, uint8_t txEn);

    /*!
      \brief Set RF switch state.

      \param rxPinState Pin state to set on Tx enable pin (usually high to transmit).

      \param txPinState  Pin state to set on Rx enable pin (usually high to receive).
    */
    void setRfSwitchState(uint8_t rxPinState, uint8_t txPinState);

    // Arduino core overrides

    /*!
      \brief Arduino core pinMode override that checks GXMODULE_NC as alias for unused pin.

      \param pin Pin to change the mode of.

      \param mode Which mode to set.
    */
    static void pinMode(uint8_t pin, uint8_t mode);

    /*!
      \brief Arduino core digitalWrite override that checks GXMODULE_NC as alias for unused pin.

      \param pin Pin to write to.

      \param value Whether to set the pin high or low.
    */
    static void digitalWrite(uint8_t pin, uint8_t value);

    /*!
      \brief Arduino core digitalWrite override that checks GXMODULE_NC as alias for unused pin.

      \param pin Pin to read from.

      \returns Pin value.
    */
    static uint8_t digitalRead(uint8_t pin);

    /*!
      \brief Arduino core attachInterrupt override.

      \param interruptNum Interrupt number.

      \param userFunc Interrupt service routine.

      \param mode Pin hcange direction.
    */
    static void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), uint8_t mode);

    /*!
      \brief Arduino core detachInterrupt override.

      \param interruptNum Interrupt number.
    */
    static void detachInterrupt(uint8_t interruptNum);

    /*!
      \brief Arduino core yield override.
    */
    static void yield();

    /*!
      \brief Arduino core delay override.

      \param ms Delay length in milliseconds.
    */
    static void delay(uint32_t ms);

    /*!
      \brief Arduino core delayMicroseconds override.

      \param us Delay length in microseconds.
    */
    static void delayMicroseconds(uint32_t us);

    /*!
      \brief Arduino core millis override.
    */
    static uint32_t millis();

    /*!
      \brief Arduino core micros override.
    */
    static uint32_t micros();

#ifndef RADIOLIB_GODMODE
  private:
#endif
    uint8_t _cs = GXMODULE_NC;
    uint8_t _irq = GXMODULE_NC;
    uint8_t _rst = GXMODULE_NC;
    uint8_t _rx = GXMODULE_NC;
    uint8_t _tx = GXMODULE_NC;

    SPISettings _spiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

    bool _initInterface = false;
    SPIClass* _spi = NULL;

    bool _useRfSwitch = false;
    uint8_t _rxEn = GXMODULE_NC, _txEn = GXMODULE_NC;

};

#endif
