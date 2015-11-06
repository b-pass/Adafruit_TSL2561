/**************************************************************************/
/*!
    @file     Adafruit_TSL2561.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the TSL2561 digital luminosity (light) sensors.

    Pick one up at http://www.adafruit.com/products/439

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v2.0 - Rewrote driver for Adafruit_Sensor and Auto-Gain support, and
           added lux clipping check (returns 0 lux on sensor saturation)
    v1.0 - First release (previously TSL2561)
*/
/**************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>

#include "Adafruit_TSL2561_U.h"

#define TSL2561_DELAY_INTTIME_13MS    (15)
#define TSL2561_DELAY_INTTIME_101MS   (120)
#define TSL2561_DELAY_INTTIME_402MS   (450)

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::write8 (uint8_t reg, uint32_t value)
{
	uint8_t buf[2] = {reg, value&0xFF};
	if (write(_fd, buf, 2) != 2)
		perror("write8");
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_TSL2561_Unified::read8(uint8_t reg)
{
	uint8_t buf[2] = {reg, 0};
	if (write(_fd, buf, 1) != 1)
	{
		perror("write inside read8");
		return 0;
	}

	if (read(_fd, &buf, 1) != 1)
	{
		perror("read8");
		return 0;
	}

	return buf[0];
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t Adafruit_TSL2561_Unified::read16(uint8_t reg)
{
	uint8_t buf[2] = {reg, 0};
	if (write(_fd, buf, 1) != 1)
	{
		perror("write inside read16");
		return -1;
	}

	if (read(_fd, &buf, 2) != 2)
	{
		perror("read16");
		return -1;
	}

	return buf[0] | (buf[1] << 8);
}

/**************************************************************************/
/*!
    Enables the device
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::enable(void)
{
  /* Enable the device by setting the control bit to 0x03 */
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
}

/**************************************************************************/
/*!
    Disables the device (putting it in lower power sleep mode)
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::disable(void)
{
  /* Turn the device off to save power */
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

/**************************************************************************/
/*!
    Private function to read luminosity on both channels
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::getData (uint16_t *broadband, uint16_t *ir)
{
  /* Enable the device by setting the control bit to 0x03 */
  enable();

  /* Wait x ms for ADC to complete */
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      usleep(1000*TSL2561_DELAY_INTTIME_13MS);  // KTOWN: Was 14ms
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      usleep(1000*TSL2561_DELAY_INTTIME_101MS); // KTOWN: Was 102ms
      break;
    default:
      usleep(1000*TSL2561_DELAY_INTTIME_402MS); // KTOWN: Was 403ms
      break;
  }

  /* Reads a two byte value from channel 0 (visible + infrared) */
  *broadband = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

  /* Reads a two byte value from channel 1 (infrared) */
  *ir = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);

  /* Turn the device off to save power */
  disable();
}

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
Adafruit_TSL2561_Unified::Adafruit_TSL2561_Unified(uint8_t addr, char const *device) 
{
  _tsl2561Initialised = false;
  _tsl2561AutoGain = true;
  _tsl2561IntegrationTime = TSL2561_INTEGRATIONTIME_13MS;
  _tsl2561Gain = TSL2561_GAIN_1X;
  
  _fd = open(device, O_RDWR);
  if (_fd < 0)
  {
    perror("open");
    return;
  }

  if (ioctl(_fd, I2C_SLAVE, addr) != 0)
    perror("ioctl I2C_SLAVE");
}

Adafruit_TSL2561_Unified::~Adafruit_TSL2561_Unified()
{
	close(_fd);
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
bool Adafruit_TSL2561_Unified::begin(void) 
{
  /* Make sure we're actually connected */
  uint8_t x = read8(TSL2561_REGISTER_ID);
  if (!(x & 0x0A))
  {
    return false;
  }
  _tsl2561Initialised = true;

  /* Set default integration time and gain */
  setIntegrationTime(_tsl2561IntegrationTime);
  setGain(_tsl2561Gain);

  /* Note: by default, the device is in power down mode on bootup */
  disable();

  return true;
}
  
/**************************************************************************/
/*!
    @brief  Enables or disables the auto-gain settings when reading
            data from the sensor
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::enableAutoRange(bool enable)
{
   _tsl2561AutoGain = enable ? true : false;
}

/**************************************************************************/
/*!
    Sets the integration time for the TSL2561
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::setIntegrationTime(tsl2561IntegrationTime_t time)
{
  if (!_tsl2561Initialised) begin();

  /* Enable the device by setting the control bit to 0x03 */
  enable();

  /* Update the timing register */
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, time | _tsl2561Gain);

  /* Update value placeholders */
  _tsl2561IntegrationTime = time;

  /* Turn the device off to save power */
  disable();
}

/**************************************************************************/
/*!
    Adjusts the gain on the TSL2561 (adjusts the sensitivity to light)
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::setGain(tsl2561Gain_t gain)
{
  if (!_tsl2561Initialised) begin();

  /* Enable the device by setting the control bit to 0x03 */
  enable();

  /* Update the timing register */
  write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, _tsl2561IntegrationTime | gain);

  /* Update value placeholders */
  _tsl2561Gain = gain;

  /* Turn the device off to save power */
  disable();
}

/**************************************************************************/
/*!
    @brief  Gets the broadband (mixed lighting) and IR only values from
            the TSL2561, adjusting gain if auto-gain is enabled
*/
/**************************************************************************/
void Adafruit_TSL2561_Unified::getLuminosity (uint16_t *broadband, uint16_t *ir)
{
  bool valid = false;

  if (!_tsl2561Initialised) begin();

  /* If Auto gain disabled get a single reading and continue */
  if(!_tsl2561AutoGain)
  {
    getData (broadband, ir);
    return;
  }

  /* Read data until we find a valid range */
  bool _agcCheck = false;
  do
  {
    uint16_t _b, _ir;
    uint16_t _hi, _lo;
    tsl2561IntegrationTime_t _it = _tsl2561IntegrationTime;

    /* Get the hi/low threshold for the current integration time */
    switch(_it)
    {
      case TSL2561_INTEGRATIONTIME_13MS:
        _hi = TSL2561_AGC_THI_13MS;
        _lo = TSL2561_AGC_TLO_13MS;
        break;
      case TSL2561_INTEGRATIONTIME_101MS:
        _hi = TSL2561_AGC_THI_101MS;
        _lo = TSL2561_AGC_TLO_101MS;
        break;
      default:
        _hi = TSL2561_AGC_THI_402MS;
        _lo = TSL2561_AGC_TLO_402MS;
        break;
    }

    getData(&_b, &_ir);

    /* Run an auto-gain check if we haven't already done so ... */
    if (!_agcCheck)
    {
      if ((_b < _lo) && (_tsl2561Gain == TSL2561_GAIN_1X))
      {
        /* Increase the gain and try again */
        setGain(TSL2561_GAIN_16X);
        /* Drop the previous conversion results */
        getData(&_b, &_ir);
        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = true;
      }
      else if ((_b > _hi) && (_tsl2561Gain == TSL2561_GAIN_16X))
      {
        /* Drop gain to 1x and try again */
        setGain(TSL2561_GAIN_1X);
        /* Drop the previous conversion results */
        getData(&_b, &_ir);
        /* Set a flag to indicate we've adjusted the gain */
        _agcCheck = true;
      }
      else
      {
        /* Nothing to look at here, keep moving ....
           Reading is either valid, or we're already at the chips limits */
        *broadband = _b;
        *ir = _ir;
        valid = true;
      }
    }
    else
    {
      /* If we've already adjusted the gain once, just return the new results.
         This avoids endless loops where a value is at one extreme pre-gain,
         and the the other extreme post-gain */
      *broadband = _b;
      *ir = _ir;
      valid = true;
    }
  } while (!valid);
}

/**************************************************************************/
/*!
    Converts the raw sensor values to the standard SI lux equivalent.
    Returns 0 if the sensor is saturated and the values are unreliable.
*/
/**************************************************************************/
float Adafruit_TSL2561_Unified::calculateLux(uint16_t broadband, uint16_t ir)
{
  /* Make sure the sensor isn't saturated! */
  uint16_t clipThreshold;
  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      clipThreshold = TSL2561_CLIPPING_13MS;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      clipThreshold = TSL2561_CLIPPING_101MS;
      break;
    default:
      clipThreshold = TSL2561_CLIPPING_402MS;
      break;
  }

  /* Return 0 lux if the sensor is saturated */
  if ((broadband > clipThreshold) || (ir > clipThreshold))
  {
    return 0;
  }

  float scale = 1;  
  // Scale for gain
  if (!_tsl2561Gain)
  {
      scale = 16;
  }

  switch (_tsl2561IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      scale *= 322.0 / 11.0;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      scale *= 322.0 / 81.0;
      break;
  }

  float channel0 = broadband * scale;
  float channel1 = ir * scale;

  float ratio = 0;
  if (channel0 != 0)
  {
    ratio = (float)channel1 / (float)channel0;
  }

  float lux;
  if (ratio <= 0.50)
  {
      lux = (float)(0.0304 * channel0 - 0.062 * channel0 * pow(ratio, 1.4));
  }
  else if (ratio <= 0.61)
  {
      lux = (float) (0.0224 * channel0 - 0.031 * channel1);
  }
  else if (ratio <= 0.8)
  {
      lux = (float)(0.0128 * channel0 - 0.0153 * channel1);
  }
  else if (ratio <= 1.3)
  {
      lux = (float)(0.00146 * channel0 - 0.00112 * channel1);
  }
  else
  {
      lux = 0;
  }

  if (lux < 0)
  {
    lux = 0;
  }
  
  return lux;
}
