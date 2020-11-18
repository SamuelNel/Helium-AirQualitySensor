#include "MICS6814.h"

MICS6814 :: MICS6814 (int pinCO, int pinNO2, int pinNH3)
{
	NH3PIN = pinCO;
	COPIN = pinNO2;
	OXPIN = pinNH3;
}

/**
 * Calibrates MICS-6814 before use
 *
 * Work algorithm:
 *
 * Continuously measures resistance,
 * with saving the last N measurements in the buffer.
 * Calculates the floating average of the last seconds.
 * If the current measurement is close to average,
 * we believe that the calibration was successful.
 */
void MICS6814 :: calibrate ()
{
	// The number of seconds that must pass before
	// than we will assume that the calibration is complete
	// (Less than 64 seconds to avoid overflow)
	uint8_t seconds = 10;

	// Tolerance for the average of the current value
	uint8_t delta = 2;

	// Circular buffer for the measurements
  uint16_t bufferNH3[seconds];
  uint16_t bufferRED[seconds];
  uint16_t bufferOX[seconds];
  // Pointers for the next element in the buffer
  uint8_t pntrNH3 = 0;
  uint8_t pntrRED = 0;
  uint8_t pntrOX = 0;
  // Current floating sum in the buffer
  uint16_t fltSumNH3 = 0;
  uint16_t fltSumRED = 0;
  uint16_t fltSumOX = 0;

  // Current measurements;
  uint16_t curNH3;
  uint16_t curRED;
  uint16_t curOX;

  // Flag to see if the channels are stable
  bool NH3stable = false;
  bool REDstable = false;
  bool OXstable = false;

  // Initialize buffer
  for (int i = 0; i < seconds; ++i) {
    bufferNH3[i] = 0;
    bufferRED[i] = 0;
    bufferOX[i] = 0;
  }

  do {
    // Wait a second
    delay(1000);
    Serial.print(".");
    // Read new resistances
    unsigned long rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(NH3PIN);
    }
    curNH3 = rs/3;
    rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(COPIN);
    }
    curRED = rs/3;
    rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(OXPIN);
    }
    curOX = rs/3;

    // Update floating sum by subtracting value
    // about to be overwritten and adding the new value.
    fltSumNH3 = fltSumNH3 + curNH3 - bufferNH3[pntrNH3];
    fltSumRED = fltSumRED + curRED - bufferRED[pntrRED];
    fltSumOX = fltSumOX + curOX - bufferOX[pntrOX];

    // Store new measurement in buffer
    bufferNH3[pntrNH3] = curNH3;
    bufferRED[pntrRED] = curRED;
    bufferOX[pntrOX] = curOX;

    // Determine new state of flags
    NH3stable = abs(fltSumNH3 / seconds - curNH3) < delta;
    REDstable = abs(fltSumRED / seconds - curRED) < delta;
    OXstable = abs(fltSumOX / seconds - curOX) < delta;

    // Advance buffer pointer
    pntrNH3 = (pntrNH3 + 1) % seconds ;
    pntrRED = (pntrRED + 1) % seconds;
    pntrOX = (pntrOX + 1) % seconds;

    //Mikä kestää?
    if(!NH3stable) {
      Serial.print("(NH3:");
      Serial.print(abs(fltSumNH3 / seconds - curNH3));
      Serial.print(")");
    }
    if(!REDstable) {
      Serial.print("(RED:");
      Serial.print(abs(fltSumNH3 / seconds - curRED));
      Serial.print(")");
    }
    if(!OXstable) {
      Serial.print("(OX:");
      Serial.print(abs(fltSumNH3 / seconds - curOX));
      Serial.print(")");
    }

  } while (!NH3stable || !REDstable || !OXstable);

  NH3baseR = fltSumNH3 / seconds;
  REDbaseR = fltSumRED / seconds;
  OXbaseR = fltSumOX / seconds;
}

void MICS6814 :: loadCalibrationData (
	uint16_t baseNH3,
	uint16_t baseCO,
	uint16_t baseNO2)
{
	NH3baseR = baseNH3;
	REDbaseR = baseCO;
	OXbaseR = baseNO2;
}

/**
 * Measures the gas concentration in ppm for the specified gas.
 *
 * @param gas
 * Gas ​​for concentration calculation.
 *
 * @return
 * Current gas concentration in parts per million (ppm).
 */
float MICS6814 :: measure (gas_t gas)
{
	float ratio;
	float c = 0;

	switch (gas)
	{
		case CO:
			ratio = getCurrentRatio(CH_RED);
			c = pow(ratio, -1.179) * 4.385;
			break;
		case NO2:
			ratio = getCurrentRatio(CH_OX);
			c = pow(ratio, 1.007) / 6.855;
			break;
		case NH3:
			ratio = getCurrentRatio(CH_NH3);
			c = pow(ratio, -1.67) / 1.47;
			break;
		case C3H8:
			ratio = getCurrentRatio(CH_NH3);
			c = pow(ratio, -2.518) * 570.164;
			break;
		case C4H10:
			ratio = getCurrentRatio(CH_NH3);
			c = pow(ratio, -2.138) * 398.107;
			break;
		case CH4:
			ratio = getCurrentRatio(CH_RED);
			c = pow(ratio, -4.363) * 630.957;
			break;
		case H2:
			ratio = getCurrentRatio(CH_RED);
			c = pow(ratio, -1.8) * 0.73;
			break;
		case C2H5OH:
			ratio = getCurrentRatio(CH_RED);
			c = pow(ratio, -1.552) * 1.622;
			break;
	}

	return isnan (c)? -1: c;
}

/**
 * Requests the current resistance for this channel from the sensor. 
 * Value is the value of the ADC in the range from 0 to 1024.
 * 
 * @param channel
 * Channel for reading base resistance.
 *
 * @return
 * Unsigned 16-bit base resistance of the selected channel.
 */
uint16_t MICS6814 :: getResistance (channel_t channel) const
{
	unsigned long rs = 0;
	int counter = 0;

	switch (channel) {
		case CH_NH3:
			for(int i = 0; i < 100; i++) {
				rs += analogRead(NH3PIN);
				counter++;
				delay(2);
			}
			return rs/counter;
		case CH_RED:
			for(int i = 0; i < 100; i++) {
				rs += analogRead(COPIN);
				counter++;
				delay(2);
			}
			return rs/counter;
		case CH_OX:      
			for(int i = 0; i < 100; i++) {
				rs += analogRead(OXPIN);
				counter++;
				delay(2);
			}
			return rs/counter;

  }

	return counter != 0 ? rs / counter : 0;
}

uint16_t MICS6814 :: getBaseResistance (channel_t channel) const
{
	switch (channel)
	{
		case CH_NH3:
			return NH3baseR;
		case CH_RED:
			return REDbaseR;
		case CH_OX:
			return OXbaseR;
	}

	return 0;
}

/**
 * Calculates the current resistance coefficient for a given channel.
 * 
 * @param channel
 * Channel for requesting resistance values.
 *
 * @return
 * Floating point resistance coefficient for this sensor.
 */
float MICS6814 :: getCurrentRatio (channel_t channel) const
{
	float baseResistance = (float) getBaseResistance (channel);
	float resistance = (float) getResistance (channel);

	return resistance / baseResistance * (1023.0 - baseResistance) / (1023.0 - resistance);

	return -1.0;
}