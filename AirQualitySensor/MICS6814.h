#ifndef MICS6814_H
#define MICS6814_H

#include "Arduino.h"

typedef enum
{
	CH_NH3,
	CH_RED,
	CH_OX
} channel_t;

typedef enum
{
	CO,
	NO2,
	NH3,
	C3H8,
	C4H10,
	CH4,
	H2,
	C2H5OH
} gas_t;

class MICS6814
{
public:
	MICS6814(int pinCO, int pinNO2, int pinNH3);

	void calibrate();
	void loadCalibrationData(
		uint16_t base_NH3,
		uint16_t base_RED,
		uint16_t base_OX);

	float measure(gas_t gas);

	uint16_t getResistance    (channel_t channel) const;
	uint16_t getBaseResistance(channel_t channel) const;
	float    getCurrentRatio  (channel_t channel) const;
private:
	uint8_t NH3PIN;
	uint8_t COPIN;
	uint8_t OXPIN;

	uint16_t NH3baseR;
	uint16_t REDbaseR;
	uint16_t OXbaseR;;
};

#endif