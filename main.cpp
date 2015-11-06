#include <iostream>
#include <cstring>
#include <errno.h>

#include "Adafruit_TSL2561_U.h"

int main(int argc, char *argv[])
{
	Adafruit_TSL2561_Unified sensor;
	
	if (!sensor.begin())
	{
		std::cerr << "Error! " << strerror(errno) << std::endl;
		return 1;
	}
	
	sensor.enableAutoRange(true);
	sensor.setGain(TSL2561_GAIN_16X);
	sensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
	
	std::cout << "Lux = " << sensor.calculateLux() << std::endl;
	
	return 0;
}
