#include <iostream>
#include <cstring>
#include <atomic>

#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>

#include "Adafruit_TSL2561_U.h"

std::atomic<bool> g_run(true);

void stop_signal(int sig)
{
	g_run = false;
}

int main(int argc, char *argv[])
{
	signal(SIGINT, stop_signal);
	signal(SIGHUP, SIG_IGN);
	
	Adafruit_TSL2561_Unified sensor;
	
	if (!sensor.begin())
	{
		perror("sensor begin");
		return 1;
	}
	
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (sock < 0)
	{
		perror("socket");
		return 1;
	}
	
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = 0;
	if (bind(sock, (sockaddr*)&addr, sizeof(addr)) != 0)
	{
		perror("bind");
		close(sock);
		return 1;
	}
	
	addr.sin_addr.s_addr = inet_addr("192.168.1.250");
	addr.sin_port = htons(40000);
	
	sensor.setGain(TSL2561_GAIN_16X);
	sensor.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
	sensor.enableAutoRange(true);

	while (g_run)
	{
		float lux = sensor.calculateLux();
		std::cout << "Lux = " << lux << std::endl;
		//std::cout << uint32_t(lux) << " / " << (uint32_t(lux * 1000) % 1000) << std::endl;
		
		uint8_t message[4];
		*((uint16_t*)&message[0]) = htons(uint16_t(lux));
		*((uint16_t*)&message[2]) = htons(uint32_t(lux * 1000) % 1000);
		
		if (sendto(sock, message, 4, 0, (sockaddr*)&addr, sizeof(addr)) <= 0)
		{
			perror("sendto");
		}
		
		sleep(1);
	}
	
	close(sock);
	return 0;
}

