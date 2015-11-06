CXX = g++
CXXFLAGS = -O3 -ggdb3

tsl2561: main.o Adafruit_TSL2561_U.o
	$(CXX) -flto -o tsl2561 main.o Adafruit_TSL2561_U.o

main.o: main.cpp Adafruit_TSL2561_U.h
	$(CXX) $(CXXFLAGS) -c main.cpp

Adafruit_TSL2561_U.o: Adafruit_TSL2561_U.cpp Adafruit_TSL2561_U.h
	$(CXX) $(CXXFLAGS) -c Adafruit_TSL2561_U.cpp

