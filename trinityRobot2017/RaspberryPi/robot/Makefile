TARGET=robot

CXX=g++

CXXFLAGS=-std=c++17 -Wall -pedantic -Wfatal-errors
LDLIBS=-lwiringPi

objects=main.o lidar.o xv11lidar/xv11lidar.o

all: $(TARGET)

$(TARGET): $(objects)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET) $(objects) $(LDLIBS)

clean:
	rm -f $(objects) $(TARGET)
