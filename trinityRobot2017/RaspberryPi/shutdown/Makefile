TARGET := shutdown

CXX := g++

CXXFLAGS := -std=c++0x -Wall -pedantic
LDLIBS := -lwiringPi

srcfiles := $(shell find . -maxdepth 1 -name "*.cpp")
objects  := $(patsubst %.cpp, %.o, $(srcfiles))

all: $(TARGET)

$(TARGET): $(objects)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET) $(objects) $(LDLIBS)

clean:
	rm -f $(objects) $(TARGET)
