TARGET := robot
CXX := g++

#add -O2 for final build
CXXFLAGS := -std=c++17 -Wall -pedantic -Wfatal-errors
LDLIBS := -lpthread -lpigpiod_if2 -lrt

srcfiles := $(shell find . -maxdepth 1 -name "*.cpp")
objects  := $(patsubst %.cpp, %.o, $(srcfiles))

all: $(TARGET)

$(TARGET): $(objects)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET) $(objects) $(LDLIBS)

clean:
	rm -f $(objects) $(TARGET)
