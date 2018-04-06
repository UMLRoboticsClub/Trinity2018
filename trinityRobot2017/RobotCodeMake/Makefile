TARGET = robot
CXX = g++
OBJDIR = obj

#add -O2 for final build
CXXFLAGS = -std=c++17 -g -Wall -Wextra -pedantic -Wfatal-errors
LDLIBS = -lpthread -lpigpiod_if2 -lrt

SRCFILES = $(shell find . -maxdepth 1 -name "*.cpp" | cut -c 3-)
OBJECTS = $(addprefix $(OBJDIR)/, $(patsubst %.cpp, %.o, $(SRCFILES)))

all : $(TARGET)

$(TARGET) : $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS) $(LDLIBS)

$(OBJDIR)/%.o : %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET)
