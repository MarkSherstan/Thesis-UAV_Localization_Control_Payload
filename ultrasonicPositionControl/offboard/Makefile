# General
CC = g++

# Names and flags
CPPSOURCES += $(wildcard *.cpp)
OBJECTS = $(CPPSOURCES:.cpp=.o)
EXECUTABLE = ultrasonicPosition
CFLAGS = -c -Wall -g

# Compile and clean
all: $(CPPSOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) -pthread $(OBJECTS) -o $@ $(LIBS)

.cpp.o:
	$(CC) $(CFLAGS) -std=c++14 $(INCLUDES) $< -o $@

clean:
	@echo $OBJECTS = $(OBJECTS)
	rm -rf $(OBJECTS) $(EXECUTABLE)
