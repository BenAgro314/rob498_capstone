CC=g++
CFLAGS=-c -Wall
LDFLAGS=`pkg-config opencv4 --cflags --libs`
SOURCES=main.cpp src/flow_opencv.cpp src/flow_px4.cpp src/optical_flow.cpp src/px4flow.cpp src/trackFeatures.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=flow_run

.PHONY: all clean flow

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf $(OBJECTS) $(EXECUTABLE)

flow:
	export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/opt/homebrew/lib/pkgconfig && make all