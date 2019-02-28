main: marker_detect_1.cpp
	g++ -o marker_detect_1 marker_detect_1.cpp serial.cpp -lstdc++ `pkg-config --cflags --libs opencv`
