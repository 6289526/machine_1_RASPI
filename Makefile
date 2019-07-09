main: marker_detect_1.cpp
	g++ -o marker_detect_1 marker_detect_1.cpp serial.cpp socket.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv` -lboost_system
