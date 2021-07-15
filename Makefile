#main: marker_detect_1.cpp
#	g++ -o marker_detect_1 marker_detect_1.cpp serial.cpp network.cpp network_tcp.cpp network_udp.cpp Safe_Network_tcp.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv4`

all: marker_detect_1

marker_detect_1: marker_detect_1.o serial.o network.o network_tcp.o network_udp.o Safe_Network_tcp.o
	g++ marker_detect_1.o -o marker_detect_1  -lstdc++ -lpthread `pkg-config --cflags --libs opencv4` -I/usr/local/include -L/usr/local/lib -lwiringPi

marker_detect_1.o: marker_detect_1.cpp
	g++ -c marker_detect_1.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv4`

serial.o: serial.cpp
	g++ -c serial.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv4`

network.o: network.cpp
	g++ -c network.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv4`

network_tcp.o: network_tcp.cpp
	g++ -c network_tcp.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv4`

network_udp.o: network_udp.cpp
	g++ -c network_udp.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv4`

Safe_Network_tcp.o: Safe_Network_tcp.cpp
	g++ -c Safe_Network_tcp.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv4`