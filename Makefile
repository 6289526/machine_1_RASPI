main: marker_detect_1.cpp
	g++ -o marker_detect_1 marker_detect_1.cpp serial.cpp network.cpp network_tcp.cpp network_udp.cpp Safe_Network_tcp.cpp -lstdc++ -lpthread `pkg-config --cflags --libs opencv` -lboost_system
