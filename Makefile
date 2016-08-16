Main: Main.o
	g++ -Wall -o Main Main.o -lRobotUtil -lwiringPi -std=c++11 -pthread
Main.o: Main.cpp
	g++ -Wall -c Main.cpp -lRobotUtil -lwiringPi -std=c++11 -pthread
clean:
	rm -f *.o Main
