view3d.o : view3d.cpp view3d.h
	g++ view3d.cpp -std=c++11 -o view3d `pkg-config opencv --cflags --libs` `pkg-config freenect2 --cflags --libs`
