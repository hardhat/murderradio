CFLAGS=-g $(shell sdl2-config --cflags) -I/opt/local/include
CXXFLAGS=-g $(shell sdl2-config --cflags) -I/opt/local/include
LIBS=$(shell sdl2-config --libs) -lSDL2_mixer

murderradio: murderradio.o
	g++ -g -o murderradio murderradio.o $(LIBS)

clean:
	rm *.o
