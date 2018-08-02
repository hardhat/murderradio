CFLAGS=-g $(shell sdl-config --cflags) -I/opt/local/include
CXXFLAGS=-g $(shell sdl-config --cflags) -I/opt/local/include
LIBS=$(shell sdl-config --libs) -lSDL_mixer

murderradio: murderradio.o
	g++ -g -o murderradio murderradio.o $(LIBS)

clean:
	rm *.o
