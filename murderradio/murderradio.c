#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h> 
#include <SDL.h>
#include <SDL_mixer.h>

Mix_Chunk *chunk[2];
int chunkTimer[2];

void initPlay()
{
	chunk[0] = Mix_LoadWAV("message.wav");
	chunk[1] = Mix_LoadWAV("tuning.wav");
}

void playMessage()
{
	if(chunk[0]) Mix_PlayChannel(0,chunk[0],0);
}

void playTuning()
{
	if(chunk[1]) Mix_PlayChannel(0,chunk[1],0);
}

int fd;

int set_interface_attribs(int fd, int speed, int parity)
{
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if(tcgetattr(fd, &tty) != 0) {
                printf("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if(tcsetattr(fd, TCSANOW, &tty) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking(int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

void initSerial(const char *portname)
{
	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if(fd < 0)
	{
		printf("error %d opening %s: %s", errno, portname, strerror (errno));
		exit(1);
	}

	set_interface_attribs(fd, B9600 /*B115200*/, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking(fd, 0);                // set no blocking

}

void termSerial()
{
	close(fd);
}

char line[256];

void parseSerial(char *message)
{
	int dial=128;
	int count=sscanf(message,"%d",&dial);
	if(count>0) {
		if(dial>120 && dial<138) {
			if(chunkTimer[0]<=0) {
				playMessage();
				chunkTimer[0]=30000;
				chunkTimer[1]=500;
			}
		} else {
			if(chunkTimer[1]<=0) {
				playTuning();
				chunkTimer[0]=200;
				chunkTimer[1]=20000;
			}
		}
	}
}

void getSerial()
{
	char buf [100];
	int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read	

	buf[n]=0;
	strcat(line,buf);
	char *eol;
	for(eol=strchr(buf,'\n');eol!=NULL;eol=strchr(buf,'\n')) {
		eol[0]=0;
		parseSerial(line);
		strcpy(line,eol+1);
	}
}

void updatePlay(int elapsed)
{
	chunkTimer[0]-=elapsed;
	if(chunkTimer[0]<0) chunkTimer[0]=0;
	chunkTimer[1]-=elapsed;
	if(chunkTimer[1]<0) chunkTimer[1]=0;
}

int main ( int argc, char** argv )
{
	int i;
	const char *port="/dev/ttyS0";
	
// initialize SDL audio
    if (SDL_Init(SDL_INIT_AUDIO) == -1)
    {
        printf( "Unable to init SDL: %s\n", SDL_GetError());
        return 1;
    }
// make sure SDL cleans up before exit
    atexit(SDL_Quit);

	//Initialize SDL_mixer
	if( Mix_OpenAudio( 44100, MIX_DEFAULT_FORMAT, 2, 2048 ) < 0 ) { 
		printf( "SDL_mixer could not initialize! SDL_mixer Error: %s\n", Mix_GetError() );
		return 1;
	}

	if(argc>1) port=argv[1];

	initSerial(port);
	initPlay();

    // program main loop
    int done = 0;
    int elapsed = 0;
    unsigned int lastTime = SDL_GetTicks();
    while (!done)
    {
        // message processing loop
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            // check for messages
            switch (event.type)
            {
            // exit if the window is closed
            case SDL_QUIT:
                done = 1;
                break;

            } // end switch
        } // end of message processing
        
	//char message[256];
	//sprintf(message,"%d\n",lastTime%1024);
	//parseSerial(message);
        getSerial();
	unsigned int now=SDL_GetTicks();
	elapsed=now-lastTime;
	lastTime=now;
	updatePlay(elapsed);
    } // end main loop

    termSerial();

    SDL_Quit();
    return 0;
}
