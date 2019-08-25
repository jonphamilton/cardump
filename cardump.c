#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include<sys/socket.h>
#include<arpa/inet.h>   //inet_addr

#include "buffy.h"
#include "crc.h"

/* raw tty code mostly from:
	http://www.cs.uleth.ca/~holzmann/C/system/ttyraw.c */

void tty_atexit(void);
int tty_reset(void);
void tty_raw(void);
int screenio(void);
void fatal(char *mess);


static struct termios orig_termios;  /* TERMinal I/O Structure */
static int ttyfd = STDIN_FILENO;     /* STDIN_FILENO is 0 by default */

int main(int argc, char **argv) {
    /* check that input is from a tty */
    if (isatty(ttyfd)) {
			/* store current tty settings in orig_termios */
			if (tcgetattr(ttyfd,&orig_termios) < 0) fatal("can't get tty settings");
			/* register the tty reset with the exit handler */
			if (atexit(tty_atexit) != 0) fatal("atexit: can't register tty reset");
			tty_raw();      /* put tty in raw mode */
	} else {
			fprintf(stderr,"Not a tty. Reading from file...\n");
	}

 for (;;) { 
    screenio();     /* run application code */
 }
    return 0;       /* tty_atexit will restore terminal */

}



#define READ_REQ 0x0b
#define WRITE_REQ 0x0c
#define REPLY 0x06
#define ERROR 0x15

#pragma pack(push, 1)

typedef struct {
	uint8_t table;
	uint8_t row;
} careg;

typedef struct {
	uint8_t type;
	uint8_t idx;
} cardev;

typedef struct {
	 cardev dst;
	 cardev src;
	uint8_t len;
   uint16_t reserved;
	uint8_t type;
	   char payload[256];
   uint16_t crc;
} carframe;

typedef struct {
	   char pad;
      careg reg;
} caread;

typedef struct {
	   char pad;
	  careg reg;
       char payload[256];
} carwrite;

typedef struct {
	   char ack;
	  careg reg;
} careply;


#pragma pack(pop)

void send_socket(char message[])
{
        int socket_desc;
        struct sockaddr_in server;

        //Create socket
        socket_desc = socket(AF_INET , SOCK_STREAM , 0);
        if (socket_desc == -1)
        {
                printf("Could not create socket");
        }


        server.sin_family = AF_INET;
        server.sin_addr.s_addr = inet_addr("192.168.0.23");
        server.sin_port = htons( 2003 );

        if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
        {
                puts("connect error");
        }

        puts("Connected");
        //Send some data
        if( send(socket_desc , message , strlen(message) , 0) < 0)
        {
                puts("Send failed");
        }
        puts("Data Send\n");
        close(socket_desc);
}

int screenio(void) {
	int bytes;

	buffy framebuf;
	framebuf.len = 0;
	carframe frame;
	crcInit();

	char buffer[32]; //serial reads tend to have less than 32 bytes

	uint8_t tries = 0;
	int shifts = 0;
	int syncs = 0;

        long curtime = (unsigned long)time(NULL); 
        long endtime = curtime + 5 ;

        int heatpump = 0;
        int furnace = 0; 
        int thermostat = 0;  
        

	while ( endtime > curtime ) {
                curtime = (unsigned long)time(NULL);
         	bytes = read(ttyfd, buffer, 32);
		if (bytes < 0) fatal("Read error");
		if (bytes == 0) { tries+=1; } else { tries = 0; }
		if (tries>9) fatal("Not trying again");

		bufadd(&framebuf, buffer, bytes);

		if (framebuf.len<4) continue;
		int datalen = framebuf.data[4];
		if (datalen == 0) {
			shifts++;
			bufshift(&framebuf, (int)(framebuf.len>>1));
			continue;
		}

		int framelen = 10+datalen;
		if (framebuf.len<framelen) continue;

		if (shifts>0) fprintf(stderr, "Looking for %d byte frame in %d byte buffer\n", datalen, framebuf.len);

		if (crcFast(framebuf.data, framelen) == 0) {
			if (shifts>0) {
				fprintf(stderr,"*** Synced stream after %d shifts ***\n", shifts);
				shifts=0;
				syncs++;
				if (syncs>100) fatal("Stream too noisy");
			}
			memcpy(&frame, framebuf.data, framelen);
			frame.crc=framebuf.data[framelen-2]<<8 | framebuf.data[framelen-1];
                       
                        
                        switch(frame.src.type) {
                         case 0x50 :
                         heatpump++;   
                         break;
                     
                         case 0x40 :
                         furnace++ ;
		         
                         // GET BLOWER RPM
                         if ( frame.payload[1] == 0x03 && frame.payload[2] == 0x06) { 
                         int16_t rpm = (frame.payload[4]<<8) | frame.payload[7];
                         char message[256] = "CC.V2.inside.Furnace.BlowerRPM " ;
                         sprintf(message, "%s %d %d \n", message, rpm, (unsigned long)time(NULL)); 
                         fprintf(stdout, message);
                         send_socket(message); 
                         }

                         // GET BLOWER CFM
                         if ( frame.payload[1] == 0x03 && frame.payload[2] == 0x16) {
                         int16_t cfm = (frame.payload[7]<<8) | frame.payload[10];
                         fprintf(stdout, "CC.Furnace.BlowerCFM %i ", cfm);
                         fprintf(stdout, "%lu\n", (unsigned long)time(NULL));
                         }
                         break;

                         case 0x20 :
                         //for (int i=0;i<frame.len;i++) fprintf(stdout, "%02x ", frame[i]);
                         thermostat++; 
                         break;

                       }


			if (REPLY == frame.type) {
				//Example of a known data point.
				if (frame.src.type == 0x50 && frame.payload[1] == 0x3E && frame.payload[2] == 0x01) {
					int16_t oat = (frame.payload[3]<<8) | frame.payload[4];
					int16_t t2 = (frame.payload[5]<<8) | frame.payload[6];
                                        char tempmessage[256] = "CC.V2.inside.HeatPump0.outsidetemp ";
                                        sprintf(tempmessage, "%s %d %d \n", tempmessage, oat/16, (unsigned long)time(NULL)); 
                                        send_socket(tempmessage);
                                        fprintf(stdout, tempmessage);
                                        fprintf(stdout, "CC.HeatPump0.outsidecoil %d ", t2/16, t2);
                                        fprintf(stdout, "%lu\n", (unsigned long)time(NULL));
                                        fflush(stdout);
                               }
			}


			bufshift(&framebuf, framelen);
		} else {
			shifts++;
			bufshift(&framebuf, 1);
		}

	}

fprintf(stdout, "CC.Thermostat.srcv2 %i ", thermostat);
fprintf(stdout, "%lu\n", (unsigned long)time(NULL));
fprintf(stdout, "CC.Furnace.srcv2 %i ", furnace);
fprintf(stdout, "%lu\n", (unsigned long)time(NULL));
fprintf(stdout, "CC.HeatPump0.srcv2 %i ", heatpump);
fprintf(stdout, "%lu\n", (unsigned long)time(NULL));

}

void fatal(char *message) {
	fprintf(stderr,"fatal error: %s\n",message);
	exit(1);
}

/* exit handler for tty reset */
void tty_atexit(void)  /* NOTE: If the program terminates due to a signal   */
{                      /* this code will not run.  This is for exit()'s     */
	tty_reset();        /* only.  For resetting the terminal after a signal, */
}                      /* a signal handler which calls tty_reset is needed. */

/* reset tty - useful also for restoring the terminal when this process
   wishes to temporarily relinquish the tty
*/
int tty_reset(void) {
	/* flush and reset */
	if (tcsetattr(ttyfd,TCSAFLUSH,&orig_termios) < 0) return -1;
	return 0;
}


void tty_raw() {
	struct termios raw;

	raw = orig_termios;  /* copy original and then modify below */

	/* input modes - clear indicated ones giving: no break, no CR to NL, 
		 no parity check, no strip char, no start/stop output (sic) control */
	raw.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

	/* output modes - clear giving: no post processing such as NL to CR+NL */
	raw.c_oflag &= ~(OPOST);

	/* control modes - set 8 bit chars */
	raw.c_cflag |= (CS8);

	/* local modes - clear giving: echoing off, canonical off (no erase with 
		 backspace, ^U,...),  no extended functions, no signal chars (^Z,^C) */
	raw.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

	/* control chars - set return condition: min number of bytes and timer */
	raw.c_cc[VMIN] = 5; raw.c_cc[VTIME] = 8; /* after 5 bytes or .8 seconds
																							after first byte seen      */
	raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0; /* immediate - anything       */

	raw.c_cc[VMIN] = 2; raw.c_cc[VTIME] = 0; /* after two bytes, no timer  */
	raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 8; /* after a byte or .8 seconds */

	/* put terminal in raw mode after flushing */
	if (tcsetattr(ttyfd,TCSAFLUSH,&raw) < 0) fatal("can't set raw mode");

	//printf("input speed was %d\n",cfgetispeed(&orig_termios));
	cfsetispeed(&orig_termios, B38400); //Set 38.4k
	cfsetospeed(&orig_termios, B38400); //Set 38.4k
	//printf("input speed set to %d\n",cfgetispeed(&orig_termios));
}

