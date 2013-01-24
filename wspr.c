/*
WSPR encoding module
All the encoding is done here
Thanks to K1JT, G4JNT and PE1NZZ for publishing
helping infos.

Encoding process is in 5 steps:
   * bits packing of user message in 50 bits
   * store the 50 bits dans 11 octets (88 bits and only 81 useful)
   * convolutionnal encoding with two pariy generators (-> 162 bits)
   * interleaving of the 162 bits with bit-reverse technique
   * synchronisation with a psudo-random vector to obtain the
      162 symbols defining one frequency of 4.

 F8CHK 29/03/2011                              */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>

#include "wspr.h"		// wspr definitions and functions


#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;


// I/O access
volatile unsigned *gpio;
volatile unsigned *allof7e;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_GET *(gpio+13)  // sets   bits which are 1 ignores bits which are 0

#define ACCESS(base) *(volatile int*)((int)allof7e+base-0x7e000000)
#define SETBIT(base, bit) ACCESS(base) |= 1<<bit
#define CLRBIT(base, bit) ACCESS(base) &= ~(1<<bit)
#define CM_GP0CTL (0x7e101070)
#define GPFSEL0 (0x7E200000)
#define CM_GP0DIV (0x7e101074)


struct GPCTL {
    char SRC         : 4;
    char ENAB        : 1;
    char KILL        : 1;
    char             : 1;
    char BUSY        : 1;
    char FLIP        : 1;
    char MASH        : 2;
    unsigned int     : 13;
    char PASSWD      : 8;
};

void txon()
{

    allof7e = (unsigned *)mmap(
                  NULL,
                  0x01000000,  //len
                  PROT_READ|PROT_WRITE,
                  MAP_SHARED,
                  mem_fd,
                  0x20000000  //base
              );

    if ((int)allof7e==-1) exit(-1);

    SETBIT(GPFSEL0 , 14);
    CLRBIT(GPFSEL0 , 13);
    CLRBIT(GPFSEL0 , 12);

    struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 1,0x5a};
    ACCESS(CM_GP0CTL) = *((int*)&setupword);
}

void txoff()
{
    struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 1,0x5a}; 
    ACCESS(CM_GP0CTL) = *((int*)&setupword); 
}

void setfreq(long freq)
{
    ACCESS(CM_GP0DIV) = (0x5a << 24) + freq;
}

//
// Set up a memory regions to access GPIO
//
void setup_io()
{
    /* open /dev/mem */
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit (-1);
    }

    /* mmap GPIO */

    // Allocate MAP block
    if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
        printf("allocation error \n");
        exit (-1);
    }

    // Make sure pointer is on 4K boundary
    if ((unsigned long)gpio_mem % PAGE_SIZE)
        gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

    // Now map it
    gpio_map = (unsigned char *)mmap(
                   gpio_mem,
                   BLOCK_SIZE,
                   PROT_READ|PROT_WRITE,
                   MAP_SHARED|MAP_FIXED,
                   mem_fd,
                   GPIO_BASE
               );

    if ((long)gpio_map < 0) {
        printf("mmap error %d\n", (int)gpio_map);
        exit (-1);
    }

    // Always use volatile pointer!
    gpio = (volatile unsigned *)gpio_map;


}

void setup_gpios()
{
   int g;
   // Switch GPIO 7..11 to output mode

    /************************************************************************\
     * You are about to change the GPIO settings of your computer.          *
     * Mess this up and it will stop working!                               *
     * It might be a good idea to 'sync' before running this program        *
     * so at least you still have your code changes written to the SD-card! *
    \************************************************************************/

    // Set GPIO pins 7-11 to output
    for (g=7; g<=11; g++) {
        INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
        //OUT_GPIO(g);
    }

}

void
Code_msg (char usr_message[], unsigned long int *N, unsigned long int *M)
{
  unsigned long int n, m;
  unsigned int i, j, power, callsign_length;

  char callsign[7] = "",	// callsign string
    locator[5] = "",		// locator string
    power_str[3] = "";		// power string


  strcpy (callsign, "      ");	// filling with spaces

  i = 0;
  while (usr_message[i] != ' ')
    {
      callsign[i] = usr_message[i];	// extract callsign
      i++;
    }
  callsign_length = i;

  i++;
  j = 0;
  while (usr_message[i] != ' ')
    locator[j++] = usr_message[i++];	// extract locator
  locator[j] = 0;

  i++;
  j = 0;
  while (usr_message[i] != 0)
    power_str[j++] = usr_message[i++];	// extract power
  power_str[j] = 0;

  power = atoi (power_str);	// power needs to be an integer

  // Place a space in first position if third character is not a digit
  if (!isdigit (callsign[2]))
    {
      for (i = callsign_length; i > 0; i--)
	callsign[i] = callsign[i - 1];
      callsign[0] = ' ';
    }

  // callsign encoding:  
  // numbers have a value between 0 and 9 
  // and letters a value between 10 and 35
  // spaces a value of 36
  n = (callsign[0] >= '0'
       && callsign[0] <= '9' ? callsign[0] - '0' : callsign[0] ==
       ' ' ? 36 : callsign[0] - 'A' + 10);
  n = n * 36 + (callsign[1] >= '0'
		&& callsign[1] <= '9' ? callsign[1] - '0' : callsign[1] ==
		' ' ? 36 : callsign[1] - 'A' + 10);
  n = n * 10 + (callsign[2] - '0');	// only number (0-9)
  n = 27 * n + (callsign[3] == ' ' ? 26 : callsign[3] - 'A');	// only space or letter
  n = 27 * n + (callsign[4] == ' ' ? 26 : callsign[4] - 'A');
  n = 27 * n + (callsign[5] == ' ' ? 26 : callsign[5] - 'A');

  // Locator encoding
  m =
    (179 - 10 * (locator[0] - 65) - (locator[2] - 48)) * 180 +
    10 * (locator[1] - 65) + locator[3] - 48;

  // Power encoding
  m = m * 128 + power + 64;

  *N = n;
  *M = m;
}

void
Pack_msg (unsigned long int N, unsigned long int M, unsigned char c[])
{
// Bit packing
// Store in 11 characters because we need 81 bits for FEC correction
  c[0] = N >> 20;		// Callsign
  c[1] = N >> 12;
  c[2] = N >> 4;
  c[3] = N;
  c[3] = c[3] << 4;

  c[3] = c[3] | (M >> 18);	// locator and power
  c[4] = M >> 10;
  c[5] = M >> 2;
  c[6] = M & 0x03;
  c[6] = c[6] << 6;

  c[7] = 0;			// always at 0
  c[8] = 0;
  c[9] = 0;
  c[10] = 0;
}

void
Generate_parity (unsigned char c[], unsigned char symbols[])
{
  unsigned long int Reg0 = 0,	// 32 bits shift register
    Reg1 = 0, result0, result1;
  int count1,			// to count the number
    count2,			// of bits at one
    bit_result = 0, i, j, k, l;

  l = 0;
  for (j = 0; j < 11; j++)	// each byte
    {
      for (i = 7; i >= 0; i--)
	{
	  Reg0 = (Reg0 << 1);
	  Reg0 = Reg0 | (c[j] >> i);	// each bit
	  Reg1 = Reg0;

	  result0 = Reg0 & POLYNOM_1;	// first polynom
	  count1 = 0;

	  for (k = 0; k < 32; k++)	// how many bit at one?
	    {
	      bit_result = result0 >> k;
	      if ((bit_result & 0x01) == 1)
		count1++;
	    }
	  if (count1 % 2 == 1)	// if number of one is odd
	    symbols[l] = 1;	// parity = 1
	  l++;

	  result1 = Reg1 & POLYNOM_2;	// second polynom
	  count2 = 0;

	  for (k = 0; k < 32; k++)	// how many bit at one?
	    {
	      bit_result = result1 >> k;
	      if ((bit_result & 0x01) == 1)
		count2++;
	    }
	  if (count2 % 2 == 1)	// if number of one is odd
	    symbols[l] = 1;	// parity = 1
	  l++;
	}			// end of each bit (32) loop
    }				// end of each byte (11) loop
}

void
Interleave (unsigned char symbols[], unsigned char symbols_interleaved[])
{
  int i, j, k, l, P;

  P = 0;
  while (P < 162)
    {
      for (k = 0; k <= 255; k++)	// bits reverse, ex: 0010 1110 --> 0111 0100
	{
	  i = k;
	  j = 0;
	  for (l = 7; l >= 0; l--)	// hard work is done here...
	    {
	      j = j | (i & 0x01) << l;
	      i = i >> 1;
	    }
	  if (j < 162)
	    symbols_interleaved[j] = symbols[P++];	// range in interleaved table
	}
    }				// end of while, interleaved table is full
}

void
Synchronise (unsigned char symbols_interleaved[],
	     unsigned char symbols_wspr[])
{
  unsigned int sync_word [162]={
    1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,
    0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,
    1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,
    1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,1,1,0,0,0,1,1,0,0,0
  };

  int i;

  for (i = 0; i < 162; i++)
    symbols_wspr[i] = sync_word[i] + 2 * symbols_interleaved[i];
}

void
code_wspr (char* wspr_message, unsigned char* wspr_symbols)
{
  unsigned char symbols_parity[162] = "",	// contains 2*81 parity bits
    symbols_interleaved[162] = "",		// contains parity bits after interleaving
    c_packed[11];		// for bit packing

  unsigned long N,		// for callsign
    M;				// for locator and power


  Code_msg (wspr_message, &N, &M);
  Pack_msg (N, M, c_packed);
  Generate_parity (c_packed, symbols_parity);
  Interleave (symbols_parity, symbols_interleaved);
  Synchronise (symbols_interleaved, wspr_symbols);

}

void calculate_tuning_info(tuning_data* tuning_info)
{
  double divisor; 
  unsigned long decimal_part;
  unsigned long fractional_part;
  double actual_divisor;

  divisor = (double)500000000/tuning_info->requested;
  decimal_part = (unsigned long) divisor;
  fractional_part = (divisor - decimal_part) * (1 << 12);
  tuning_info->tuning_word = decimal_part * (1 << 12) + fractional_part;
  actual_divisor = (double)tuning_info->tuning_word / (float)(1 << 12);

  tuning_info->actual = (double)500000000 / actual_divisor;
}

void sym_to_tuning_words(double base_freq, unsigned char* wspr_symbols, unsigned long* tuning_words)
{
  int i;
  double symbol_freq;
  tuning_data tuning_info[4];


  for (i = 0; i < 4; i++)
  {
    symbol_freq = base_freq + (i-2) * WSPR_OFFSET;
    tuning_info[i].requested = symbol_freq;
    calculate_tuning_info(&tuning_info[i]);
    printf("Symbol %d: Target freq=%fHz, Actual freq=%fHz, Error=%fHz, Tuning Word=%lx\n", i, symbol_freq, tuning_info[i].actual, symbol_freq-tuning_info[i].actual, tuning_info[i].tuning_word);
  }

  for (i = 0; i < 162; i++)
  {
    tuning_words[i] = tuning_info[wspr_symbols[i]].tuning_word;
  }
}

int main(int argc, char *argv[])
{
  char wspr_message[20];          // user beacon message to encode
  unsigned char wspr_symbols[162] = "";   // contains 162 finals symbols
  unsigned long tuning_words[162];
  int i;
  double centre_freq;

  // argv[1]=callsign, argv[2]=locator, argv[3]=power(dBm)
  sprintf(wspr_message, "%-7.7s %-6.6s %.2s", argv[1], argv[2], argv[3]);
  printf("Sending |%s|\n", wspr_message);

  code_wspr(wspr_message, wspr_symbols);

  for (i = 0; i < 162; i++)
    printf("%d, ", wspr_symbols[i]);

  printf("\n");
  centre_freq = atof(argv[4]);
  sym_to_tuning_words(centre_freq, wspr_symbols, tuning_words);

  /* Now we have the list of tuning words, let's transmit them
  Note that this version doesn't check whether we are in a correct timeslot
  */

  setup_io();
  setup_gpios();
  txon();
  for (i = 0; i < 162; i++) {
    setfreq(tuning_words[i]);
    usleep(8192/12000);
  }
  txoff();
  return 0;
}
