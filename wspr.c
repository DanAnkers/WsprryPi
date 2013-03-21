/*

 Raspberry Pi bareback LF/MF/HF/VHF WSPR transmitter

 Makes a very simple WSPR beacon from your RasberryPi by connecting GPIO 
 port to Antanna (and LPF), operates on LF, MF, HF and VHF bands from 
 0 to 250 MHz.


License:
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

Credits:
  Credits goes to Oliver Mattos and Oskar Weigl who implemented PiFM [1]
  based on the idea of exploiting RPi DPLL as FM transmitter. Dan MD1CLV
  combined this effort with WSPR encoding algorithm from F8CHK, resulting  
  in WsprryPi a WSPR beacon for LF and MF bands. Guido PE1NNZ extended 
  this effort with DMA based PWM modulation of fractional divider that was 
  part of PiFM, allowing to operate the WSPR beacon also on HF and VHF bands.
  
  [1] PiFM code from http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter

To use:
  In order to transmit legally, a HAM Radio License is required for running 
  this experiment. The output is a square wave so a low pass filter is REQUIRED.
  Connect a low-pass filter to GPIO4 (GPCLK0) and Ground pins on your 
  Raspberry Pi, connect an antenna to the LPF. The GPIO4 and GND pins can be 
  found on header P1 pin 7 and 9 respectively, the pin closest to P1 label is 
  pin 1 and its  3rd and 4th neighbour is pin 7 and 9 respectively, see this 
  link for pin layout: http://elinux.org/RPi_Low-level_peripherals
  The expected power output is 10mW (+10dBm) in a 50 Ohm load. This looks
  neglible, but when connected to a simple dipole antenna this may result in 
  reception reports ranging up to several thousands of kilometers.

  This software is using system time to determine the start of a WSPR 
  transmissions, so keep the system time synchronised within 1sec precision, 
  i.e. use NTP network time synchronisation or set time manually with date 
  command. Reception reports are logged on Weak Signal Propagation Reporter 
  Network: http://wsprnet.org/drupal/wsprnet/spots

  As the WSPR band is only 200 Hz wide, some frequency calibration may be needed 
  to ensure that the transmission is done within the WSPR band. You can correct
  the frequency error manually in the command line or adjust CAL_PLL_CLK in the
  code. 

Usage: 
  sudo ./wspr <callsign> <locator> <power in dBm> <frequency in Hz>
        e.g.: sudo ./wspr K1JT FN20 10 7040074

  WSPR is used on the following frequencies (local restriction may apply):
     LF   137400 - 137600
     MF   475600 - 475800
    160m  1838000 - 1838200
     80m  3594000 - 3594200
     60m  5288600 - 5288800
     40m  7040000 - 7040200
     30m  10140100 - 10140300
     20m  14097000 - 14097200
     17m  18106000 - 18106200
     15m  21096000 - 21096200
     12m  24926000 - 24926200
     10m  28126000 - 28126200
      6m  50294400 - 50294600
      4m  70092400 - 70092600
      2m  144490400 -144490600

Compile:
  gcc -lm -std=c99 wspr.c -owspr

*/




#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <dirent.h>
#include <math.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <malloc.h>
#include <time.h>

void Code_msg(char[], unsigned long int*, unsigned long int*);      // encode callsign, locator and power
void Pack_msg(unsigned long int, unsigned long int, unsigned char[]);// packed 50 bits in 11 bytes
void Generate_parity(unsigned char[], unsigned char[]);  // generate 162 parity bits
void Interleave( unsigned char[], unsigned char[]);  // interleave the 162 parity bits
void Synchronise(unsigned char[], unsigned char[]);  // synchronize with a pseudo random pattern

void code_wspr(char* wspr_message, unsigned char* wspr_symbol);         // encode the wspr message
void go_wspr(void);                     // start WSPR beacon mode
void go_wspr_tx(void);          // set cube in wspr tx mode

#define CAL_PWM_CLK   (31500000 * 1.078431372549019607843137254902)         // calibrated PWM clock
#define CAL_PLL_CLK   (500000000.0 * 0.99993565799251550864072437977875)    // calibrated PLL reference clock 

#define WSPR_SYMTIME (8192.0/12000.0)  // symbol time
#define WSPR_OFFSET  (1.0/WSPR_SYMTIME)     //  tone separation

#define POLYNOM_1 0xf2d05351    // polynoms for
#define POLYNOM_2 0xe4613c47    // parity generator

/* RF code: */

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;


// I/O access
volatile unsigned *gpio = NULL;
volatile unsigned *allof7e = NULL;

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
#define CLKBASE (0x7E101000)
#define DMABASE (0x7E007000)
#define PWMBASE  (0x7e20C000) /* PWM controller */

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

void getRealMemPage(void** vAddr, void** pAddr) {
    void* a = (void*)valloc(4096);

    ((int*)a)[0] = 1;  // use page to force allocation.

    mlock(a, 4096);  // lock into ram.

    *vAddr = a;  // yay - we know the virtual address

    unsigned long long frameinfo;

    int fp = open("/proc/self/pagemap", 'r');
    lseek(fp, ((int)a)/4096*8, SEEK_SET);
    read(fp, &frameinfo, sizeof(frameinfo));

    *pAddr = (void*)((int)(frameinfo*4096));
}

void freeRealMemPage(void* vAddr) {

    munlock(vAddr, 4096);  // unlock ram.

    free(vAddr);
}

struct CB {
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int RES1;
    volatile unsigned int RES2;

};

struct DMAregs {
    volatile unsigned int CS;
    volatile unsigned int CONBLK_AD;
    volatile unsigned int TI;
    volatile unsigned int SOURCE_AD;
    volatile unsigned int DEST_AD;
    volatile unsigned int TXFR_LEN;
    volatile unsigned int STRIDE;
    volatile unsigned int NEXTCONBK;
    volatile unsigned int DEBUG;
};

struct PageInfo {
    void* p;  // physical address
    void* v;   // virtual address
};

struct PageInfo constPage;
struct PageInfo instrPage;
struct PageInfo instrs[1024];

double fracs[1024];

void txon()
{
    if(allof7e == NULL){
      allof7e = (unsigned *)mmap(
                  NULL,
                  0x01000000,  //len
                  PROT_READ|PROT_WRITE,
                  MAP_SHARED,
                  mem_fd,
                  0x20000000  //base
              );
      if ((int)allof7e==-1) exit(-1);
    }

    SETBIT(GPFSEL0 , 14);
    CLRBIT(GPFSEL0 , 13);
    CLRBIT(GPFSEL0 , 12);

    struct GPCTL setupword = {6/*SRC*/, 1, 0, 0, 0, 1,0x5a};
    ACCESS(CM_GP0CTL) = *((int*)&setupword);
}

void txoff()
{
    struct GPCTL setupword = {6/*SRC*/, 0, 0, 0, 0, 1,0x5a}; 
    ACCESS(CM_GP0CTL) = *((int*)&setupword); 
}

void setfreq(long freq)
{
    ACCESS(CM_GP0DIV) = (0x5a << 24) + freq;
}

void txSym(int sym, float tsym)
{
    int bufPtr=0;
    short data;
    int iter = 1400; //4000
    int clocksPerIter = (int)(CAL_PWM_CLK*tsym/(float)iter);
    //printf("tsym=%f iter=%u clocksPerIter=%u tsymerr=%f\n", tsym, iter, clocksPerIter, tsym - ((float)clocksPerIter*(float)iter)/CAL_PWM_CLK );
    int i = sym*3 + 511;
    double dval = fracs[i]; // ratio between 0 and 1 of frequency position that is in between two fractional clock divider bins
    int k = (int)(round(dval));  // integer component
    double frac = (dval - (double)k)/2 + 0.5;
    unsigned int fracval = (frac*clocksPerIter);
    //printf("i=%d *i=%u %u fracval=%u dval=%f sym=%d\n", i, ((int*)(constPage.v))[i-1], ((int*)(constPage.v))[i+1], fracval, dval, sym); 
    for(int j=0; j!=iter; j++){
        bufPtr++;
        while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (int)(instrs[bufPtr].p)) usleep(1000);
        ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (int)constPage.p + (i-1)*4;

        bufPtr++;
        while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (int)(instrs[bufPtr].p)) usleep(1000);
        ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = clocksPerIter-fracval;

        bufPtr++;
        while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (int)(instrs[bufPtr].p)) usleep(1000);
        ((struct CB*)(instrs[bufPtr].v))->SOURCE_AD = (int)constPage.p + (i+1)*4;

        bufPtr=(bufPtr+1) % (1024);
        while( ACCESS(DMABASE + 0x04 /* CurBlock*/) ==  (int)(instrs[bufPtr].p)) usleep(1000);
        ((struct CB*)(instrs[bufPtr].v))->TXFR_LEN = fracval;
    }
}

void unSetupDMA(){
    printf("exiting\n");
    struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
    DMA0->CS =1<<31;  // reset dma controller
    txoff(); 
}

void handSig() {
  exit(0);
}
void setupDMATab( float centerFreq, double symOffset ){
   // make data page contents - it's essientially 1024 different commands for the
   // DMA controller to send to the clock module at the correct time.
  for (int i=1; i<1023; i+=3){
     double freq = centerFreq + ((double)(-511 + i))*symOffset/3.0;
     double divisor = CAL_PLL_CLK/freq;
     unsigned long integer_part = (unsigned long) divisor;
     unsigned long fractional_part = (divisor - integer_part) * (1 << 12);
     unsigned long tuning_word = (0x5a << 24) + integer_part * (1 << 12) + fractional_part;
     if(fractional_part == 0 || fractional_part == 1023){
       printf("warning: symbol %u unusable because fractional divider is out of range, try near frequency.\n", i/3);
     }
     ((int*)(constPage.v))[i-1] = tuning_word - 1;
     ((int*)(constPage.v))[i] = tuning_word;
     ((int*)(constPage.v))[i+1] = tuning_word + 1;
     double actual_freq = CAL_PLL_CLK/((double)integer_part + (double)fractional_part/(double)(1<<12));
     double freq_corr = freq - actual_freq;
     double delta = CAL_PLL_CLK/((double)integer_part + (double)fractional_part/(double)(1<<12)) - CAL_PLL_CLK/((double)integer_part + ((double)fractional_part+1.0)/(double)(1<<12));
     double resolution = 2.0 * delta / pow(2,32);
     if(resolution > symOffset ){
       printf("warning: PWM/PLL fractional divider has not enough resolution: %f while %f is required , try lower frequency.\n", resolution, symOffset);
       exit(0);
     }
     fracs[i] = -1*freq_corr/delta;
     //printf("i=%u f=%f fa=%f corr=%f delta=%f percfrac=%f int=%u frac=%u tuning_word=%u resolution=%fimHz\n", i, freq, actual_freq, freq_corr, delta, fracs[i], integer_part, fractional_part, tuning_word, resolution *1000);
   }
}

void setupDMA(){
   atexit(unSetupDMA);
   signal (SIGINT, handSig);
   signal (SIGTERM, handSig);
   signal (SIGHUP, handSig);
   signal (SIGQUIT, handSig);

   // allocate a few pages of ram
   getRealMemPage(&constPage.v, &constPage.p);
 
   int instrCnt = 0;
  
   while (instrCnt<1024) {
     getRealMemPage(&instrPage.v, &instrPage.p);
    
     // make copy instructions
     struct CB* instr0= (struct CB*)instrPage.v;
    
     for (int i=0; i<4096/sizeof(struct CB); i++) {
       instrs[instrCnt].v = (void*)((int)instrPage.v + sizeof(struct CB)*i);
       instrs[instrCnt].p = (void*)((int)instrPage.p + sizeof(struct CB)*i);
       instr0->SOURCE_AD = (unsigned int)constPage.p+2048;
       instr0->DEST_AD = PWMBASE+0x18 /* FIF1 */;
       instr0->TXFR_LEN = 4;
       instr0->STRIDE = 0;
       //instr0->NEXTCONBK = (int)instrPage.p + sizeof(struct CB)*(i+1);
       instr0->TI = (1/* DREQ  */<<6) | (5 /* PWM */<<16) |  (1<<26/* no wide*/) ;
       instr0->RES1 = 0;
       instr0->RES2 = 0;

       if (i%2) {
         instr0->DEST_AD = CM_GP0DIV;
         instr0->STRIDE = 4;
         instr0->TI = (1<<26/* no wide*/) ;
       }

       if (instrCnt!=0) ((struct CB*)(instrs[instrCnt-1].v))->NEXTCONBK = (int)instrs[instrCnt].p;
       instr0++;
       instrCnt++;
     }
   }
   ((struct CB*)(instrs[1023].v))->NEXTCONBK = (int)instrs[0].p;

   // set up a clock for the PWM
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000026;
   usleep(1000);
   ACCESS(CLKBASE + 41*4 /*PWMCLK_DIV*/)  = 0x5A002800;
   ACCESS(CLKBASE + 40*4 /*PWMCLK_CNTL*/) = 0x5A000016;
   usleep(1000);

   // set up pwm
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = 0;
   usleep(1000);
   ACCESS(PWMBASE + 0x4 /* status*/) = -1;  // clear errors
   usleep(1000);
   ACCESS(PWMBASE + 0x0 /* CTRL*/) = -1; //(1<<13 /* Use fifo */) | (1<<10 /* repeat */) | (1<<9 /* serializer */) | (1<<8 /* enable ch */) ;
   usleep(1000);
   ACCESS(PWMBASE + 0x8 /* DMAC*/) = (1<<31 /* DMA enable */) | 0x0707;

   //activate dma
   struct DMAregs* DMA0 = (struct DMAregs*)&(ACCESS(DMABASE));
   DMA0->CS =1<<31;  // reset
   DMA0->CONBLK_AD=0;
   DMA0->TI=0;
   DMA0->CONBLK_AD = (unsigned int)(instrPage.p);
   DMA0->CS =(1<<0)|(255 <<16);  // enable bit = 0, clear end flag = 1, prio=19-16
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

/*
WSPR encoding module:
Thanks to K1JT, G4JNT and PE1NNZ for publishing
helping infos.

Encoding process is in 5 steps:
   * bits packing of user message in 50 bits
   * store the 50 bits dans 11 octets (88 bits and only 81 useful)
   * convolutionnal encoding with two pariy generators (-> 162 bits)
   * interleaving of the 162 bits with bit-reverse technique
   * synchronisation with a psudo-random vector to obtain the
      162 symbols defining one frequency of 4.

 F8CHK 29/03/2011                              */

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
      callsign[i] = islower(usr_message[i])?toupper(usr_message[i]):usr_message[i];	// extract callsign
      i++;
    }
  callsign_length = i;

  i++;
  j = 0;
  while (usr_message[i] != ' ')
    locator[j++] = islower(usr_message[i])?toupper(usr_message[i++]):usr_message[i++];	// extract locator
  locator[j] = 0;

  i++;
  j = 0;
  while (usr_message[i] != 0)
    power_str[j++] = usr_message[i++];	// extract power
  power_str[j] = 0;

  power = atoi (power_str);	// power needs to be an integer

  printf("Call: %s / Locator: %s / Power: %ddBm\n", callsign, locator, power);

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

void wait_even_minute()
{
  time_t t;
  struct tm* ptm;
  for(;;){
    time(&t); 
    ptm = gmtime(&t);
    if((ptm->tm_min % 2) == 0 && ptm->tm_sec == 1) break;
    usleep(1000);
  }
}

int main(int argc, char *argv[])
{
  char wspr_message[20];          // user beacon message to encode
  unsigned char wspr_symbols[162] = {};
  unsigned long tuning_words[162];
  int i;
  double centre_freq;
  int nbands = argc - 4;
  int band = 0;

  if(argc < 5){
    printf("Usage: wspr <callsign> <locator> <power in dBm> [<frequency in Hz> ...]\n");
    printf("\te.g.: sudo ./wspr PE1NNZ JO21 10 7040074\n");
    printf("\tchoose freq in range +/-100 Hz around one of center frequencies: 137500, 475700, 1838100, 3594100, 5288700, 7040100, 10140200, 14097100, 18106100, 21096100, 24926100, 28126100, 50294500, 70092500, 144490500 Hz\n");
    return 1;
  }
  // argv[1]=callsign, argv[2]=locator, argv[3]=power(dBm)
  sprintf(wspr_message, "%s %s %s", argv[1], argv[2], argv[3]);
  printf("Sending |%s|\n", wspr_message);

  code_wspr(wspr_message, wspr_symbols);
  printf("Symbols: ");
  for (i = 0; i < 162; i++)
    printf("%d,", wspr_symbols[i]);
  printf("\n");
  setup_io();
  setup_gpios();
  txon();
  setupDMA();
  printf("Transmission starts on even minute...\n");

  for(;;)
  {
    txoff();
    centre_freq = atof(argv[band + 4]);
    band++;
    if(band >= nbands)
      band = 0;
    setupDMATab(centre_freq, WSPR_OFFSET);
    wait_even_minute();
    txon();
    printf("%f\n", centre_freq);
    for (i = 0; i < 162; i++) {
      txSym(wspr_symbols[i], WSPR_SYMTIME);
    }
  }
  return 0;
}
