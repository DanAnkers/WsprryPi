/* Functions for wspr encoding


F8CHK   29/03/2011  */

#ifndef __WSPR_H
#define __WSPR_H

void Code_msg(char[], unsigned long int*, unsigned long int*);      // encode callsign, locator and power
void Pack_msg(unsigned long int, unsigned long int, unsigned char[]);// packed 50 bits in 11 bytes
void Generate_parity(unsigned char[], unsigned char[]);  // generate 162 parity bits
void Interleave( unsigned char[], unsigned char[]);  // interleave the 162 parity bits
void Synchronise(unsigned char[], unsigned char[]);  // synchronize with a pseudo random pattern

void code_wspr(void);		// encode the wspr message
void go_wspr(void);			// start WSPR beacon mode
void go_wspr_tx(void);		// set cube in wspr tx mode

#define WSPR_OFFSET  1.4648  	//  tone separation

#define POLYNOM_1 0xf2d05351	// polynoms for
#define POLYNOM_2 0xe4613c47	// parity generator

#endif
