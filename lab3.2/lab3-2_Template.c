/* Sample code for Lab 3.2. This code provides a basic start. */

#define RIN 123     // Not actually needed for this lab.

#include "C8051_SIM.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init(void);
void Interrupt_Init(void);
void XBR0_Init(void);
void SMB_Init(void);
//void PCA_ISR ( void );
void ReadCompass(void);
void ReadRanger(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
uint8_t Data[2];        // Data array for use with the SMB/I2C
uint8_t h_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_heading = 0;    // Flag to denote time to read heading
uint8_t r_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_range = 0;    // Flag to denote time to read heading
uint8_t p_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_print = 0;    // Flag to denote time to read heading
uint16_t heading = 0;       // Value of heading
uint16_t range;



//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    Sys_Init();
    Port_Init();
    PCA_Init();
    Interrupt_Init();
    XBR0_Init();
    SMB_Init();


    // Do an initial read of the ranger to get start measurement
    ReadRanger();

    while(1){
        Sim_Update();
        if(new_heading){
            ReadCompass();
            new_heading = 0;
        }
         // Do similar stuff for the compass and the ranger
        if(new_range){
            ReadRanger();
            new_range = 0;
        }
        if(new_print){
            printf("Current heading: %u\n",heading);
            printf("Current range: %u\n",range);
            new_print =0;
        }
}
}
//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{
    // You don't need to initialize the SDA/SCL, leave them as-is!
    P0MDOUT |= 0x30;  //set output pin for CEX0 or CEX2 in push-pull mode
    //P3MDOUT &= 0x3F;  //set output pin for CEX0 or CEX2 in push-pull mode
    P3MDOUT |= ~0xC0; //high impedance
    P3 |= 0xC0;
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{

    XBR0 = 0x1D;  //configure crossbar as directed in the laboratory
     // Should be same as in 3-1

}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
    PCA0MD = 0x81;//SYSCLK/12
    PCA0CPM0 = PCA0CPM1 =0xC2;//CCM0 & CCM2 in 16 bits compare mode
    PCA0CN = 0x40; //enable PCA counter
    EIE1 |= 0x08;//enable PCA interrupt
    EA=1;//enable global interrupt
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up the PCA overflow interrupts
//
void Interrupt_Init()
{
    //Enable interrupts and PCA interrupts specifically
    EA = 1;		// Enable all interrupts
	PCA0MD = 0x01;
}

void SMB_Init()
{
    // Set the clock rate of the SMBus/I2C and
    // Enable the SMBus/I2C (check manual)
    SMB0CR = 0x93;
    ENSMB=1;

}

//-----------------------------------------------------------------------------
// ReadCompass
//-----------------------------------------------------------------------------
//
// Read the heading value from the compass
//
void ReadCompass(void){
    i2c_read_data(0xC0,2,Data,2);   // Read the 0-3600 heading bytes

    heading = ((unsigned int) Data[0]<<8)|Data[1];                  // Put the bytes together

}

//-----------------------------------------------------------------------------
// ReadRanger
//-----------------------------------------------------------------------------
//
// Read the distance value from the ranger and start a ping
//
void ReadRanger(void){
    // Read the first echo from the ranger
    // Put the bytes together and save value to global variable
    // Trigger next measurement

    i2c_read_data(0xE0,2,Data,2);
    range = (Data[0]<<8)|Data[1];

    Data[0]=0x51;
    i2c_write_data(0xE0,0,Data,1);


}


//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR (void)
{
    if(CF){
        CF = 0;
        PCA0=28671; //start count for 20ms
        h_count++;
        r_count++;
        p_count++;

        if(h_count == 2){  // Count 40 ms
            h_count = 0;
            new_heading = 1;
        }
        if(r_count == 4){  // Count 80 ms
            r_count = 0;
            new_range = 1;
        }
        if(p_count == 5){  // Count 100 ms
            p_count = 0;
            new_print = 1;
        }
    }
    PCA0CN = 0x40;
}

