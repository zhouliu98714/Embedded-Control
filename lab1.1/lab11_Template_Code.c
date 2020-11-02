/*  Name: Liu Zhou
    Date: 9/23/2020
    Program description:
        1. The slide switch is “off”: all outputs are off.
        2. The slide switch is “on”: the LED is on and ...
            (a) Pushbutton 1 is pressed: Bi-color LED is green
            (b) Pushbutton 2 is pressed: Bi-color LED is red
            (c) Both Pushbuttons are pressed: Bi-color LED is green and the motor is on.

*/
/* ENGR-2350 FALL 2020 - Lab 1-1 Template Code
  This program is incomplete. Part of the code is provided as an example. You
  need to modify the code, adding code to satisfy the stated requirements. Blank
  lines have also been provided at some locations, indicating an incomplete line.
 * */

#define RIN 661672682         // Put your RIN here.  It will be used to generate the Port Pins that you need to implement

//#include <c8051_SDCC.h> // File we would use if we were using the normal development environment
#include "C8051_SIM.h"
#include<stdio.h>
#include<stdlib.h>

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);  // Initialize ports for input and output
void Set_outputs(void);// function to set output bits

//-----------------------------------------------------------------------------
// sbits
//-----------------------------------------------------------------------------
//__sbit __at 0xB6 PB0; // EXAMPLE ONLY: Format we would use in the lab with an actual C8051
//#define PB0 P3_6    // EXAMPLE ONLY: Format we use with the simulator

//#define PB1 P1_0    // pushbutton 1 associated with Port 1 pin 0 (FIX ME!!!)
//#define SS P0_7     // Slideswitch associated with Port 0 pin 7 (FIX ME!!!)

#define PB1 P3_3 // pushbutton 1 associated with Port 3 pin 3
#define LED1 P3_5
#define SS P3_7 // Slideswitch associated with Port 3 pin 7
#define PB2 P3_0 // pushbutton 2 associated with Port 3 pin 0
#define MT P3_4
#define Bi_LED0 P3_1
#define Bi_LED1 P2_6
// Fill in more for the rest of the inputs/outputs


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// We dont need any for this program!


//***************
// Main program

void main(void)
{
    Sys_Init();        // System Initialization
    putchar(' ');
    Port_Init();       // Initialize ports 2 and 3

    while (1)          // infinite loop
    {
        Sim_Update();   // Call this in every loop! The simulator will not work without

        // main program manages the function calls
        Set_outputs();
    }
}


//***************
/* Port_Init - Initializes Ports 2 and 3 in the desired modes for input and output */

void Port_Init(void)
{
    // Port 3
  P3MDOUT =0x32; // set Port 3 output pins to push-pull mode (fill in the blank)
  P3 |= ~0x32; // set Port 3 input pins to high impedance state (fill in the blank)

    // Port 2
    P2MDOUT =0x40;
    P2 |= ~0x40;
}



//***************
/* Set outputs:
    The following code is incomplete, lighting an LED depending
    on the state of the slide switch. */

void Set_outputs(void)
{
    if (!SS)        // if Slide Switch activated (On position)
    {

        LED1 = 0;   // turn on LED1
        printf("\r Slide switch is on    \n");
        if (PB1 == 0 && PB2 == 0){ //PB1 & PB2 both pushed
            printf("\r Pushbutton 1 and 2 ACTIVATED\n");

            Bi_LED0 = 1;
            //Bi_LED1 = 0;
            MT = 0;
        }
        else if(PB1==1 && PB2==0){ //PB1 pushed
            printf("\r Pushbutton 1 ACTIVATED \n");
            Bi_LED0 = 1; //Green light
            MT = 1; //Motor off
        }
        else if(PB1==0 && PB2==1){
            printf("\r Pushbutton 2 ACTIVATED \n");

            Bi_LED1 = 1;//Red light
            MT = 1;
        }
        else{
            Bi_LED0 = 0; //Bi Led off
            Bi_LED1 = 0;//Bi Led off
            MT = 1;//Motor off
        }
    }
    else            // if Slide Switch is not activated (Off position)
    {
        //LED0 = 1;   // turn off LED0
        printf("\r Slide switch is off   \n");
        MT = 1;//Motor off
        LED1 = 1;//LED1 off
        Bi_LED0 = 0; //Bi Led off
        Bi_LED1 = 0;//Bi Led off
    }
}
