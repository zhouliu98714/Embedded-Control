/* Sample code for Lab 3.1. This code provides a basic start. */

#define RIN 123     // Not actually needed for this lab.

#include "C8051_SIM.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void Interrupt_Init(void);
void XBR0_Init();
void Set_Motor_Pulsewidth(uint8_t user_input);
void Set_Servo_Pulsewidth(uint8_t user_input);
void one_second(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// Global variables for the important motor pulse widths
unsigned int MOTOR_CENTER = 2765; //= 1.5ms -> 2764.8 ;
unsigned int MOTOR_MIN =2028; //= 1.1ms ->2027.52;
unsigned int MOTOR_MAX =3502; //= 1.9ms -> 3502.08;
unsigned int MOTOR_PW = 0;

// Do the same thing for servo pulse widths
unsigned int SERVO_CENTER= 2765; //= 1.5ms -> 2764.8;
unsigned int SERVO_MIN = 1659; //= 0.9ms -> 1658.88;
unsigned int SERVO_MAX = 3871; //= 2.1ms -> 3870.72;
unsigned int SERVO_PW = 0;

unsigned int Counts;
//Ports
//#define MT    P0_5
//#define SV    P0_4

#define SS1    P3_5
#define SS2    P3_6
#define SS3    P3_7
//-----------------------------------------------------------------------------
// Main Function
//-----------------------------------------------------------------------------
void main(void)
{
    // initialize board
    Sys_Init();
    putchar(' '); //the quotes in this line may not format correctly
    Port_Init();
    XBR0_Init();
    PCA_Init();
    Interrupt_Init();
    printf("START 1 SEC");

    // set the MOTOR PCA output to a neutral setting and wait 1 second
    MOTOR_PW = MOTOR_CENTER;
    Counts = 0;
    one_second();
    //__________________________________________
    //__________________________________________
    //__________________________________________

    // Print instructions
    printf("START");

    while(1){
        Sim_Update();
        uint8_t input = getchar();
        Set_Motor_Pulsewidth(input);
        Set_Servo_Pulsewidth(input);
        printf("MOTOR_PW: %u\t SERVO_PW: %u\r\n", MOTOR_PW,SERVO_PW);
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
    P0MDOUT |= 0x30;  //set output pin for CEX0 or CEX2 in push-pull mode
    P3MDOUT &= 0x1F;  //set output pin for CEX0 or CEX2 in push-pull mode
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

}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
    // reference to the sample code in Example 4.5 -Pulse Width Modulation
    // implemented using the PCA (Programmable Counter Array), p. 50 in Lab Manual
    PCA0MD = 0x81;//SYSCLK/12
    PCA0CPM0 = PCA0CPM2 =0xC2;//CCM0 & CCM2 in 16 bits compare mode
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

//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void )
{
    // reference to the sample code in Example 4.5 -Pulse Width Modulation
    // implemented using the PCA (Programmable Counter Array), p. 50 in Lab Manual.
    if(CF){
        CF=0;
        PCA0=28671; //start count for 20ms
        Counts++;

    }
    PCA0CN =0x40;
}

void Set_Motor_Pulsewidth(uint8_t user_input)
{
    if(user_input == 'f')  // single character input to increase the pulsewidth
    {
        MOTOR_PW+=20; // towards forward (+pca0counts)
    }
    else if(user_input == 's')  // single character input to decrease the pulsewidth
    {
        MOTOR_PW-=20; // towards reverse (-pca0counts)
    }
    // Check Limits
    if(MOTOR_PW > MOTOR_MAX){
        // do something
        MOTOR_PW = MOTOR_MAX;
    }else if(MOTOR_PW < MOTOR_MIN){
        // do something else
        MOTOR_PW = MOTOR_MIN;
    }
    // Check if slide switch is on or not and act if needed
    if(SS2){//slide switch (P3.6) is off
        MOTOR_PW = MOTOR_CENTER; //set to neutural

    }
    printf("hello");
    // Assign MOTOR_PW to the PCA...
    //Apply the calculated pulse width to the PCA0 CCM for the Drive Motor
    PCA0CP2 = 0xFFFF-MOTOR_PW;


}

void Set_Servo_Pulsewidth(uint8_t user_input)
{
    // Essentially just repeat what Set_Motor_Pulsewidth does for the servo...
        if(user_input == 'l')  // single character input to increase the pulsewidth
    {
        SERVO_PW+=20; // towards forward (+pca0counts)
    }
    else if(user_input == 'r')  // single character input to decrease the pulsewidth
    {
        SERVO_PW-=20; // towards reverse (-pca0counts)
    }
    // Check Limits
    if(SERVO_PW > SERVO_MAX){
        // do something
        SERVO_PW = SERVO_MAX;
    }else if(SERVO_PW < SERVO_MIN){
        // do something else
        SERVO_PW = SERVO_MIN;
    }


    // Check if slide switch is on or not and act if needed
    if(SS3){//slide switch (P3.7) is off
            SERVO_PW = SERVO_CENTER; //set to neutural

    }


    // Assign MOTOR_PW to the PCA...
    //Apply the calculated pulse width to the PCA0 CCM for the Drive Motor
    PCA0CP0 = 0xFFFF-SERVO_PW;


}
void one_second(void){
    //wait for 1 sec
    while(Counts<28){
        Sim_Update();
        //printf("Counts: %d\n",Counts);
    }
}
