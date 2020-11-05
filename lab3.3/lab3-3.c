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
void one_second(void);
void PCA_ISR ( void );
void ReadCompass(void);
void ReadRanger(void);

void Set_Motor_Pulsewidth(uint16_t distance);
void Set_Servo_Pulsewidth(uint16_t angle);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
#define SS1    P3_5
#define SS2    P3_6
#define SS3    P3_7

uint8_t Data[2];        // Data array for use with the SMB/I2C
uint8_t h_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_heading = 0;    // Flag to denote time to read heading
uint8_t r_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_range = 0;    // Flag to denote time to read heading
uint8_t p_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_print = 0;    // Flag to denote time to read heading
uint16_t heading = 0;       // Value of heading
uint16_t range;

unsigned int MOTOR_CENTER = 2765; //= 1.5ms -> 2764.8 ;
unsigned int MOTOR_MIN =2028; //= 1.1ms ->2027.52;
unsigned int MOTOR_MAX =3502; //= 1.9ms -> 3502.08;
unsigned int MOTOR_PW = 2765;
unsigned int K_m;//Gain

// Do the same thing for servo pulse widths
unsigned int SERVO_CENTER= 2765; //= 1.5ms -> 2764.8;
unsigned int SERVO_MIN = 1659; //= 0.9ms -> 1658.88;
unsigned int SERVO_MAX = 3871; //= 2.1ms -> 3870.72;
unsigned int SERVO_PW = 2765;
unsigned int K_s;//Gain

unsigned int Counts;

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


    printf("START 1 SEC\n");

    // set the MOTOR PCA output to a neutral setting and wait 1 second
    PCA0CP0 = 0xFFFF-SERVO_CENTER;
    PCA0CP1 = 0xFFFF-MOTOR_CENTER;

    //MOTOR_PW = MOTOR_CENTER;
    //SERVO_PW = SERVO_CENTER;
    Counts = 0;
    one_second();
            printf("HIIII\n");
    // Do an initial read of the ranger to get start measurement
    ReadRanger();

    printf("START\n");
    while(1){
        Sim_Update();
        if(new_heading){//40ms
            new_heading = 0;//clear the new heading flag
            ReadCompass();//read & stored in variable: heading
            Set_Servo_Pulsewidth(heading);
        }
         // Do similar stuff for the compass and the ranger
        if(new_range){//80ms
            new_range = 0;
            ReadRanger();
            Set_Motor_Pulsewidth(range);
        }
        if(new_print){//100ms
            new_print =0;
            printf("Current heading: %u\n",heading);
            printf("Current range: %un\n",range);
            printf("Current Motor_PW: %un\n",MOTOR_PW);
            printf("Current Servo_PW: %un\n",SERVO_PW);

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
        Counts++;

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
void Set_Motor_Pulsewidth(uint16_t distance)//distance
{
    K_m = (MOTOR_MAX-MOTOR_CENTER)/(200-100);//get the gain
    if(distance < 100)  // single character input to increase the pulsewidth
    {
        MOTOR_PW= MOTOR_MIN; // towards forward (+pca0counts)
    }
    else if(distance > 300)  // single character input to decrease the pulsewidth
    {
        MOTOR_PW = MOTOR_MAX; // towards reverse (-pca0counts)
    }
        else   //
    {
        MOTOR_PW = MOTOR_CENTER+K_m*(200-distance); //
    }

    // Check if slide switch is on or not and act if needed
    if(!SS2){//slide switch (P3.6) is off
        MOTOR_PW = MOTOR_CENTER; //set to neutural

    }
    // Assign MOTOR_PW to the PCA...
    //Apply the calculated pulse width to the PCA0 CCM for the Drive Motor
    PCA0CP1 = 0xFFFF-MOTOR_PW;
}

void Set_Servo_Pulsewidth(uint16_t angle)
{

    unsigned int modified_heading;
    int erro = 0-angle;

    if(angle<1800){
        modified_heading = angle;

        //SERVO_PW = SERVO_CENTER + K_s*(0-);
    }
    else if(angle>1800){
            modified_heading = angle-3600;
    }
    K_s = (SERVO_MAX-SERVO_CENTER)/750;
    /*
    if(angle>750){//completely left
        SERVO_PW = SERVO_MAX;

    }else if(angle<-750){//completely right
        SERVO_PW = SERVO_MIN;
    }
    else{
        */
        //NORTH
        SERVO_PW = SERVO_CENTER + K_s*(0-modified_heading);
    //}
    if(SERVO_PW > SERVO_MAX){
        SERVO_PW = SERVO_MAX;
    }else if(SERVO_PW < SERVO_MIN){
        SERVO_PW = SERVO_MIN;
    }

    // Check if slide switch is on or not and act if needed
    if(!SS3){//slide switch (P3.7) is off
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
