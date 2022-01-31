/* ENGR-2050 SUMMER 2020 - MiniProject
 * **** Template Code FOR ENGR-2350 LAB 4 ****
 * This file must be modified for IED Mini Project
 * */

//#define PRINTTOFILE     "FILENAME.csv"
#define RIN             662008784

#include "C8051_SIM.h"
#include<stdio.h>
#include<stdlib.h>

void Port_Init(void);               // Initialize all the GPIO ports (I/O only)
void PCA_Init(void);                // Initialize PCA function
void XBR_Init(void);                // Initialize Crossbar function
void Interrupt_Init(void);          //Initialize interrupts function
void SMB_Init(void);                // Initialize I2C function
void read_ranger(void);             // Function to read the ranger and start a ping
void read_compass(void);            // Function to read the compass heading
void set_servoPW(signed int user_input);
void set_motorPW(signed int user_input);
void ReadRanger(uint8_t direction);

signed int MOTOR_NEUT = 2765;       // Neutral              1.5 ms
signed int MOTOR_MAX = 3502;        // Full Forward         1.9 ms
signed int MOTOR_MIN = 2027;        // Full Reverse         1.1 ms
signed int MOTOR_PW=0;                // Current PW

signed int SERVO_CENTER = 2765;     // Centered             1.5 ms
signed int SERVO_RIGHT = 3871;      // All the way right    2.1 ms
signed int SERVO_LEFT = 1659;       // All the way left     0.9 ms
signed int SERVO_PW=0;                // Current PW

unsigned char counts = 0;           // PCA overflow counter
unsigned char new_range = 0;        // Flag denoting if a new range value should be retrieved (set in interrupt)
unsigned char new_heading = 0;      // Flag denoting if a new heading value should be retrieved (set in interrupt)
unsigned char new_print = 0;        // Flag denoting that we can print (set in interrupt)
unsigned char r_count = 0;          // Counter to count for ranger delay
unsigned char h_count = 0;          // Counter to count for compass delay
unsigned char p_count = 0;          // Counter to count print delay
signed int heading = 0;             // Variable to hold compass heading value
unsigned int range = 0;             // Variable to hold ranger distance value
unsigned int time = 0;
uint8_t Data[2];
signed int des_heading = 0;         // Desired heading
signed int heading_error = 0;       // Error calculation
uint16_t distance = 0;
uint16_t distancef = 0;
uint16_t distancel = 0;
uint16_t distancer = 0;
uint8_t front = 0xE0;
uint8_t left = 0xE4;
uint8_t right = 0xE6;

uint8_t state = 0;

uint16_t desired_distance = 50;
signed int distance_error = 0;

float ksteer = 6;                   // Proportional Gain constant for steering
//float kdrive = _;                   // Proportional Gain constant for Drive

//__sbit __at 0xB0 SS;        // Sbit on Port 3 pin 0
#define SS P3_0           // Simulator Sbit on Port 3 pin 0
//__sbit __at 0xB0 PB;
#define PB P2_1

void main(void){

    Sys_Init();
    putchar(0);
    Port_Init();
    PCA_Init();
    Interrupt_Init();
    XBR_Init();
    SMB_Init();

    // Read ranger once to trigger ping.

    // Initialize the drive motor to neutral for 1 s
    // Set motor pulsewidth to neutral
    read_compass();
    ReadRanger(front);
    distancef = distance;
    ReadRanger(left);
    distancel = distance;
    ReadRanger(right);
    distancer = distance;
    MOTOR_PW = 0xFFFF-MOTOR_NEUT;
    PCA0CP2 = MOTOR_PW;
    SERVO_PW = 0xFFFF-SERVO_CENTER;
    PCA0CP0 = SERVO_PW;
    printf("Initializing motors, please wait.\n");
    while(counts < 200){ // Wait some seconds
        Sim_Update();   // Called in all loops!
    }

    // Clear the flag counter variables to prevent a double read
    r_count = h_count = p_count = 0;
    new_range = new_heading = new_print = 0;



    // Make sure pushbutton is not pressed before starting
    while (!PB) {Sim_Update();}
    printf("\nPush Button Released... ");
    if (!SS) {printf("\nPlease toggle the slide switch on. ");}
    while (!SS) {Sim_Update();}
    printf("Beginning Simulation:\n");
    // Print data headers
    printf("Time\tHeading\tRangeF\tRangeL\tRangeR\tServoPW\tMotorPW\tState\n");
    // Run program loop
    // while(1) loop may or may not be needed, depending on how it's implemented.
    while(PB){
        set_motorPW(30000);
        // This stuff below is close to what a team should have had at the end of LITEC Lab3-3 (with pieces missing)
        if (new_range){
            // Get distance and act upon it
            ReadRanger(front);
            distancef = distance;
            ReadRanger(left);
            distancel = distance;
            ReadRanger(right);
            distancer = distance;
            distance_error = desired_distance - distancer;
            new_range = 0;
        }
        if (new_heading){
            // Get heading and act upon it
            read_compass();
            new_heading=0;
        }
        if (new_print){
            // Print import stuff
            new_print = 0;
            time++;
            printf("%d\t%d\t%u\t%u\t%u\t%u\t%u\t%d\n", time, heading, distancef, distancel, distancer, PCA0CP0, PCA0CP2, state);
        }
        if (distancer < 330) {state = 1;}
        if (state == 1) {
            // idk man make it turn towards the target
        }
        if (true /* idk smth about getting close to the target*/) {
            state == 2;
        }
        if (state == 2) {
            set_servoPW(0-(distance_error*ksteer));
        }
        Sim_Update(); // MUST BE CALLED IN ALL LOOPS!!! (used to update the simulation and this code)
    }
}

void Port_Init(void){
    P0MDOUT |= 0x30;
    P1MDOUT |= 0x0D;        // Make P1.0,.2,.3 outputs
    P3MDOUT &= 0xFE;
    P2MDOUT &= 0xFD;
    P3 |= ~0xFE;
    P2 |= ~0xFD;
}

void XBR_Init(void){
    XBR0 = 0x27;    // 00100111, Enable SPI, I2C, and CEX0-3
}

void PCA_Init(void){

    PCA0MD = 0x81;
    PCA0CPM0 = 0xC2;
    PCA0CPM3 = 0xC2;
    PCA0CPM2 = 0xC2;
    PCA0CN |= 0x40;
}

void Interrupt_Init(void){
    EIE1 |= 0x08;       // Enable PCA interrupt
    EA = 1;             // Globally Enable interrupts
}

void SMB_Init(void){
    SMB0CR = 0x93;      // Configure the I2C Clock Rate
    ENSMB=1;            // Enable the module
}

void read_compass(void){
    unsigned char addr = 0xC0;          // the address of the sensor, 0xC0 for the compass
    unsigned char Data[2];              // Data is an array with a length of 2
    i2c_read_data(addr, 2, Data, 2);                        // read two byte, starting at reg 2
    heading =(((unsigned int)Data[0] << 8) | Data[1]);      // combine the two values
                            //heading has units of 1/10 of a degree
}

void read_ranger(void){
    unsigned char addr = 0xE0;          // the address of the ranger is 0xE0
    unsigned char Data[2];              // Data is an array with length of 2
    i2c_read_data(addr, 2, Data, 2);   //read two bytes, starting at reg 2
    range = (((unsigned int)Data[0] << 8) | Data[1]);       // combine the two values
    // Tell the ranger to trigger a ping
    Data[0] = 0x51;                     // want to write 0x51 to reg 0 (ping in cm)
    i2c_write_data(0xE0, 0, Data, 1);    // write data byte to ranger
}

void ReadRanger(uint8_t address){
    i2c_read_data(address, 2, Data, 2);   // Read the 0-3600 heading bytes
    distance = (((unsigned int)Data[0] << 8) | Data[1]);
    Data[0] = 0x51;
    i2c_write_data(address, 0, Data, 1);
    distance = distance;
}


void set_servoPW(signed int user_input){
    // Suggest adding slide switch control here
    SERVO_PW = SERVO_CENTER + user_input;

    if(SERVO_PW > SERVO_RIGHT){
        SERVO_PW = SERVO_RIGHT;
    }else if(SERVO_PW < SERVO_LEFT){
        SERVO_PW = SERVO_LEFT;
    }
    PCA0CP0 = 0xFFFF - SERVO_PW;
}

void set_motorPW(signed int user_input){
    // Suggest adding slide switch control here
    MOTOR_PW = MOTOR_NEUT + user_input;

    if(MOTOR_PW > MOTOR_MAX){
        MOTOR_PW = MOTOR_MAX;
    }else if(MOTOR_PW < MOTOR_MIN){
        MOTOR_PW = MOTOR_MIN;
    }
    PCA0CP2 = 0xFFFF - MOTOR_PW;
}

void PCA_ISR(void){
    if(CF){
        CF = 0;
        h_count++;
        r_count++;
        p_count++;
        if(h_count == 4){  // Count 40 ms
            h_count = 0;
            new_heading = 1;
        }
        if(r_count == 8){  // Count 80 ms
            r_count = 0;
            new_range = 1;
        }
        if(p_count == 47){  // Count 1 second
            p_count = 0;

            new_print = 1;
        }
        PCA0 = 28672;    // Configure the period of the PCA
        counts ++;
    }
    PCA0CN = 0x40;
}
