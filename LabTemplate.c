/* ENGR-2050 SUMMER 2020 - MiniProject
 * **** Template Code FOR ENGR-2350 LAB 4 ****
 * This file must be modified for IED Mini Project
 * */

//#define PRINTTOFILE     "FILENAME.csv"
#define RIN             662008784

#include "C8051_SIM.h"
#include<stdio.h>
#include<stdlib.h>

void Port_Init(void);                // Initialize all the GPIO ports (I/O only)
void PCA_Init(void);                 // Initialize PCA function
void XBR_Init(void);                 // Initialize Crossbar function
void Interrupt_Init(void);           //Initialize interrupts function
void SMB_Init(void);                 // Initialize I2C function
void read_ranger(uint8_t direction); // Reads the ranger specified by the direction parameter
void update_rangers(void);           // Updates all distance variables to current values using read_ranger calls
void read_compass(void);             // Function to read the compass heading
void set_servoPW(signed int user_input); //Sets the servo PW according to desired direction
void set_motorPW(signed int user_input); //Sets the motor PW to desired pulsewidth value
void update_heading(void);           // Updates heading variable to current compass measurement
void print_info(void);               // Prints a selection of important info

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

unsigned int time = 0;
uint8_t Data[2];

signed int heading = 0;             // Variable to hold compass heading value
signed int des_heading = 0;         // Variable to hold desired heading in deg/10
signed int heading_error = 0;       // heading_error = des_heading - heading [set in switch statement]

uint16_t distance = 0;              // Temporary variable to hold most recent value of ANY ranger
uint16_t distancef = 0;             // Variable to hold front ranger distance value
uint16_t distancel = 0;             // Variable to hold left ranger distance value
uint16_t distancer = 0;             // Variable to hold right ranger distance value

uint8_t front = 0xE0;               // Register location of the front ranger data
uint8_t left = 0xE4;                // Register location of the left ranger data
uint8_t right = 0xE6;               // Register location of the right ranger data
uint8_t head_addr = 0xC0;           // Register location of the compass data

uint8_t state = 1;                  // Operative variable for the main case statement, initial state is 1

uint16_t des_distance = 50;         // Constant governing how close the car stays to the beacon while circling
signed int distance_error = 0;      // distance_error = des_distance - distancer [set during read_ranger]

float korbit = 6;                   // Proportional Gain constant used while orbiting the beacon
float kturn = 0.3;                  // Proportional Gain constant used while turning towards/away from the beacon


//__sbit __at 0xB0 SS;        // Sbit on Port 3 pin 0
#define SS P3_0               // Simulator Sbit on Port 3 pin 0
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

    // Read from all rangers once REGARDLESS OF new_range flag to trigger a start ping.
    // Because a sonar ping takes time to bounce off an object and return to the sensor, a start ping is necessary before the while loop.
    read_ranger(front); // Use read_ranger() calls rather than update_rangers() to trigger the start ping regardless of new_range flag.
    distancef = distance;
    read_ranger(left);
    distancel = distance;
    read_ranger(right);
    distancer = distance;

    // Initialize the drive motor to neutral for 1 s. This is a requirement of the simulator.
    MOTOR_PW = 0xFFFF-MOTOR_NEUT; // Set motor pulsewidth to neutral
    PCA0CP2 = MOTOR_PW;
    SERVO_PW = 0xFFFF-SERVO_CENTER;
    PCA0CP0 = SERVO_PW;
    printf("Initializing motors, please wait.\n"); // Print status of simulation
    while(counts < 200){ // Until one second has elapsed...
        Sim_Update();    // Refresh the simulation (necessary in all loops) and do not do anything else.
    }

    // Clear the flag counter variables to prevent a double read
    r_count = h_count = p_count = 0;
    new_range = new_heading = new_print = 0;

    // Make sure pushbutton is not pressed before starting
    while (!PB) {Sim_Update();}
    printf("\nPush Button Released... ");

    // Wait for slide switch to toggle ON to begin program.
    if (!SS) {printf("\nPlease toggle the slide switch on. ");}
    while (!SS) {Sim_Update();}

    printf("Beginning Simulation:\n");

    // Print data headers for console output.
    printf("Time\tHeading\tRangeF\tRangeL\tRangeR\tServoPW\tMotorPW\tState\n");
    // Run program loop
    while(1){

        // Update the ranger and heading measurements if their respective flags are up.
        update_rangers();
        update_heading();

        // Print measurement readouts if new_print flag is up.
        print_info();

        // Switch case statement governs how the robot moves - movement goal is divided into states.
        switch(state){
            case(1): // Initial state. The robot moves northeast towards desired heading 220deg.
                set_motorPW(MOTOR_MAX);
                des_heading = 2200;
                set_servoPW(heading_error*kturn); // Turn using experimentally derived PD constant for turns.
                if(distancer < 100) { // Advance state when car is within 100 ranger units of target (limit experimentally derived).
                    state++;
                }
                break;
            case(2): // Orbit in a circle around the target.
                set_servoPW(0-(distance_error*korbit)); // korbit experimentally derived to produce even circle
                if(!PB) { // Advance state when pushbutton is depressed.
                    state++;
                }
                break;
            case(3): // Continue orbiting until heading is sufficiently close to zero. This prevents collision with target.
                set_servoPW(0-(distance_error*korbit));
                if(heading < 100 || heading > 3500) { // Advance state when heading is within 1deg of 0/360 wrap.
                    state++;
                }
                break;
            case(4): // When the car is pointed towards home (0/360deg), drive southwest to desired heading 0deg.
                if (heading < 100) { // Two separate heading error calculations necessary due to 0/360deg wrap.
                    des_heading = 0;
                }
                else {
                    des_heading = 3600;
                }

                set_servoPW((heading_error*kturn));
                break;

        }

        Sim_Update(); // MUST BE CALLED IN ALL LOOPS!!! (used to update the simulation and this code)
    }
}

// When new_print flag is up, print out measurements from the vehicle sensors, PWs,  and state number.
void print_info(){
    if (new_print){
        new_print = 0; // Put flag back down since task is completed.
        time++;
        printf("%d\t%d\t%u\t%u\t%u\t%u\t%u\t%d\n", time, heading, distancef, distancel, distancer, PCA0CP0, PCA0CP2, state);
    }
}

// Reads heading data from the compass' output register on the I2C. Called in update_heading().
void read_compass(void){
    i2c_read_data(head_addr, 2, Data, 2);                   // Since the output size is 2 register lengths, read it to array Data of length 2
    heading =(((unsigned int)Data[0] << 8) | Data[1]);      // Combine the two values in Data[] to get the full heading measurement in deg/10
}

// When new_heading flag is up, call read_compass() to trigger heading measurement.
void update_heading(){
    if (new_heading){
        read_compass();
        heading_error = des_heading - heading; // Calculate heading error from desired heading and measuremnt taken
        new_heading=0; // Put flag back down since task is completed.
    }
}

// Reads the data from a single ranger's I2C output register. Called for each ranger in update_rangers().
void read_ranger(uint8_t address){
    i2c_read_data(address, 2, Data, 2);                     // Since the output size is 2 register lengths, read it to array Data of length 2
    distance = (((unsigned int)Data[0] << 8) | Data[1]);    // Combine the two values in Data[] to get the full range measurement
    Data[0] = 0x51;                                         // The next two lines trigger the next sonar ping
    i2c_write_data(address, 0, Data, 1);
}

// When new_range flag is up, call read_ranger() for each ranger to trigger distance measurements.
void update_rangers(){
    if (new_range){
        read_ranger(front);    // Read from the front ranger and store measurement in distancef
        distancef = distance;
        read_ranger(left);     // Read from the left ranger and store measurement in distancel
        distancel = distance;
        read_ranger(right);    // Read from the right ranger and store measurement in distancer
        distancer = distance;
        distance_error = des_distance - distancer; // Calculate error from desired distance using the distancer measurement
        new_range = 0;         // Put flag back down since task is completed.
    }
}

// Sets the turning servo pulsewidth based on input from navigation statements
void set_servoPW(signed int user_input){
    SERVO_PW = SERVO_CENTER + user_input;

    if(SERVO_PW > SERVO_RIGHT){      // If new servo PW is outside the right or left limits, fix to the appropriate limit
        SERVO_PW = SERVO_RIGHT;
    }
    else if(SERVO_PW < SERVO_LEFT){
        SERVO_PW = SERVO_LEFT;
    }
    PCA0CP0 = 0xFFFF - SERVO_PW;     // Set PCA0CP0 (PCA controlling servo) using calculated PW
}

// Sets the drive motor pulsewidth based on input from navigation statements
void set_motorPW(signed int user_input){
    MOTOR_PW = MOTOR_NEUT + user_input;

    if(MOTOR_PW > MOTOR_MAX){
        MOTOR_PW = MOTOR_MAX;       // If new motor PW is outside the min or max limits, fix to the appropriate limit
    }
    else if(MOTOR_PW < MOTOR_MIN){
        MOTOR_PW = MOTOR_MIN;
    }
    PCA0CP2 = 0xFFFF - MOTOR_PW;    // Set PCA0CP2 (PCA controlling motor) using calculated PW
}

// Port initializations for simulation I2C
void Port_Init(void){
    P0MDOUT |= 0x30;
    P1MDOUT |= 0x0D;
    P3MDOUT &= 0xFE;
    P2MDOUT &= 0xFD;
    P3 |= ~0xFE;
    P2 |= ~0xFD;
}

// Initialization for simulation crossbar
void XBR_Init(void){
    XBR0 = 0x27;    // 00100111, Enable SPI, I2C, and CEX0-3
}

// Initialization for simulation PCAs
void PCA_Init(void){

    PCA0MD = 0x81;
    PCA0CPM0 = 0xC2;
    PCA0CPM3 = 0xC2;
    PCA0CPM2 = 0xC2;
    PCA0CN |= 0x40;
}

// Initialization for simulation interrupts
void Interrupt_Init(void){
    EIE1 |= 0x08;       // Enable PCA interrupt
    EA = 1;             // Globally Enable interrupts
}

// Initialization for simulation SMBus incl. clockrate
void SMB_Init(void){
    SMB0CR = 0x93;      // Configure the I2C Clock Rate
    ENSMB=1;            // Enable the SMBus module
}

// PCA Interrupt Service Routine (ISR) - sets new_X flags based on clock count
void PCA_ISR(void){
    if(CF){             // If PCA period has passed (10ms) increment all flag trackers
        CF = 0;
        h_count++;
        r_count++;
        p_count++;

        if(h_count == 4){  // Raise new_heading flag every 4 PCA cycles (40 ms)
            h_count = 0;
            new_heading = 1;
        }
        if(r_count == 8){  // Raise new_range flag every 8 PCA cycles (80 ms)
            r_count = 0;
            new_range = 1;
        }
        if(p_count == 47){ // Raise new_print flag every 47 PCA cycles (10 ms)
            p_count = 0;
            new_print = 1;
        }
        PCA0 = 28672;    // Configure the period of the PCA
        counts ++;
    }
    PCA0CN = 0x40;
}

