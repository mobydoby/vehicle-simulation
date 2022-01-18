#ifndef C8051_SIM_H_
#define C8051_SIM_H_

#if defined( _WIN32)
#include<windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <conio.h> // for getchar redefine
#define __SOCKTYPE      SOCKET
#define __SOCKLENTYPE   int
#define SENDBUFFTYPE    const char *
#define RECVBUFFTYPE    char *
#define MSG_CONFIRM     0
#define MSG_WAITALL     0
#define WAIT1MS()       Sleep(1)
#define GETERR()        WSAGetLastError()


#elif (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>  // Only to get gloabl errono
#include <time.h>   // For nanosleep()
#include <sys/select.h> // For getchar redefine
#include <termios.h> // for getchar redefine
#define __SOCKTYPE      int
#define __SOCKLENTYPE   socklen_t
#define SENDBUFFTYPE    const void *
#define RECVBUFFTYPE    void *
#define WAIT1MS()       nanosleep(&onems,NULL)
#define GETERR()        errno
// 1 ms sleep struct
struct timespec onems = {.tv_sec=0,.tv_nsec=1e6};

#include <stdio.h>


#else
#error "Unknown Operation System Type"
#endif



#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>

#define MAJOR_VERSION    1
#define MINOR_VERSION    4
#define SUBMINOR_VERSION IED

// pragma to prevent compiler from complaining about main() void return type.
#pragma GCC diagnostic ignored "-Wmain"
// Make ints 2-bytes instead of the default 4
//#define INT_TYPE_SIZE       16
//#define SHORT_TYPE_SIZE     16

// Contained functions
    // Called by program immediately after main
void Sys_Init();
    // Must be called in EVERY LOOP in main program
void Sim_Update();
    // i2c functions replacing i2c.h
void i2c_write_data(uint8_t addr,uint8_t start_reg,uint8_t *buffer,uint8_t num_bytes);
void i2c_read_data(uint8_t addr,uint8_t start_reg,uint8_t *buffer,uint8_t num_bytes);

    // Socket communication functions
void sim_wait_for_ack();
void sim_update_to_server();
void sim_update_from_server();
void sim_interrupt_handlers();

// Interrupt function prototypes here!
extern void PCA_ISR(void);

// Support for printf_fast_f
#define printf_fast_f   printf


/* Communication between client and server is done through several packets
 * Each packet starts with a label byte, indicating the struct/array stored.
 * The labels are as follows:
 */
#define LABEL_ZERO          0x00
#define LABEL_INT           0x01
#define LABEL_GPIO          0x02
#define LABEL_TIMERS        0x03
#define LABEL_ADC1          0x04
#define LABEL_PCA0          0x05
#define LABEL_I2C           0x06
#define LABEL_XBR           0x07
// ...
#define LABEL_I2C_SENSORS   0x10
// ...
#define LABEL_GPIO_P0       0x20
#define LABEL_GPIO_P1       0x21
#define LABEL_GPIO_P2       0x22
#define LABEL_GPIO_P3       0x23
// ...
#define LABEL_ERR           0xEE
#define LABEL_UPDATE_REQ    0xF1    // Asks simulator to send all
#define LABEL_UPDATE_DONE   0xF2    // Simulator tells us when its done
#define LABEL_INIT          0x5A    // First packet to be sent from client
#define LABEL_ACK           0xA5    // Acknowledge tx/rx
#define LABEL_RESET         0xFF

////////////////////////////////////////////////////////////////////////
////////////////// REGISTER STORAGE ////////////////////////////////////
////////////////////////////////////////////////////////////////////////
#pragma pack(1) // Set byte alignment (Windows compatibility)

/****GPIO****/
typedef union gpio_data_t {
    struct { // x86 is little-endian.  List lsb first
        uint8_t p0 : 1;
        uint8_t p1 : 1;
        uint8_t p2 : 1;
        uint8_t p3 : 1;
        uint8_t p4 : 1;
        uint8_t p5 : 1;
        uint8_t p6 : 1;
        uint8_t p7 : 1;
    };
    uint8_t value;
}gpio_data_t;

typedef struct gpio_n_regs_t {
    gpio_data_t data;
    uint8_t mdout;
    uint8_t mdin;   // Only used for P1
}gpio_n_regs_t;

typedef struct gpioregs_t {
    uint8_t label;
    gpio_n_regs_t p0;
    gpio_n_regs_t p1;
    gpio_n_regs_t p2;
    gpio_n_regs_t p3;
}gpioregs_t;
gpioregs_t gpioregs = {.label=LABEL_GPIO,.p0.mdout=0x01,.p1.mdin=0xFF};

typedef struct gpio_impstate_t {
    uint8_t p0;
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
}gpio_impstate_t;
gpio_impstate_t gpio_impstate;

#define P0          gpioregs.p0.data.value
#define P0MDOUT     gpioregs.p0.mdout
#define P1          gpioregs.p1.data.value
#define P1MDOUT     gpioregs.p1.mdout
#define P1MDIN      gpioregs.p1.mdin
#define P2          gpioregs.p2.data.value
#define P2MDOUT     gpioregs.p2.mdout
#define P3          gpioregs.p3.data.value
#define P3MDOUT     gpioregs.p3.mdout

#define P0_0        gpioregs.p0.data.p0
#define P0_1        gpioregs.p0.data.p1
#define P0_2        gpioregs.p0.data.p2
#define P0_3        gpioregs.p0.data.p3
#define P0_4        gpioregs.p0.data.p4
#define P0_5        gpioregs.p0.data.p5
#define P0_6        gpioregs.p0.data.p6
#define P0_7        gpioregs.p0.data.p7
#define P1_0        gpioregs.p1.data.p0
#define P1_1        gpioregs.p1.data.p1
#define P1_2        gpioregs.p1.data.p2
#define P1_3        gpioregs.p1.data.p3
#define P1_4        gpioregs.p1.data.p4
#define P1_5        gpioregs.p1.data.p5
#define P1_6        gpioregs.p1.data.p6
#define P1_7        gpioregs.p1.data.p7
#define P2_0        gpioregs.p2.data.p0
#define P2_1        gpioregs.p2.data.p1
#define P2_2        gpioregs.p2.data.p2
#define P2_3        gpioregs.p2.data.p3
#define P2_4        gpioregs.p2.data.p4
#define P2_5        gpioregs.p2.data.p5
#define P2_6        gpioregs.p2.data.p6
#define P2_7        gpioregs.p2.data.p7
#define P3_0        gpioregs.p3.data.p0
#define P3_1        gpioregs.p3.data.p1
#define P3_2        gpioregs.p3.data.p2
#define P3_3        gpioregs.p3.data.p3
#define P3_4        gpioregs.p3.data.p4
#define P3_5        gpioregs.p3.data.p5
#define P3_6        gpioregs.p3.data.p6
#define P3_7        gpioregs.p3.data.p7

/****ADC1****/

typedef struct adc1regs_t{
    uint8_t label;
    uint8_t REF0CN;
    uint8_t ADC1CF;
    uint8_t ADC1CN;
    uint8_t ADC1;
}adc1regs_t;
adc1regs_t adc1regs = {.label=LABEL_ADC1,.ADC1CF=0xF8};

#define REF0CN      adc1regs.REF0CN
#define ADC1CF      adc1regs.ADC1CF
#define ADC1CN      adc1regs.ADC1CN
#define ADC1        adc1regs.ADC1

/****PCA0****/

typedef union pca0cn_t {
    struct { // x86 is little-endian.  List lsb first
        uint8_t CCF0 : 1;
        uint8_t CCF1 : 1;
        uint8_t CCF2 : 1;
        uint8_t CCF3 : 1;
        uint8_t CCF4 : 1;
        uint8_t bit5 : 1;
        uint8_t CR : 1;
        uint8_t CF : 1;
    };
    uint8_t value;
}pca0cn_t;

typedef struct pca0regs_t {
    uint8_t label;
    pca0cn_t PCA0CNu;
    uint8_t PCA0MD;
    uint16_t PCA0;
    uint8_t PCA0CPM[5];
    uint16_t PCA0CP[5];
}pca0regs_t;
pca0regs_t pca0regs = {.label=LABEL_PCA0};

#define PCA0CN      pca0regs.PCA0CNu.value
#define PCA0MD      pca0regs.PCA0MD
#define PCA0        pca0regs.PCA0
#define PCA0CPM0    pca0regs.PCA0CPM[0]
#define PCA0CPM1    pca0regs.PCA0CPM[1]
#define PCA0CPM2    pca0regs.PCA0CPM[2]
#define PCA0CPM3    pca0regs.PCA0CPM[3]
#define PCA0CPM4    pca0regs.PCA0CPM[4]
#define PCA0CP0     pca0regs.PCA0CP[0]
#define PCA0CP1     pca0regs.PCA0CP[1]
#define PCA0CP2     pca0regs.PCA0CP[2]
#define PCA0CP3     pca0regs.PCA0CP[3]
#define PCA0CP4     pca0regs.PCA0CP[4]

#define CF          pca0regs.PCA0CNu.CF
#define CR          pca0regs.PCA0CNu.CR
#define CCF4        pca0regs.PCA0CNu.CCF4
#define CCF3        pca0regs.PCA0CNu.CCF3
#define CCF2        pca0regs.PCA0CNu.CCF2
#define CCF1        pca0regs.PCA0CNu.CCF1
#define CCF0        pca0regs.PCA0CNu.CCF0

/****SMB0****/

typedef struct smb0regs_t {
    uint8_t label;
    uint8_t SMB0CR;
    uint8_t ENSMB;  // Not actually a register, but we don't teach SMB0CN
}smb0regs_t;
smb0regs_t smb0regs  = {.label=LABEL_I2C};

#define SMB0CR      smb0regs.SMB0CR
#define ENSMB       smb0regs.ENSMB

/****XBR****/

typedef struct xbrregs_t {
    uint8_t label;
    uint8_t XBR0;
    uint8_t XBR1;
    uint8_t XBR2;
}xbrregs_t;
xbrregs_t xbrregs  = {.label=LABEL_XBR,.XBR0=0x04,.XBR2=0x40};

#define XBR0        xbrregs.XBR0
#define XBR1        xbrregs.XBR1
#define XBR2        xbrregs.XBR2

/****INTERRUPTS****/

typedef union ie_t {
    struct { // x86 is little-endian.  List lsb first
        uint8_t EX0 : 1;
        uint8_t ET0 : 1;
        uint8_t EX1 : 1;
        uint8_t ET1 : 1;
        uint8_t ES0 : 1;
        uint8_t ET2 : 1;
        uint8_t IEGF0 : 1;
        uint8_t EA : 1;
    };
    uint8_t value;
}ie_t;

typedef struct intregs_t {
    uint8_t label;
    ie_t IEu;
    uint8_t EIE1;
}intregs_t;
intregs_t intregs  = {.label=LABEL_INT};

#define IE          intregs.IEu.value
#define EIE1        intregs.EIE1

#define EA          intregs.IEu.EA
#define ET2         intregs.IEu.ET2
#define ES0         intregs.IEu.ES0
#define ET1         intregs.IEu.ET1
#define EX1         intregs.IEu.EX1
#define ET0         intregs.IEu.ET0
#define EX0         intregs.IEu.EX0

#pragma pack()


/////////////////////
///// SENSORS ///////
/////////////////////
    // Memory allocation is complete overkill...whatever!
#define N_SENSORS           6
#define SENSOR_REG_LENGTH  50
uint8_t sensor_addrs[] = {0xE0,0xE2,0xE4,0xE6,0xC0,0xC2}; // Ranger/Compass/Accel/Actuator
uint8_t sensor_regs_read[N_SENSORS][SENSOR_REG_LENGTH] = {0};
// sensor_regs_read index of sensor (should match sensor_addrs)

/////////////////////
//// SOCKET CFG /////
/////////////////////
// Simulation socket
#define SOCK_PORT   23500
__SOCKTYPE _sock = 0;
struct sockaddr_in serv_addr;
__SOCKLENTYPE addrlen = sizeof(struct sockaddr_in);

// Simulation transfer buffer
uint8_t sim_buffer[1024];
uint8_t *sim_buffer_payload;
uint8_t ack_buffer = LABEL_ACK;


void Sys_Init(){

    // Print out the version (easier to compare to simulator)
    printf("***** C8051_SIM.h version: %u.%u.IED *****\r\n\n",MAJOR_VERSION,MINOR_VERSION);

    // Set up printing to file, if requested
#ifdef PRINTTOFILE
    printf("Warning: Printing to file %s\r\n",PRINTTOFILE);
    FILE *fp = freopen(PRINTTOFILE,"w",stdout);
    if(fp == NULL){
        printf("ERROR: Opening %s failed.  Printing to terminal...\r\n\r\n",PRINTTOFILE);
    }
#endif
    // Set printf to print immediately (no buffer)
    setvbuf(stdout, NULL, _IONBF, 0);


    #ifdef _WIN32
    WSADATA wsaData;
    int iResult;
    iResult = WSAStartup(MAKEWORD(2,2),&wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed: %d\rn",iResult);
        while(1);
    }
    #endif
    // Set up socket to communicate with simulation
    if ((_sock = socket(AF_INET,SOCK_DGRAM, 0)) < 0){
        printf("\n Problem creating socket, try rebooting, then contact instructor\r\n");
        while(1);
    }
    // Set socket timeout
    #ifdef __WIN32
    DWORD timeout = 100;
    if (setsockopt(_sock,SOL_SOCKET,SO_RCVTIMEO,(const char *)&timeout,sizeof(DWORD)) < 0){
    #else
    struct timeval timeout = {.tv_sec=0,.tv_usec=100000};
    if (setsockopt(_sock,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(struct timeval)) < 0){
    #endif
        printf("\n Problem setting receive timeout. Try again?\r\n");
        while(1);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SOCK_PORT);
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Set up communication buffers
    sim_buffer_payload = sim_buffer+1;

    // Connect to simulation and tell to reset
    sim_buffer[0] = LABEL_INIT;
    sendto(_sock,(SENDBUFFTYPE)sim_buffer,1,MSG_CONFIRM,
               (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
    sim_wait_for_ack();
    sim_buffer[0] = LABEL_RESET;

#ifndef RIN // Check if RIN was provided
#warning "RIN is not provided at top of code. If Lab >= 4, Simulator will break, otherwise this warning can be ignored."
    sendto(_sock,(SENDBUFFTYPE)sim_buffer,1,MSG_CONFIRM,
               (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
#else
    unsigned long rin = RIN;
    sim_buffer[1] = (rin>>24)^(rin>>16)^(rin>>8)^(rin);
    sendto(_sock,(SENDBUFFTYPE)sim_buffer,2,MSG_CONFIRM,
                   (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
#endif
    sim_wait_for_ack();

    // Wait a little bit for it to reset
    uint32_t delay_count = 1e6;
    while(delay_count--);

    // Perform first update
    Sim_Update();
}

void Sim_Update(){
    sim_update_from_server(); // Do this first to ensure that interrupt flags aren't overwritten
    sim_interrupt_handlers(); // Handle interrupts as necessary
    sim_update_to_server();   // Update simulation
    WAIT1MS();  // Sleep for a millisecond
}

int16_t sim_recvfrom(){
    sim_buffer[0] = LABEL_ERR; // Prefill the buffer with an error in case it doesn't get overwritten
    int16_t reclen = -1;
    uint8_t err_printed = 0;
    while(reclen == -1){
        reclen = recvfrom(_sock,(RECVBUFFTYPE)sim_buffer,1024,MSG_WAITALL,
             (struct sockaddr *) &serv_addr,&addrlen);
        if(reclen == -1){   // Error returned.  Report it and stop program
            int err = GETERR();
            if(err == 10060){
                if(!err_printed){
                    printf("\r\n\tSIMULATION WARNING: Simulation Response Timeout.  Continuing to wait...\r\n\n");
                    err_printed = 1;
                }
            }else{
                printf("\tSocket Error: %u\tRESTART SIMULATION THEN RUN AGAIN\r\n",err);
                while(1);
            }
        }
    }
    return reclen;
}

void sim_wait_for_ack(){
    sim_recvfrom();
    if(sim_buffer[0] != LABEL_ACK){
        printf("SIM WARNING: Expected acknowledgment from server, got something else [%02X]\r\n",sim_buffer[0]);
    }
}

void sim_update_to_server(){
    // Only need to do register stuff here.  I2C handled in i2c_write_data()

    // Send PCA0 Stuff
    sendto(_sock,(SENDBUFFTYPE)&pca0regs,sizeof(pca0regs),MSG_CONFIRM,
           (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
    sim_wait_for_ack();

    // Send GPIO
    sendto(_sock,(SENDBUFFTYPE)&gpioregs,sizeof(gpioregs),MSG_CONFIRM,
            (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
    sim_wait_for_ack();

    // Send Crossbar
    sendto(_sock,(SENDBUFFTYPE)&xbrregs,sizeof(xbrregs),MSG_CONFIRM,
               (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
    sim_wait_for_ack();
}

void sim_update_from_server(){
    // Save the GPIO impedance state (this won't be super accurate, but will be good enough for small programs)
    // If the pin is ever set to 1, assume high-impedance for the rest of the program
    gpio_impstate.p0 |= ~P0MDOUT & P0;
    gpio_impstate.p1 |= ~P1MDOUT & P1;
    gpio_impstate.p2 |= ~P2MDOUT & P2;
    gpio_impstate.p3 |= ~P3MDOUT & P3;
    // Ask for an update
    sim_buffer[0] = LABEL_UPDATE_REQ;
    sendto(_sock,(SENDBUFFTYPE)sim_buffer,1,MSG_CONFIRM,
           (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
    sim_wait_for_ack();
    uint8_t _label = LABEL_ZERO;
    uint8_t n_rec_sensors;
    // Take in packets from the simulation until it says it's done.
    while(_label != LABEL_UPDATE_DONE){
        sim_buffer[0] = LABEL_ERR; // Prefill the buffer with an error in case it doesn't get overwritten
        int16_t reclen = sim_recvfrom();
        _label = sim_buffer[0];
        if(_label == LABEL_ACK){ // Received an ACK for some reason.  Ignore it
            continue;
        }
        sendto(_sock,(SENDBUFFTYPE)&ack_buffer,1,MSG_CONFIRM,
             (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
        // Update registers as needed //
        /*Only update hardware controlled bits*/
        switch(_label){
            case LABEL_GPIO:
                P0 &= P0MDOUT;                                      // Clear inputs
                P0 |= sim_buffer[1] & ~P0MDOUT & gpio_impstate.p0;  // Then set the active ones high (if high imp state)
                P1 &= P1MDOUT;                                      // Clear inputs
                P1 |= sim_buffer[2] & ~P1MDOUT & gpio_impstate.p1;  // Then set the active ones high
                P2 &= P2MDOUT;                                      // Clear inputs
                P2 |= sim_buffer[3] & ~P2MDOUT & gpio_impstate.p2;  // Then set the active ones high
                P3 &= P3MDOUT;                                      // Clear inputs
                P3 |= sim_buffer[4] & ~P3MDOUT & gpio_impstate.p3;  // Then set the active ones high
                break;
            case LABEL_GPIO_P0:
                break;
            case LABEL_GPIO_P1:
                break;
            case LABEL_GPIO_P2:
                break;
            case LABEL_GPIO_P3:
                break;
            case LABEL_TIMERS:
                break;
            case LABEL_ADC1:
                ADC1CN |= sim_buffer[1] & 0x20;     // Set ADC1INT if applicable
                ADC1CN &= sim_buffer[1] & ~0x10;    // Clear ADC1BUSY if applicable
                ADC1 = sim_buffer[2];
                break;
            case LABEL_PCA0:
                PCA0CN |= sim_buffer[1] & 0x9F; // Update CF, CCF4-0
                break;
            case LABEL_I2C_SENSORS:
                // Figure out what sensor was sent and then copy data over.
                n_rec_sensors = (reclen-1)/(SENSOR_REG_LENGTH+1);
                uint8_t _i;
                uint8_t *i2c_buffer_payload = sim_buffer_payload;
                while(n_rec_sensors--){
                    for(_i=0;_i<N_SENSORS;_i++){
                        if(i2c_buffer_payload[0] == sensor_addrs[_i]){
                            i2c_buffer_payload++;
                            memcpy(sensor_regs_read[_i],i2c_buffer_payload,SENSOR_REG_LENGTH);
                            i2c_buffer_payload += SENSOR_REG_LENGTH;
                            break;
                        }
                    }
                }
                break;
            case LABEL_UPDATE_DONE:
                break;
            default:
                printf("\r\nWARNING: Unsupported packet label received (0x%02X). " \
                       "\r\n\tCheck your C8051_SIM.h version versus the simulator" \
                       "\r\n\tIf compatible, post on Piazza.\r\n\r\n",_label);
                break;
        }
    }
}

// Handle any interrupts that should happen.  Only PCA for now
void sim_interrupt_handlers(){
    while(EA && (EIE1 & 0x08) && (PCA0CN & 0x80) && (PCA0MD & 0x01)){
        PCA_ISR();
    }
}

// Write data to the "sensors".  This is done immediately (e.g., data sent over socket)
void i2c_write_data(uint8_t addr,uint8_t start_reg,uint8_t *buffer,uint8_t num_bytes){
    if(!(ENSMB==1 && SMB0CR==0x93))
        return;
    sim_buffer[0] = LABEL_I2C_SENSORS;
    sim_buffer[1] = addr & ~0x01;
    sim_buffer[2] = start_reg;
    uint8_t *i2c_buffer = sim_buffer+3;
    while(num_bytes--)
        *i2c_buffer++ = *buffer++;
    sendto(_sock,(SENDBUFFTYPE)sim_buffer,i2c_buffer-sim_buffer,MSG_CONFIRM,
           (const struct sockaddr *) &serv_addr,sizeof(serv_addr));
    sim_wait_for_ack();
}

// Read data from the sensors.  The actual register download of the sensors is done
// During "Sim_Update()".  Here, we just report back the contents of each register.
void i2c_read_data(uint8_t addr,uint8_t start_reg,uint8_t *buffer,uint8_t num_bytes){
    uint8_t _i,_j;
    if(!(ENSMB==1 && SMB0CR==0x93)){
        for(_i=0;_i<num_bytes;_i++){
            buffer[_i] = 0xFF;
        }
        return;
    }
    for(_i=0;_i<sizeof(sensor_addrs);_i++){
        if(sensor_addrs[_i] == addr){
            for(_j=0;_j<num_bytes;_j++){
                if(start_reg<50){
                    buffer[_j] = sensor_regs_read[_i][start_reg++];
                }else{
                    buffer[_j] = 0xFF;
                }
            }
            return;
        }
    }
    // Sensor not found, fill with 0xFF
    for(_j=0;_j<num_bytes;_j++){
        buffer[_j] = 0xFF;
    }
}

//define new getchar method
#ifdef _WIN32
 
 DWORD WINAPI ThreadFunc(void * data) {
   *((char*) data) = getch();
   return 0;
 }

 // runs getchar on another thread while it calls Sim_Update()
 int getchar_with_Sim_Update(void) {
   // make the return variable that will be set by the thread
   char * ret = (char *) malloc(sizeof(char));
   // make the thread and make it execute ThreadFunc
   HANDLE thread = CreateThread(NULL, 0, ThreadFunc, ret, 0, NULL);

   // every 50ms call Sim_Update until thread is finished
   while (WaitForSingleObject(thread, 50) == WAIT_TIMEOUT) {
     Sim_Update();
   }
   char r = *ret;
   free(ret);
   return r;
 }

#else
 void turn_off_echo_and_buffer() {
   struct termios old = {0};
   tcgetattr(0, &old);
   old.c_lflag &= ~ICANON;
   old.c_lflag &= ~ECHO;
   tcsetattr(0, TCSANOW, &old);
 }

 void turn_on_echo_and_buffer() {
   struct termios old = {0};
   tcgetattr(0, &old);
   old.c_lflag |= ICANON;
   old.c_lflag |= ECHO;
   tcsetattr(0, TCSANOW, &old);
 }

 // times out every 50 ms and calls Sim_Update()
 int getchar_with_Sim_Update(void) {
   fd_set set;
   struct timeval timeout;
   int filedesc = 0; // stdin

   timeout.tv_sec = 0;
   timeout.tv_usec = 50000;

   turn_off_echo_and_buffer();
   // wait for data to be available
   do {
     Sim_Update();

     // need to reinitialize the set each loop
     FD_ZERO(&set); /* clear the set */
     FD_SET(filedesc, &set); /* add our file descriptor to the set */
   }while (0 == select(filedesc + 1, &set, NULL, NULL, &timeout));
   turn_on_echo_and_buffer();
   
   // read data
   char buff[1];
   int len = 1;
#pragma GCC diagnostic ignored "-Wunused-result"
   read( filedesc, buff, len );
#pragma GCC diagnostic warning "-Wunused-result"
   return buff[0];
 }
#endif

// intercept all getchar calls with our new get char
#define getchar getchar_with_Sim_Update

#endif
