/*
 * File:   RotaryEncoder.c
 * Author: Isaac Paredes Hernandez
 *
 * Created on February 8, 2021, 5:34 PM
 */


#include "xc.h"
#include "RotaryEncoder.h"
#include "Protocol.h"
#include "MessageIDs.h"
#include "stdio.h"
#include "stdlib.h"
#include "BOARD.h"
#include <sys/attribs.h>

int parityBit(int in);

/**
 * @Function RotaryEncoder_Init(char interfaceMode)
 * @param interfaceMode, one of the two #defines determining the interface
 * @return SUCCESS or ERROR
 * @brief initializes hardware in appropriate mode along with the needed interrupts */
int RotaryEncoder_Init(char interfaceMode){
    if(interfaceMode == ENCODER_BLOCKING_MODE){
        SPI2CON = 0; //Clear everything
        //SPI2CON2 = 0;
        SPI2CONbits.MSTEN = 1; 
        SPI2CONbits.MODE32 = 0;
        SPI2CONbits.MODE16 = 1; //Set 16 bit mode
        SPI2CONbits.SMP = 1;
        SPI2CONbits.CKP = 1;
        SPI2CONbits.CKE = 1;
        SPI2BRG = CLOCK_5MHZ; //Set clock rate
        SPI2CONbits.ON = 1; //Turn on SPI
        TRISDbits.TRISD3 = 0; //SS or CS pin 9. 10 didn't work for some odd reason
        LATDbits.LATD3 = 1; //CS set high
    }else{
        
        SPI2CON = 0; //Clear everything
        //SPI2CON2 = 0;
        SPI2CONbits.MSTEN = 1; 
        SPI2CONbits.MODE32 = 0;
        SPI2CONbits.MODE16 = 1; //Set 16 bit mode
        SPI2CONbits.SMP = 1;
        SPI2CONbits.CKP = 1;
        SPI2CONbits.CKE = 1;
        SPI2BRG = CLOCK_5MHZ; //Set clock rate
        SPI2CONbits.ON = 1; //Turn on SPI
        TRISDbits.TRISD3 = 0; //SS or CS pin 9. 10 didn't work for some odd reason
        LATDbits.LATD3 = 1; //CS set high
        LATDbits.LATD3 = 0;
        
        
        IFS1bits.SPI2RXIF = 0;
        IPC7bits.SPI2IP = 4;
        IPC7bits.SPI2IS = 0;
        IEC1bits.SPI2RXIE = 1;
        
        T2CON = 0; //clear everything
        T2CONbits.TCKPS = 0b110; //Set prescaler 1:63
        TMR2 = 0x0; //Make sure timer starts at 0
        PR2 = BOARD_GetPBClock() / 64 / 1000; //Set period for 1ms interrupt
        T2CONbits.ON = 1; //Turn on the timer
    
        IFS0bits.T2IF = 0; //Clear flag
        IPC2bits.T2IP = 5; //Set priority
        IPC2bits.T2IS = 0; 
        IEC0bits.T2IE = 1; //Enable interrupt
        
        flag = READ_1ST;
        
        SPI2BUF = angle_command; 
    }
}



/**
 * @Function int RotaryEncoder_ReadRawAngle(void)
 * @param None
 * @return 14-bit number representing the raw encoder angle (0-16384) */
unsigned short RotaryEncoder_ReadRawAngle(void){
    
//    int i;
//    unsigned short packet;
//    unsigned short data;
//    LATDbits.LATD3 = 0; //set CS low
//    data = SPI2BUF; //clear buffer

//    for (i = 0; i < DELAY_350; i++){ //delay
//        asm("nop");
//    }
//    packet = ANGLECOM; //Read from angle address
//    SPI2BUF = packet; //Place packet in buffer
//    while(SPI2STATbits.SPIRBF == 0); //Wait until buffer is full
//    //LEDS_SET(0xF0);
//    data = SPI2BUF; //clear buffer
//    LATDbits.LATD3 = 1; //Set CS high
//    for (i = 0; i < DELAY_350; i++){
//        asm("nop");
//    }
//    LATDbits.LATD3 = 0; //Set low
//    for (i = 0; i < DELAY_350; i++){
//        asm("nop");
//    }
//    packet = NOP_COMMAND; //Send NOP command
//    SPI2BUF = packet; //Place in buffer
//    while(SPI2STATbits.SPIRBF == 0); //Wait until full
//    data = SPI2BUF; //read data
//    LATDbits.LATD3 = 1; //set cs high
//    return data & MASK_DATA; //return 14 bits, mask off bit 14 and 15.
      return info_read;
}

int partityBit(int in){
    int p = 0, i;
    for(i = 0; i < PARITY; i++){
        if(in & i){
            p = p + 1;
        }
        i = i<<1;
    }
    p = p%2;
    return p;
}

void __ISR(_TIMER_2_VECTOR) Timer2IntHandler(void){
    IFS0bits.T2IF = 0; //Clear flag
    LATDbits.LATD3 =  0;
    SPI2BUF = angle_command;
}
void __ISR(_SPI_2_VECTOR) __SPI2Interrupt(void) {
    IFS1bits.SPI2RXIF = 0; //Clear flag
    if(flag == READ_1ST){
        junk = SPI2BUF;
        flag = RECEIVE;
    }
    info_read = SPI2BUF & MASK_DATA; //Get 14-bit read
    LATDbits.LATD3 = 1;
}



//int main(void){
//    BOARD_Init();
//    Protocol_Init();
//    LEDS_INIT();
//    RotaryEncoder_Init(ENCODER_INTERRUPT_MODE);
//    FreeRunningTimer_Init();
//    char test[MAXPAYLOADLENGTH];
//    int velocity;
//    unsigned short rawAngle;
//    sprintf(test, "Encoder test compiled at %s %s", __DATE__, __TIME__);
//    Protocol_SendDebugMessage(test);
//    while(1){
//        if(FreeRunningTimer_GetMilliSeconds() % 2 == 0){
//            velocity = RotaryEncoder_ThresholdCheck(FreeRunningTimer_GetMilliSeconds());
//        }
//        if (FreeRunningTimer_GetMilliSeconds() % 10 == 0){
//            velocity = Protocol_IntEndednessConversion(velocity);
//            Protocol_SendMessage(4, ID_REPORT_RATE, &velocity);
            //sprintf(test, "W: %X, time:%X", velocity, FreeRunningTimer_GetMilliSeconds());
            //Protocol_SendDebugMessage(test);
            //rawAngle = info_read;
            //rawAngle = RotaryEncoder_ReadRawAngle();
            //rawAngle = Protocol_ShortEndednessConversion(rawAngle);
            //Protocol_SendMessage(2, ID_ROTARY_ANGLE, &rawAngle);
            //sprintf(test, "ANLGE: %X", rawAngle);
            //Protocol_SendDebugMessage(test);
//        }
//    }
//    while(1);
//}
