

#ifndef ROTARYENCODER_H
#define	ROTARYENCODER_H

/*******************************************************************************
 * PUBLIC #DEFINES                                                            *
 ******************************************************************************/

#define ENCODER_BLOCKING_MODE 0
#define ENCODER_INTERRUPT_MODE 1
#define DELAY_350 6000
#define CLOCK_5MHZ 0xF00D
#define ANGLECOM 0xFFFF
#define NOP_COMMAND 0xC000
#define ENCODER_ROLLOVER (1 << 13)
#define TICK_RATE 2
#define MAX_RATE 6552
#define PARITY 0x8000
#define MASK_DATA 0x3FFF

//#define 
enum state{
    CSLOW,
    SEND_READ,
    READ_1ST,
    LOW_AGAIN,
    SEND_NOP,
    RECEIVE
};
static unsigned short info_read, junk, flag, angle_command = ANGLECOM;





/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * @Function RotaryEncoder_Init(char interfaceMode)
 * @param interfaceMode, one of the two #defines determining the interface
 * @return SUCCESS or ERROR
 * @brief initializes hardware in appropriate mode along with the needed interrupts */
int RotaryEncoder_Init(char interfaceMode);

/**
 * @Function int RotaryEncoder_ReadRawAngle(void)
 * @param None
 * @return 14-bit number representing the raw encoder angle (0-16384) */
unsigned short RotaryEncoder_ReadRawAngle(void);



#endif	/* ROTARYENCODER_H */

