/*
 * File:   Lab5application.c
 * Author: Isaac Paredes Hernandez
 *
 * Created on March 12, 2021, 10:45 PM
 */

#include "BOARD.h"
#include "Protocol.h"
#include "DCMotorDrive.h"
#include "FeedbackControl.h"
#include "MessageIDs.h"
#include "ADCFilter.h"
#include "RotaryEncoder.h"
#include "NonVolatileMemory.h"

#include "stdio.h"
#include "stdlib.h"

#include <sys/attribs.h>
#include "xc.h"

#define MS_5 5
#define MS_1 1
#define MS_10 10
#define GAINS_SIZE 12
#define INT_LEN 4
#define SHORT_LEN 2
#define REPORT_LEN 16
#define GEAR_HEAD 84
#define DRIVE_MOTOR_FORWARD 800
#define DRIVE_MOTOR_BACKWARD -800
#define ADDRESS_COMMAND (1290 * 64)
#define ADDRESS_SENSOR (1135 * 64)
#define COMMAND_MODE 0
#define SENSOR_MODE 1
#define POT_CHANNEL 0

struct{
    int error;
    int reference;
    int sensor;
    int head_position;
}Report;

static short low_pass[FILTERLENGTH] = {-54, -64, -82, -97, -93, -47, 66, 266, 562, 951, 1412, 1909, 
2396, 2821, 3136, 3304, 3304, 3136, 2821, 2396, 1909, 1412, 951, 562, 266, 66, -47, -93, -97, -82, 
-64, -54};
static int Op = 0, nOa = 0, counts, Ot, Ot2, FeedData[3], FeedCur[3], DegOt, deg2Ticks;
static unsigned int ms_5 = 0, ms_1 = 0, ms_10 = 0, t_ms, delay = 0;
static unsigned short send_angle, Oc, resp = 0xFFFF;
unsigned char mode, set_mode = 0;
static short raw_adc, channel;
static int T_p = 0, T_c, theta_p = 0, theta_c, w;

int Rotary_AbsolutePosition(int current_angle);
int RotaryEncoder_ThersholdCheck(int t_current);

int main(void)
{
    //Initialize hardware and software
    BOARD_Init();
    Protocol_Init();
    LEDS_INIT();
    DCMotorDrive_Init();
    RotaryEncoder_Init(ENCODER_INTERRUPT_MODE);
    ADCFilter_Init();
    NonVolatileMemory_Init();
    FreeRunningTimer_Init();
    signed short filterReading, hello = 0;
    char test[MAXPAYLOADLENGTH];
    unsigned char junk;
    int dutyCycle, position, U, posDeg;
    sprintf(test, "%d**************%d", hello, hello); //Send to avoid error
    Protocol_SendDebugMessage(test);
    sprintf(test, "Lab 5 compiled at %s, %s", __DATE__, __TIME__); //ID Debug message
    Protocol_SendDebugMessage(test);
    NonVolatileMemory_ReadPage(ADDRESS_COMMAND, PAGE_SIZE, (unsigned char*)FeedData); //Fetch gains
    Protocol_SendMessage(SHORT_LEN, ID_LAB5_CUR_MODE, &mode); //Send default mode to interface
    FeedCur[0] = Protocol_IntEndednessConversion(FeedData[0]); //Convert gains to report
    FeedCur[1] = Protocol_IntEndednessConversion(FeedData[1]);
    FeedCur[2] = Protocol_IntEndednessConversion(FeedData[2]);
    Protocol_SendMessage(GAINS_SIZE, ID_FEEDBACK_CUR_GAINS, &FeedCur); //Report gains
    ADCFilter_SetWeights(POT_CHANNEL, low_pass); //Set weights to ADC library
    while(FreeRunningTimer_GetMilliSeconds() - delay < MS_10); //Delay 10 ms
    while(1){
        if(Protocol_IsMessageAvailable()){ //If message is available
            if(Protocol_ReadNextID() == ID_COMMAND_OPEN_MOTOR_SPEED){ //Used for debugging
               Protocol_GetPayload(&dutyCycle);
               dutyCycle = Protocol_IntEndednessConversion(dutyCycle);
               DCMotorDrive_SetMotorSpeed(dutyCycle);
            }
            if(Protocol_ReadNextID() == ID_LAB5_SET_MODE){ //Set mode command
                Protocol_GetPayload(&mode);
                set_mode = 1; 
                if (mode == COMMAND_MODE){//If command mode, load command PID gains
                    NonVolatileMemory_ReadPage(ADDRESS_COMMAND,PAGE_SIZE,(unsigned char *)FeedData);
                }else if(mode == SENSOR_MODE){//If sensor mode, load sensor gains
                    NonVolatileMemory_ReadPage(ADDRESS_SENSOR,PAGE_SIZE,(unsigned char *)FeedData);
                }
                FeedCur[0] = Protocol_IntEndednessConversion(FeedData[0]); //Convert to report
                FeedCur[1] = Protocol_IntEndednessConversion(FeedData[1]);
                FeedCur[2] = Protocol_IntEndednessConversion(FeedData[2]);
                Protocol_SendMessage(GAINS_SIZE, ID_FEEDBACK_CUR_GAINS, &FeedCur); //Report gains
            }
            if(Protocol_ReadNextID() == ID_COMMANDED_POSITION){//Obtain position 
                Protocol_GetPayload(&position);
                position = Protocol_IntEndednessConversion(position);
                posDeg = (position * 1916667)/1000 ; //Convert degrees to ticks
            }
            if(Protocol_ReadNextID() == ID_LAB5_REQ_MODE){
                Protocol_GetPayload(&junk); //Clear RX buffer
                Protocol_SendMessage(1, ID_LAB5_CUR_MODE, &mode); //Send current
            }
             if(Protocol_ReadNextID() == ID_FEEDBACK_SET_GAINS){
                Protocol_GetPayload(&FeedData); //Get gains
                FeedData[0] = Protocol_IntEndednessConversion(FeedData[0]); //Convert gains
                FeedData[1] = Protocol_IntEndednessConversion(FeedData[1]);
                FeedData[2] = Protocol_IntEndednessConversion(FeedData[2]);
                FeedbackControl_SetProportionalGain(FeedData[0]); //Set gains
                FeedbackControl_SetIntegralGain(FeedData[1]);
                FeedbackControl_SetDerivativeGain(FeedData[2]);
                Protocol_SendMessage(2, ID_FEEDBACK_SET_GAINS_RESP, &resp); //Respond to console
                FeedbackControl_ResetController(); //Reset controller
                if (mode == COMMAND_MODE){//Store in command address
                    NonVolatileMemory_WritePage(ADDRESS_COMMAND,PAGE_SIZE,(unsigned char *)FeedData);
                }else if(mode == SENSOR_MODE){//Store in sensor address
                    NonVolatileMemory_WritePage(ADDRESS_SENSOR,PAGE_SIZE,(unsigned char *)FeedData);
                } 
            }
            if(Protocol_ReadNextID() == ID_FEEDBACK_REQ_GAINS){//If asked to report gains
                Protocol_GetPayload(&junk);
                FeedCur[0] = Protocol_IntEndednessConversion(FeedData[0]);
                FeedCur[1] = Protocol_IntEndednessConversion(FeedData[1]);
                FeedCur[2] = Protocol_IntEndednessConversion(FeedData[2]);
                Protocol_SendMessage(GAINS_SIZE, ID_FEEDBACK_CUR_GAINS, &FeedCur); //Report gains
            }
        }
        if(FreeRunningTimer_GetMilliSeconds() - ms_1 >= MS_1){//Calculate current position every 1ms
            Oc = RotaryEncoder_ReadRawAngle();
            Ot = Rotary_AbsolutePosition((int)Oc);
            DegOt = (Ot * 1000) / 1916667; //Convert current position to degrees
            ms_1 = FreeRunningTimer_GetMilliSeconds();
        }
        if(FreeRunningTimer_GetMilliSeconds() - ms_10 >= MS_10){//Every 10ms run PID & POT readings
            filterReading = ADCFilter_FilteredReading(POT_CHANNEL); //Get raw reading
            if(filterReading >= 512){ //Set range from -150 to 150 degrees
                filterReading = ((((int)filterReading - 512) * 293543) / 1000000); 
            }else{
                filterReading = ((((int)filterReading - 512) * 293542)/ 1000000);
            }
            if(mode == COMMAND_MODE){ //PID for command mode
                U = FeedbackControl_Update(posDeg, Ot); 
                Report.error = position - DegOt;
                Report.head_position = DegOt;
                Report.reference = position;
            }
            if(mode == SENSOR_MODE){ //PID for sensor mode
                //LEDS_SET(0xFF);
                deg2Ticks = ((int)filterReading * 1916667)/1000;
                U = FeedbackControl_Update(deg2Ticks, Ot);
                Report.error = filterReading - DegOt;
                Report.head_position = DegOt;
                Report.reference = filterReading;
            }
            U = ((int64_t)U * MAXMOTORSPEED) >> FEEDBACK_MAXOUTPUT_POWER; //scale speed
            DCMotorDrive_SetMotorSpeed(U); //Set speed
            Report.sensor = Protocol_IntEndednessConversion((int)filterReading);
            Report.error = Protocol_IntEndednessConversion(Report.error);
            Report.head_position = Protocol_IntEndednessConversion(Report.head_position);
            Report.reference = Protocol_IntEndednessConversion(Report.reference);
            Protocol_SendMessage(REPORT_LEN, ID_LAB5_REPORT, &Report); //Report data back
        }
    }
    while(1);
}

Rotary_AbsolutePosition(int current_angle){//Algorithm 1 from lab 5
    counts = current_angle - Op;
    Op = current_angle;
    if (counts > MAX_RATE){
        counts = counts - ENCODER_ROLLOVER;
        nOa = nOa - ENCODER_ROLLOVER;
    }
    if (counts < -MAX_RATE){
        counts = counts + ENCODER_ROLLOVER;
        nOa = nOa + ENCODER_ROLLOVER;
    }
    return current_angle + nOa;
}

int RotaryEncoder_ThresholdCheck(int t_current){//Algorithm 1 from lab 4
    if (t_current - T_p >= TICK_RATE){
        T_p = t_current;
        theta_c = (int)RotaryEncoder_ReadRawAngle();
        w = theta_c - theta_p;
        theta_p = theta_c;
        if(w > MAX_RATE){
            w = w - ENCODER_ROLLOVER;
        }
        if(w < -MAX_RATE){
            w = w + ENCODER_ROLLOVER;
        }
        return w;
    }
}
