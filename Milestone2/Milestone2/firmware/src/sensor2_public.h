/* 
 * File:   sensor2_public.h
 * Author: Andrew Wang
 *
 * Created on February 11, 2016, 1:54 AM
 */

#ifndef SENSOR2_PUBLIC_H
#define	SENSOR2_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

    int sensor2SendTimerValToMsgQ(unsigned int millisecondsElapsed);
    
    void sensor2SendSensorValToSensorQ(unsigned char sensorValue);

    unsigned char sensor2ReceiveVal();
#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR2_PUBLIC_H */

