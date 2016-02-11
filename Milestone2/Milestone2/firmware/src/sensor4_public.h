/* 
 * File:   sensor4_public.h
 * Author: Andrew Wang
 *
 * Created on February 11, 2016, 1:54 AM
 */

#ifndef SENSOR4_PUBLIC_H
#define	SENSOR4_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

    int sensor4SendTimerValToMsgQ(unsigned int millisecondsElapsed);
    
    void sensor4SendSensorValToSensorQ(unsigned char sensorValue);

    unsigned char sensor4ReceiveVal();

#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR4_PUBLIC_H */

