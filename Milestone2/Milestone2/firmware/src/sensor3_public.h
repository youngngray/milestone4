/* 
 * File:   sensor3_public.h
 * Author: Andrew Wang
 *
 * Created on February 11, 2016, 1:54 AM
 */

#ifndef SENSOR3_PUBLIC_H
#define	SENSOR3_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

    int sensor3SendTimerValToMsgQ(unsigned int millisecondsElapsed);
    
    void sensor3SendSensorValToSensorQ(unsigned char sensorValue);
    
    unsigned char sensor3ReceiveVal();


#ifdef	__cplusplus
}
#endif

#endif	/* SENSOR3_PUBLIC_H */

