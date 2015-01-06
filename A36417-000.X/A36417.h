/* 
 * File:   A36417.h
 * Author: cosorio
 *
 * Created on January 5, 2015, 3:28 PM
 */

#ifndef __A36417_H
#define	__A36417_H


#include <p30f6014a.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>

#include "ETM_CAN_PUBLIC.h"
#include "ETM_ANALOG.h"

typedef struct{
    AnalogInput analog_input_ion_pump_current;
    AnalogInput analog_input_ion_pump_voltage;
    AnalogInput analog_input_target_current;
    
    AnalogOutput analog_output_emco_control;
}IonPumpControlData;


#endif	/* A36417_H */

