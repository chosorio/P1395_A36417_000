/* 
 * File:   A36417_SETTINGS.h
 * Author: hwanetick
 *
 * Created on December 21, 2015, 10:18 AM
 */

#ifndef A36417_SETTINGS_H
#define	A36417_SETTINGS_H


#define ION_PUMP_VOLTAGE_SCALE_FACTOR            .0764              //1V per 1kV
#define ION_PUMP_VOLTAGE_OVER_TRIP_POINT         3300               //3.3kV
#define ION_PUMP_VOLTAGE_UNDER_TRIP_POINT        2900               //2.9kV
//#define ION_PUMP_VOLTAGE_RELATIVE_TRIP           MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
//#define ION_PUMP_VOLTAGE_RELATIVE_FLOOR
//#define ION_PUMP_VOLTAGE_RELATIVE_TRIP_TIME      50                 // This is in 10ms Units
#define ION_PUMP_VOLTAGE_ABSOLUTE_TRIP_TIME      50                 // This is in 10ms Units

#define ION_PUMP_CURRENT_SCALE_FACTOR            2*.0763            //1V per 2uA
#define ION_PUMP_CURRENT_OVER_TRIP_POINT         6000               //
#define ION_PUMP_CURRENT_UNDER_TRIP_POINT        0                 //
//#define ION_PUMP_CURRENT_RELATIVE_TRIP           MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
//#define ION_PUMP_CURRENT_RELATIVE_FLOOR
//#define ION_PUMP_CURRENT_RELATIVE_TRIP_TIME      50                 // This is in 10ms Units
#define ION_PUMP_CURRENT_ABSOLUTE_TRIP_TIME      50                 // This is in 10ms Units

#define TARGET_CURRENT_SCALE_FACTOR              0.0763/5           //placeholder
#define TARGET_CURRENT_OVER_TRIP_POINT           555                //placeholder
#define TARGET_CURRENT_UNDER_TRIP_POINT          10                 //placeholder
//#define TARGET_CURRENT_RELATIVE_TRIP             MACRO_DEC_TO_CAL_FACTOR_2(.25)        // 25%
//#define TARGET_CURRENT_RELATIVE_FLOOR
//#define TARGET_CURRENT_RELATIVE_TRIP_TIME        50                 // This is in 10ms Units
#define TARGET_CURRENT_ABSOLUTE_TRIP_TIME        50                 // This is in 10ms Units


#define _5V_MONITOR_SCALE_FACTOR                  0.0763
#define _5V_MONITOR_OVER_TRIP_POINT               2600
#define _5V_MONITOR_UNDER_TRIP_POINT              2400

#define _15V_MONITOR_SCALE_FACTOR                 0.0763
#define _15V_MONITOR_OVER_TRIP_POINT              2600
#define _15V_MONITOR_UNDER_TRIP_POINT             2400

#define MINUS_5V_MONITOR_SCALE_FACTOR             0.0763
#define MINUS_5V_MONITOR_OVER_TRIP_POINT          1770
#define MINUS_5V_MONITOR_UNDER_TRIP_POINT         1570

#define EMCO_CTRL_VOLTAGE_SCALE_FACTOR            1
#define EMCO_MAX_CTRL_VOLTAGE                     5  //Placeholder
#define EMCO_MIN_CTRL_VOLTAGE                     1  //Placeholder



#endif	/* A36417_SETTINGS_H */

