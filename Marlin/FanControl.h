#ifndef FANCONTROL_H
#define FANCONTROL_H

#include "Marlin.h"
#include "AirControl.h"
#include "watchdog.h"
#include "fastio.h"

#define BASE_LC_NAME lastExtruderCheck_E
#define LAST_CHECK_E0 BASE_LC_NAME##0
#define LAST_CHECK_E1 BASE_LC_NAME##1

unsigned long LAST_CHECK_E0 = 0;
unsigned long LAST_CHECK_E1 = 0;


#define CHECK_FAN(EXTRUDER_NUM) \
 if(HEATER_##EXTRUDER_NUM##_PIN > 0) \
 {\
 	if ((millis() - LAST_CHECK_E##EXTRUDER_NUM) >= 2500) \
	{ \
		LAST_CHECK_E##EXTRUDER_NUM = millis(); \
		int value = LOW; \
		if (degHotend(EXTRUDER_NUM) < FAN_TEMP_ACTIVE) \
		{ \
			value = LOW;\
		} \
		else \
		{ \
			value = HIGH;\
		} \
		if(FAN_0_E##EXTRUDER_NUM##_PIN > -1) { \
			WRITE(FAN_0_E##EXTRUDER_NUM##_PIN, value); \
		} \
		if(FAN_1_E##EXTRUDER_NUM##_PIN > -1) { \
			WRITE(FAN_1_E##EXTRUDER_NUM##_PIN, value); \
		} \
		if(FAN_MOTOR_E##EXTRUDER_NUM##_PIN > -1) { \
			WRITE(FAN_MOTOR_E##EXTRUDER_NUM##_PIN, value); \
		} \
	} \
 } 
void control_fan() { //Check if fans should be turned on or off
	//We have an extruder active, setup fans
	if ( EXTRUDERS > 1 ) {
		CHECK_FAN(1)
	}
	if ( EXTRUDERS > 0 ) {
	//else if (active_extruder == 1 )
		CHECK_FAN(0)
	}
}
#define INIT_FAN_IF(VARIABLE_NAME,EXTRUDER_NUM) \
if( VARIABLE_NAME##EXTRUDER_NUM##_PIN > -1 ){ \
	SET_OUTPUT( VARIABLE_NAME##EXTRUDER_NUM##_PIN );\
}
void fan_init() {	
	if ( EXTRUDERS > 1 ) {
		INIT_FAN_IF( FAN_0_E, 1)
		INIT_FAN_IF( FAN_1_E, 1)
		INIT_FAN_IF( FAN_MOTOR_E, 1)
	}
	if ( EXTRUDERS > 1 ) {
		INIT_FAN_IF( FAN_0_E, 0)
		INIT_FAN_IF( FAN_1_E, 0)
		INIT_FAN_IF( FAN_MOTOR_E, 0)
	}
}

#endif //FANCONTROL_H
