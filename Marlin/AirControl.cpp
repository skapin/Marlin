
#include "Marlin.h"
#include "AirControl.h"
#include "watchdog.h"
/**
 * @since may 2013
 * @author  Florian boudinet
 * @contact  fboudinet@gmail.com
 * 
 * this file follow the pattern given by temperature.[cpp/h]
 **/

//===========================================================================
//=============================public variables============================
//===========================================================================

int target_temperature_air = 0;
int current_temperature_air_raw = 0;
float current_temperature_air = 0;

#ifdef PIDTEMPAIR
  float airKp=DEFAULT_airKp;
  float airKi=(DEFAULT_airKi*PID_dT);
  float airKd=(DEFAULT_airKd/PID_dT);
#endif //PIDTEMPAIR


//===========================================================================
//=============================private variables============================
//===========================================================================

#ifdef PIDTEMPAIR
  //static cannot be external:
  static float temp_iState_air = { 0 };
  static float temp_dState_air = { 0 };
  static float pTerm_air;
  static float iTerm_air;
  static float dTerm_air;
  //int output;
  static float pid_error_air;
  static float temp_iState_min_air;
  static float temp_iState_max_air;
#else //PIDTEMPAIR
	static unsigned long  previous_millis_air_heater;
#endif //PIDTEMPAIR
  static unsigned char soft_pwm_air;

#ifdef AIR_MAXTEMP
static int air_maxttemp_raw = HEATER_AIR_RAW_HI_TEMP;
#endif

static float analog2tempAir(int raw);
static void updateTemperaturesFromRawValues();
  

void updatePID_air()
{
#ifdef PIDTEMPAIR
  temp_iState_max_air = PID_INTEGRAL_DRIVE_MAX / airKi;  
#endif
}

int getHeaterAirPower( ) {
	return soft_pwm_air;
  
}
  
void manage_air() {
// is AirControl activated ?
#if TEMP_SENSOR_AIR  == 0
	return;
#endif
	float pid_input;
	float pid_output;

	updateTemperaturesFromRawValues_air();

	pid_input = current_temperature_air;

	pid_error_air = target_temperature_air - pid_input;
	pTerm_air = airKp * pid_error_air;
	temp_iState_air += pid_error_air;
	temp_iState_air = constrain(temp_iState_air, temp_iState_min_air, temp_iState_max_air);
	iTerm_air = airKi * temp_iState_air;

	//K1 defined in Configuration.h in the PID settings
	#define K2 (1.0-K1)
	dTerm_air= (airKd * (pid_input - temp_dState_air))*K2 + (K1 * dTerm_air);
	temp_dState_air = pid_input;
	pid_output = constrain(pTerm_air + iTerm_air - dTerm_air, 0, MAX_AIR_POWER);

    if((current_temperature_air > AIR_MINTEMP) && (current_temperature_air < AIR_MAXTEMP)) 
	{
		soft_pwm_air = (int)pid_output >> 1;
	}
	else {
		soft_pwm_air = 0;
	}
}




