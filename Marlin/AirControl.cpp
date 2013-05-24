
#include "Marlin.h"
#include "AirControl.h"
#include "watchdog.h"
#include "fastio.h"

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
static volatile bool temp_meas_ready = false; //syncr. variable (for interrupt)
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
static void updateTemperaturesFromRawValues_air();
  

void air_init() {
	#ifdef PIDTEMPAIR
		temp_iState_min_air = 0.0;
		temp_iState_max_air = PID_INTEGRAL_DRIVE_MAX / airKi;
	#endif //PIDTEMPAIR
	
	//on definit HEATER_AIR_0/1_PIN comme  outpute
	#if (HEATER_AIR_0_PIN > -1) 
		SET_OUTPUT(HEATER_AIR_0_PIN);
	#endif //HEATER_AIR_0_PIN
	#if (HEATER_AIR_1_PIN > -1) 
		SET_OUTPUT(HEATER_AIR_1_PIN);
	#endif //HEATER_AIR_1_PIN
	
	//DIDR0/2 Disable Inpute Digital
	// i.e we want to disable the digital inpute of AIR pin. Cause it's Analog :)
	#if (TEMP_AIR_PIN > -1)
		#if TEMP_AIR_PIN < 8
		   DIDR0 |= 1<<TEMP_AIR_PIN; 
		#else
		   DIDR2 |= 1<<(TEMP_AIR_PIN - 8); 
		#endif
	 #endif
	 
	#ifdef AIR_MAXTEMP
	  while(analog2tempAir(air_maxttemp_raw) > AIR_MAXTEMP) {
	#if HEATER_AIR_RAW_LO_TEMP < HEATER_AIR_RAW_HI_TEMP
		air_maxttemp_raw -= OVERSAMPLENR;
	#else
		air_maxttemp_raw += OVERSAMPLENR;
	#endif
	  }
	#endif //AIR_MAXTEMP
}

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
	if(temp_meas_ready != true)   //better readability
		return; 

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

#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempAir(int raw) {
  #ifdef AIR_USES_THERMISTOR
    float celsius = 0;
    byte i;

    for (i=1; i<AIRTEMPTABLE_LEN; i++)
    {
      if (PGM_RD_W(AIRTEMPTABLE[i][0]) > raw)
      {
        celsius  = PGM_RD_W(AIRTEMPTABLE[i-1][1]) + 
          (raw - PGM_RD_W(AIRTEMPTABLE[i-1][0])) * 
          (float)(PGM_RD_W(AIRTEMPTABLE[i][1]) - PGM_RD_W(AIRTEMPTABLE[i-1][1])) /
          (float)(PGM_RD_W(AIRTEMPTABLE[i][0]) - PGM_RD_W(AIRTEMPTABLE[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == AIRTEMPTABLE_LEN) celsius = PGM_RD_W(AIRTEMPTABLE[i-1][1]);

    return celsius;
  #else
    return 0;
  #endif
}


/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues_air()
{
    current_temperature_air = analog2tempAir(current_temperature_air_raw);

    //Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    CRITICAL_SECTION_START;
    temp_meas_ready = false;
    CRITICAL_SECTION_END;
}


