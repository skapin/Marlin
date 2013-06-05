
#include "Marlin.h"
#include "AirControl.h"
#include "watchdog.h"
#include "fastio.h"
#include "temperature.h"
/**
 * @since may 2013
 * @author  Florian boudinet
 * @contact  fboudinet@gmail.com
 * 
 * this file follow the pattern given by temperature.[cpp/h]
 **/

#define END_COMMAND_DELIMITER '.'

//===========================================================================
//=============================public variables============================
//===========================================================================
double maxtemp_air= AIR_MAXTEMP;

int target_temperature_air = 0;
#if TEMP_AIR_NUMBER > 0
float current_temperature_air[TEMP_AIR_NUMBER] = {0};
#endif
#if HEATER_AIR_NUMBER > 0
unsigned char soft_pwm_air[HEATER_AIR_NUMBER] = {0};
#endif

#ifdef PIDTEMPAIR
  float airKp=DEFAULT_airKp;
  float airKi=(DEFAULT_airKi*PID_dT);
  float airKd=(DEFAULT_airKd/PID_dT);
#endif //PIDTEMPAIR


//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false; //syncr. variable (for interrupt)
static unsigned long last_millis_check = 0;


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


static float analog2tempAir(int raw, const short  table[][2]);
static void updateTemperaturesFromRawValues_air();
  

void air_init() {
	#ifdef PIDTEMPAIR
		temp_iState_min_air = 0.0;
		temp_iState_max_air = PID_INTEGRAL_DRIVE_MAX / airKi;
	#endif //PIDTEMPAIR
	
	//on definit HEATER_AIR_0/1_PIN comme  output
	#if (HEATER_AIR_0_PIN > -1) 
		SET_OUTPUT(HEATER_AIR_0_PIN);
	#endif //HEATER_AIR_0_PIN
	#if (HEATER_AIR_1_PIN > -1) 
		SET_OUTPUT(HEATER_AIR_1_PIN);
	#endif //HEATER_AIR_1_PIN
	
	//DIDR0/2 Disable Inpute Digital
	// i.e we want to disable the digital input of AIR pin. Cause it's Analog :)
	#if (TEMP_AIR_0_PIN > -1)
		#if TEMP_AIR_0_PIN < 8
		   DIDR0 |= 1<<TEMP_AIR_0_PIN; 
		#else
		   DIDR2 |= 1<<(TEMP_AIR_0_PIN - 8); 
		#endif
	 #endif
	 
	 #if (TEMP_AIR_1_PIN > -1)
		#if TEMP_AIR_1_PIN < 8
		   DIDR0 |= 1<<TEMP_AIR_1_PIN; 
		#else
		   DIDR2 |= 1<<(TEMP_AIR_1_PIN - 8); 
		#endif
	 #endif
	 
	 #if (TEMP_AIR_2_PIN > -1)
		#if TEMP_AIR_2_PIN < 8
		   DIDR0 |= 1<<TEMP_AIR_2_PIN; 
		#else
		   DIDR2 |= 1<<(TEMP_AIR_2_PIN - 8); 
		#endif
	 #endif
	 
}

void disable_air() {
  #if ( HEATER_AIR_0_PIN  > -1 )
	target_temperature_air = 0;
	soft_pwm_air[0] = 0;
    WRITE( HEATER_AIR_0_PIN, LOW );
  #endif 
  #if ( HEATER_AIR_1_PIN  > -1 )
	target_temperature_air = 0;
	soft_pwm_air[1] = 0;
    WRITE( HEATER_AIR_1_PIN, LOW );
  #endif   
}

void updatePID_air()
{
#ifdef PIDTEMPAIR
  temp_iState_max_air = PID_INTEGRAL_DRIVE_MAX / airKi;  
#endif
}

/**
 * Based on PID_autotune() temperature.cpp.
 * **/
void PID_air_autotune(float temp, int ncycles)
{
  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;


  #if (TEMP_AIR_0_PIN <= -1 || TEMP_SENSOR_AIR_0 == 0 || HEATER_AIR_NUMBER == 0 || TEMP_AIR_NUMBER == 0 )

	)
		SERIAL_ECHOLN("PID_air Autotune failed. No pin conf. or number of heater/therm set to 0...");
		return;
	#endif	
	
	SERIAL_ECHOLN("PID_AIR Autotune start");
  
	disable_heater(); // switch off all heaters.

	soft_pwm_air[0] = (MAX_AIR_POWER)/2;
	bias = d = (MAX_AIR_POWER)/2;


 for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues_air();

      input = calculate_air_temp();

      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) { 
          heating=false;
					soft_pwm_air[0] = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(MAX_AIR_POWER)-20);
            if(bias > MAX_AIR_POWER/2) d = MAX_AIR_POWER - 1 - bias;
            else d = bias;

            SERIAL_PROTOCOLPGM(" bias: "); SERIAL_PROTOCOL(bias);
            SERIAL_PROTOCOLPGM(" d: "); SERIAL_PROTOCOL(d);
            SERIAL_PROTOCOLPGM(" min: "); SERIAL_PROTOCOL(min);
            SERIAL_PROTOCOLPGM(" max: "); SERIAL_PROTOCOLLN(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              SERIAL_PROTOCOLPGM(" Ku: "); SERIAL_PROTOCOL(Ku);
              SERIAL_PROTOCOLPGM(" Tu: "); SERIAL_PROTOCOLLN(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              SERIAL_PROTOCOLLNPGM(" Clasic PID ")
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              /*
              Kp = 0.33*Ku;
              Ki = Kp/Tu;
              Kd = Kp*Tu/3;

              Kp = 0.2*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/3;

              */
            }
          }
		soft_pwm_air[0] = (bias + d) >> 1;
        cycles++;
        min=temp;
        }
      } 
    }
    if(input > (temp + 20)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature to high");
      return;
    }
    if(millis() - temp_millis > 2000) {
		int p;
	    p=soft_pwm_air[0];       
	    SERIAL_PROTOCOLPGM("ok B:");	
		SERIAL_PROTOCOL(input);   
		SERIAL_PROTOCOLPGM(" @:");
		SERIAL_PROTOCOLLN(p);       

		temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
      return;
    }
    if(cycles > ncycles) {
      SERIAL_PROTOCOLLNPGM("PID Autotune finished ! Place the Kp, Ki and Kd constants in the configuration.h");
      return;
    }
  }
}


void manage_air() {
// is AirControl activated ?
  #if (TEMP_SENSOR_AIR_0  != 0 && HEATER_AIR_0_PIN != -1 )
	
	float pid_input;
	float pid_output;
	float current_temp;

	//convert Raw values to readable value
	updateTemperaturesFromRawValues_air();
	
	
	current_temp = pid_input = calculate_air_temp();
	
	// air too hot ?, better stop
	if ( current_temp > maxtemp_air ) {
		disable_air();
	}
	else {

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

		for (int a = 0; a < HEATER_AIR_NUMBER ; ++a ) {
			if((current_temp > AIR_MINTEMP) && (current_temp < AIR_MAXTEMP)) 
			{
				soft_pwm_air[a] = (int)pid_output >> 1;
			}
			else {
				soft_pwm_air[a] = 0;
			}
		}
	}
#endif
}

float calculate_air_temp() {
	float sum = 0.0;
	for (int a =0; a < TEMP_AIR_NUMBER; ++a ) {
		sum +=current_temperature_air[a];
	}
	return sum/TEMP_AIR_NUMBER;
	 
 }
 
#define PGM_RD_W(x)   (short)pgm_read_word(&x)
// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempAir(int raw, const short table[][2], int table_size) {
    float celsius = 0;
    byte i;
    for (i=1; i<table_size; i++)
    {
      if (PGM_RD_W(table[i][0]) > raw)
      {
        celsius  = PGM_RD_W(table[i-1][1]) + 
          (raw - PGM_RD_W(table[i-1][0])) * 
          (float)(PGM_RD_W(table[i][1]) - PGM_RD_W(table[i-1][1])) /
          (float)(PGM_RD_W(table[i][0]) - PGM_RD_W(table[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == table_size) celsius = PGM_RD_W(table[i-1][1]);

    return celsius;
 // #else
    return 0;
 // #endif
}


/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues_air()
{
	int temp=0;
	#ifdef THERMISTORAIR_0
		CRITICAL_SECTION_START;
		temp = getRawTempAir(0);
		CRITICAL_SECTION_END;
		current_temperature_air[0] = analog2tempAir( temp, AIR_0_TEMPTABLE, AIR_0_TEMPTABLE_LEN);	 
	#endif
	#ifdef THERMISTORAIR_1
		CRITICAL_SECTION_START;
		temp = getRawTempAir(1);
		CRITICAL_SECTION_END;
		current_temperature_air[1] = analog2tempAir( temp, AIR_1_TEMPTABLE, AIR_1_TEMPTABLE_LEN);
	#endif
	#ifdef THERMISTORAIR_2
		CRITICAL_SECTION_START;
		temp = getRawTempAir(2);
		CRITICAL_SECTION_END;
		current_temperature_air[2] = analog2tempAir( temp, AIR_2_TEMPTABLE, AIR_2_TEMPTABLE_LEN);
	#endif
	
	    
}
// print current air status (temp/target)
void printAirStatus() {
	
	#if (TEMP_SENSOR_AIR_0 != 0 || TEMP_SENSOR_AIR_1 != 0 || TEMP_SENSOR_AIR_2 != 0 )
		SERIAL_PROTOCOLPGM("(");
		#if (TEMP_SENSOR_AIR_0 > 0 )
			SERIAL_PROTOCOLPGM(" A0:");
			SERIAL_PROTOCOL_F(degAir(0),1);         
		#endif
		#if (TEMP_SENSOR_AIR_1 > 0 )
   		    SERIAL_PROTOCOLPGM(" A1:");
		    SERIAL_PROTOCOL_F(degAir(1),1); 
        #endif
        #if (TEMP_SENSOR_AIR_2 > 0 )
            SERIAL_PROTOCOLPGM(" A2:");
	        SERIAL_PROTOCOL_F(degAir(2),1); 
        #endif
        SERIAL_PROTOCOLPGM(" )/");
        SERIAL_PROTOCOL_F(degTargetAir(),1);
        SERIAL_PROTOCOLPGM(" ");
	  #endif
}

/*
int receiveCommand( char delim ) {
    while (Serial.available() > 0  && write_ptr < MAX_BYTE_RECEIVED ) {
      datas[write_ptr] = Serial.read();

      if ( datas[write_ptr] == delim ) {
          read_ptr = write_ptr + 1;
          strncpy(command, datas, read_ptr);
          command[read_ptr-1] = '\0';
          write_ptr = 0;
          return 1;
        } 
        else {
          write_ptr++;
        }
    }
    return 0;
}

void sendCommand(char* action, char* variable, double value ) {
  Serial.print(action); 
  Serial.print(' '); 
  Serial.print(variable); 
  Serial.print(' '); 
  Serial.print(value); 
  Serial.print(END_COMMAND_DELIMITER); 
}
void parseDatas( ) {
  int lenght_action = (strchr(command,' ')-command);
  int lenght_variable = 0;
  char* ptr = NULL;
  if ( strncmp("set",command,lenght_action ) == 0 ) {
      ptr = &(command[lenght_action+1]);
      lenght_variable = strchr( ptr, ' ') - ptr;
      double value = strtod( ptr+lenght_variable, NULL);
      if ( strncmp("ki", ptr, lenght_variable ) == 0 ) {
          airKi = value;
      }    
      else if ( strncmp("kp", ptr, lenght_variable ) == 0 ) {
          airKp = value;
      }
      else if ( strncmp("kd", ptr, lenght_variable ) == 0 ) {
          airKd = value;
      }
      else if ( strncmp("maxtemp", ptr, lenght_variable ) == 0 ) {
          maxtemp_air = value;
      }
      else if ( strncmp("targettemp", ptr, lenght_variable ) == 0 ) {
          target_temperature_air = value;
      }
     
  }
  else if ( strncmp("get",command,lenght_action ) == 0 ) {
      ptr = &(command[lenght_action+1]);
      lenght_variable = strchr( ptr, '\0') - ptr;
      if ( strncmp("ki", ptr, lenght_variable ) == 0 ) {
          sendCommand("set","ki",airKi);
      }    
      else if ( strncmp("kp", ptr, lenght_variable ) == 0 ) {
          sendCommand("set","kp",airKp);

      }
      else if ( strncmp("kd", ptr, lenght_variable ) == 0 ) {
          sendCommand("set","kd",airKd);
      }
      else if ( strncmp("maxtemp", ptr, lenght_variable ) == 0 ) {
          sendCommand("set","maxtemp",maxtemp_air);
      }
      else if ( strncmp("temp", ptr, lenght_variable ) == 0 ) {
          sendCommand("set","temp", target_temperature_air);
      }

   }
	//user want to start PID autotune process
	else if ( strncmp("start_autotune_pid",command,lenght_action ) == 0 ) {
		double temp = strtod( ptr, NULL);
		int ncycle = strtod( ptr, NULL);
		PID_air_autotune( temp, ncycles );
	}

}

*/
