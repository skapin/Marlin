#ifndef AIRCONTROL_H
#define AIRCONTROL_H

// public functions
void manage_air();
float calculate_air_temp();
void disable_air();

void air_init();
void updatePID_air();
int receiveCommand( char delim );
void sendCommand(char* action, char* variable, double value );
void parseDatas( );
void printAirStatus();
void PID_air_autotune(float temp, int ncycles);

extern int target_temperature_air;
extern float current_temperature_air[TEMP_AIR_NUMBER];
extern unsigned char soft_pwm_air[HEATER_AIR_NUMBER];

#ifdef PIDTEMPAIR
  extern float airKp, airKi, airKd;
#endif

FORCE_INLINE float degAir( unsigned int device ) {
  return current_temperature_air[device];
};

FORCE_INLINE float degTargetAir() {   
  return target_temperature_air;
};

FORCE_INLINE void setTargetAir(const float &celsius) {  
  target_temperature_air = celsius;
};

FORCE_INLINE float getHeaterAirPower( int heater_number) {
	return soft_pwm_air[heater_number]; 
}
/*
FORCE_INLINE bool isHeatingAir() {
  return target_temperature_air > current_temperature_air;
};*/
/*
FORCE_INLINE bool isCoolingAir() {
  return target_temperature_air < current_temperature_air;
};*/


#endif //AIRCONTROL
