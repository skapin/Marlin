#ifndef AIRCONTROL_H
#define AIRCONTROL_H

// public functions
void manage_air();

extern int target_temperature_air;
extern float current_temperature_air;

#ifdef PIDTEMPAIR
  extern float airKp, airKi, airKd;
#endif


FORCE_INLINE float degAir() {
  return current_temperature_air;
};

FORCE_INLINE float degTargetAir() {   
  return target_temperature_air;
};

FORCE_INLINE void setTargetAir(const float &celsius) {  
  target_temperature_air = celsius;
};

FORCE_INLINE bool isHeatingAir() {
  return target_temperature_air > current_temperature_air;
};

FORCE_INLINE bool isCoolingAir() {
  return target_temperature_air < current_temperature_air;
};

void air_init();
void updatePID_air();
int getHeaterAirPower( );

#endif //AIRCONTROL
