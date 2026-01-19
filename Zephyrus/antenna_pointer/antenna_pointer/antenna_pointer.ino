#include "SPI.h"
#include "DRV8452.h"

#define MOSI PA7
#define MISO PA6
#define SCLK PA5
#define CS_AZ PB0
#define SLP_AZ PB1
#define EN_AZ PB2
#define CS_EL PB6
#define SLP_EL PB7
#define EN_EL PB8

#define POINT_ERR 1

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE1);

DRV8452 drv_azimuth(&SPI_3, settings, CS_AZ, SLP_AZ, EN_AZ);
DRV8452 drv_elevation(&SPI_3, settings, CS_EL, SLP_EL, EN_EL);

float elevation_target;
float azimuth_target;
float old_a_target;
float old_e_target;
int elevation_target_steps;
int azimuth_target_steps;
int elevation_steps;
int azimuth_steps;
float error_tolerance = 0.05;
long last_step_elevation;
long last_step_azimuth;
unsigned long el_interval = 3600;
unsigned long az_interval = 3600;

unsigned long newStepsAz = 0;
unsigned long newStepsEl = 0;

unsigned long pastTAz = 0;
unsigned long pastTEl = 0;

void setup() {
  Serial.begin(115200);
  SPI_3.begin();
  drv_azimuth.setup();
  drv_elevation.setup();
  drv_azimuth.setHoldCurrentLimit(2.5);
  drv_azimuth.setStepCurrentLimit(2.5);
  drv_elevation.setHoldCurrentLimit(2.5);
  drv_elevation.setStepCurrentLimit(2.5);
  last_step_elevation = micros();
  last_step_azimuth = micros();
}

void loop() {
  if (Serial.available() >= 11) {
    if(Serial.peek() == 0xAA){
      uint8_t data[11];
      Serial.readBytes((char*)data, 11);
      uint8_t checksum = 0;
      for(int i = 1; i < 10; i++){
        checksum += data[i];
      }
      if(checksum == data[10]){
        switch (data[1]){
          case 0x00:
            old_a_target = azimuth_target;
            old_e_target = elevation_target;
            memcpy(&azimuth_target, &data[2], 4); //0 - 360 deg
            azimuth_target *= -1.0; 
            memcpy(&elevation_target, &data[6], 4); // -90 to 90 deg
            azimuth_target = (abs(azimuth_target - old_a_target) > POINT_ERR ? azimuth_target : old_a_target);
            elevation_target = abs(elevation_target - old_e_target) > POINT_ERR ? elevation_target : old_e_target;
            break;
          case 0x01:
            elevation_target += 5;
            break;
          case 0x02:
            elevation_target -= 5;
            break;
          case 0x03:
            azimuth_target += 5;
            break;
          case 0x04:
            azimuth_target -= 5;
            break;
          case 0x05:
            elevation_steps = 0;
            azimuth_steps = 0;
            elevation_target = 0.0;
            azimuth_target = 0.0;
            break;
        }
      }


      Serial.println(elevation_target);      
      azimuth_target_steps = round(azimuth_target * (1/1.8) * 50.0 * 4.0);
      elevation_target_steps = round(elevation_target * (1/1.8) * 50.0 * 4.0);
      shortest_path();

      newStepsAz = 0;
      newStepsEl = 0;

      pastTAz = 0;
      pastTEl = 0;

      az_adjust();
      el_adjust();
      
    } else {
      Serial.read();
    }
  }

  if(micros() - last_step_azimuth >= az_interval){
    if(azimuth_steps < azimuth_target_steps){
      drv_azimuth.fullStep(true);
      azimuth_steps ++;
      newStepsAz ++;
      last_step_azimuth = micros();
      if (az_interval > 1000) { az_adjust(); };
    } else if(azimuth_steps > azimuth_target_steps){
      drv_azimuth.fullStep(false);
      azimuth_steps --;
      newStepsAz ++;
      last_step_azimuth = micros();
      if (az_interval > 1000) { az_adjust(); };
    }
  }
  if(micros() - last_step_elevation >= el_interval){
    if(elevation_steps < elevation_target_steps){
      drv_elevation.fullStep(true);
      elevation_steps ++;
      newStepsEl ++;
      last_step_elevation = micros();
      if (el_interval > 1000) { el_adjust(); };
    } else if(elevation_steps > elevation_target_steps){
      drv_elevation.fullStep(false);
      elevation_steps --;
      newStepsEl ++;
      last_step_elevation = micros();
      if (el_interval > 1000) { el_adjust(); };
    }
  }
  
}

void shortest_path(){
  while(abs(azimuth_target_steps - azimuth_steps) > 4 * 5000){ //if we are > 180 deg away, we can do better
    if(azimuth_target_steps - azimuth_steps > 0){
      azimuth_target_steps -= 4 * 10000; //decrease 360deg
    } else {
      azimuth_target_steps += 4 * 10000; //increase 360deg
    }
  }
}

/*
Given constant angular acceleration, max deg (t) = 0.5 * a * (t^2)
To find the next time, we solve for t: t = sqrt(2 * max deg / a)

From spreadsheet: with 2 Nm motor toque our max angular acceleration is 1255 deg/s^2
So t = sqrt(2 * max deg / 1255)
*/ 

void az_adjust() {
  unsigned long newT = sqrt((2.0 * ((newStepsAz + 1)/ 4.0) / 1255.0)) * 1000000.0;
  az_interval = newT - pastTAz;
  pastTAz = newT;
}

void el_adjust() {
  unsigned long newT = sqrt((2.0 * ((newStepsEl + 1)/ 4.0) / 1255.0)) * 1000000.0;
  el_interval = newT - pastTEl;
  pastTEl = newT;
}
