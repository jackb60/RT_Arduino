#include "SPI.h"
#include "DRV8452.h"

#define MOSI PA7
#define MISO PA6
#define SCLK PA5
#define CS_AZ PB0
#define CS_EL PB6

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

DRV8452 drv_azimuth(&SPI_3, settings, CS_AZ);
DRV8452 drv_elevation(&SPI_3, settings, CS_EL);

float elevation_target;
float azimuth_target;
int elevation_target_steps;
int azimuth_target_steps;
int elevation_steps;
int azimuth_steps;
float error_tolerance = 0.05;
long last_step_elevation;
long last_step_azimuth;

void setup() {
  Serial.begin(115200);
  SPI_3.begin();
  drv_azimuth.setup();
  drv_elevation.setup();
  last_step_elevation = micros();
  last_step_azimuth = micros();
}

void loop() {
  if (Serial.available() >= 10) {
    if(Serial.peek() == 0xAA){
      uint8_t data[10];
      Serial.readBytes((char*)data, 10);
      uint8_t checksum = 0;
      for(int i = 1; i < 9; i++){
        checksum += data[i];
      }
      if(checksum == data[9]){
        memcpy(&azimuth_target, &data[1], 4);
        memcpy(&elevation_target, &data[5], 4);
      } else {
        Serial.println("Checksum not met");
      }

      azimuth_target_steps = round(azimuth_target * (1/1.8) * 50.0);
      elevation_target_steps = round(elevation_target * (1/1.8) * 50.0);
    } else {
      Serial.read();
    }
  }

  if(micros() - last_step_azimuth >= 3600){
    if(azimuth_steps < azimuth_target_steps){
      drv_azimuth.fullStep(true);
      azimuth_steps ++;
      last_step_azimuth = micros();
    } else if(azimuth_steps > azimuth_target_steps){
      drv_azimuth.fullStep(false);
      azimuth_steps --;
      last_step_azimuth = micros();
    }
  }
  if(micros() - last_step_elevation >= 3600){
    if(elevation_steps < elevation_target_steps){
      drv_elevation.fullStep(true);
      elevation_steps ++;
      last_step_elevation = micros();
    } else if(elevation_steps > elevation_target_steps){
      drv_elevation.fullStep(false);
      elevation_steps --;
      last_step_elevation = micros();
    }
  }
  
}
