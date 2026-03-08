#include <SPI.h>
#include <CC1200.h>
#include <baro.h>
#include <ADXL357.h>
#include <gyro.h>
#include <flash.h>
#include <cam.h>
#include <vtx.h>
#include <pyro.h>


#define MOSI PD7
#define MISO PB4
#define SCLK PA5
#define MAG_CS PD0
#define GYRO_CS PD1
#define FLASH_CS PD4
#define ACCEL_CS PD2
#define BARO_CS  PD3
#define CC_CS PC12


HardwareSerial debugSer(PC7, PC6);
HardwareSerial gpsSer(PB12, PB13);
HardwareSerial CAM0_SER(PE7, PE8);
HardwareSerial CAM1_SER(PE0, PE1);
HardwareSerial CAM2_SER(PB15, PB14);
HardwareSerial VTX_SER(PA12);


SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

CC1200 cc(&SPI_3, settings, CC_CS);
ADXL357 accel(&SPI_3, settings, ACCEL_CS);
baro barometer(&SPI_3, settings, BARO_CS);
gyro mygyro(&SPI_3, settings, GYRO_CS);
flash flashMem(&SPI_3, settings, FLASH_CS);

cam cam0(&CAM0_SER);
cam cam1(&CAM1_SER);
cam cam2(&CAM2_SER);
vtx myVTX(&VTX_SER);

pyro pyros;

enum State {
  GROUND_TESTING,
  PRE_FLIGHT,
  FLIGHT,
  APOGEE,
  DISREEF,
  END
};

enum PyroStatus {
  PYRO_FAILURE,
  PYRO_UNCONNECTED,
  PYRO_CONNECTED,
  PYRO_SUCCESS
};

State currentState = GROUND_TESTING;

State recState = GROUND_TESTING;


void setup() {
  // put your setup code here, to run once:
  delay(100);

  pinMode(MAG_CS, OUTPUT); //Remove when mag library added
  digitalWrite(MAG_CS, 1); //Remove when mag library added

  SPI_3.begin();

  pyros.begin();
  accel.setup();
  barometer.begin();
  mygyro.begin();
  mygyro.config();
  flashMem.begin();
  myVTX.begin();
  cam0.begin();
  cam1.begin();
  cam2.begin();
  cc.begin();
  cc.simpleConfig();

}
void loop() {
}