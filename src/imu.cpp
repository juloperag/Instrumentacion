#include <Arduino.h>
#include <stdio.h>
#include "imu.h"
#include <Wire.h>

//---------------------------------Contructor de la clase deviceBNO055-------------------------------
deviceBNO055::deviceBNO055(void){ listData = {0, 0, 0, 0};};

//---------------------------------Funciones de la clase deviceBNO055-------------------------------
//Funcion lectura Data output
void deviceBNO055::readDataOuput(uint8_t DataOuput, uint16_t factorConData, uint8_t numberAxis)
{
    //Variables Adicionales
  char buffermessage[100];

  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(DataOuput);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDR, numberAxis*2);
  //while (Wire.available()<6)

  for (int i=0; i<numberAxis;i++)
  {
    uint8_t lowByte = Wire.read();
    uint8_t highByte = Wire.read();
    listData[i] = ((int16_t)((highByte << 8) | lowByte)) / (float)factorConData;
  }
}

//Funcion configuracion IMU
void deviceBNO055::config_IMU(uint8_t add_Register, uint8_t value_config)
{
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write(add_Register);       // Target register address
  Wire.write(value_config);     // Value to write
  Wire.endTransmission();  // Ends and sends
}