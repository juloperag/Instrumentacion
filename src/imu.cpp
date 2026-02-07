#include <Arduino.h>
#include <stdio.h>
#include "imu.h"
#include <Wire.h>

//---------------------------------Contructor de la clase deviceBNO055-------------------------------
deviceBNO055::deviceBNO055(int address_I2C){ add_I2C = address_I2C; listData = {0, 0, 0, 0}; 
calData = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};};

//---------------------------------Funciones de la clase deviceBNO055-------------------------------
//Funcion lectura Data output
void deviceBNO055::readDataOuput(uint8_t DataOuput, uint16_t factorConData, uint8_t numberAxis)
{
    //Variables Adicionales
  char buffermessage[100];

  Wire.beginTransmission(add_I2C);
  Wire.write(DataOuput);
  Wire.endTransmission(false);
  Wire.requestFrom(add_I2C, numberAxis*2);
  //while (Wire.available()<6)

  for (int i=0; i<numberAxis;i++)
  {
    uint8_t lowByte = Wire.read();
    uint8_t highByte = Wire.read();
    listData[i] = ((int16_t)((highByte << 8) | lowByte)) / (float)factorConData;
  }
}

//Funcion para leer los registros de offset
void deviceBNO055::read_Data_Offset(void)
{
  Wire.beginTransmission(add_I2C);
  Wire.write(ACC_OFFSET);
  Wire.endTransmission(false);
  Wire.requestFrom(add_I2C, 22);
  //Lectura de de los bytes de los registros de offset
  for (int i=0; i<22;i++)
  {
    calData[i] = Wire.read();
  }
}

//Funcion para establecer la configuracion del IMU
void deviceBNO055::set_configuration_IMU(void)
{ 
  //Comprobacion de conexion del sensor
  uint8_t chipID_read = read_single_I2C(add_I2C, BNO055_CHIP_ID_ADDR);
  if (chipID_read == 0xA0) {
    //Entrar en modo normal
    write_I2C(add_I2C, OPR_MODE, CONFIGMODE);           //Iniciar Modo de configuracion
    write_I2C(add_I2C, PWR_MODE, NORMALMODE);           //Iniciar Modo de normal
    //Configuracion de offset
    for (int i = 0; i<22;i++)
    {
      write_I2C(add_I2C, ACC_OFFSET + i, calData[i]);   //Registro de offset
    }
    //Establecer modo de operacion
    write_I2C(add_I2C, OPR_MODE, mode_operation);       //Establecer modo NDOF y fin de configuracion
    Serial.write("Successfull set configuration IMU \n");
  }
  else{ Serial.write("BNO055 not found \n"); }
}

void deviceBNO055::read_calibration_status(void)
{
  //Read register
  uint8_t resg_cal_status = read_single_I2C(add_I2C, CALIB_STAT);
  //Almacenamiento de valores
  imu_status_cal[0] = (resg_cal_status >> 6)&(0b11);
  imu_status_cal[1] = (resg_cal_status >> 4)&(0b11);
  imu_status_cal[2] = (resg_cal_status >> 2)&(0b11);
  imu_status_cal[3] = (resg_cal_status)&(0b11);
}

// Metodo actualiza calData con los valores del array recibido
void deviceBNO055::update_Calibration(const uint8_t* newCalData, size_t len)
{
    if (newCalData == nullptr) return;           // nada que hacer
    const size_t maxLen = sizeof(calData)/sizeof(calData[0]);
    size_t copyLen = (len < maxLen) ? len : maxLen;

    for (size_t i = 0; i < copyLen; ++i) {
        calData[i] = newCalData[i];
    }
    // Si se pasó menos de maxLen, opcional: rellenar el resto con 0
    if (copyLen < maxLen) {
        for (size_t i = copyLen; i < maxLen; ++i) calData[i] = 0;
    }
}

//--------------------------------Funciones Auxiliares---------------------------------------------- 
//Funcion para escribir en un registro especifico
void write_I2C(uint8_t address_I2C, uint8_t add_Register, uint8_t value)
{
  Wire.beginTransmission(address_I2C);
  Wire.write(add_Register);       // Target register address
  Wire.write(value);     // Value to write
  Wire.endTransmission();  // Ends and sends
}

//Funcion para la lectura de un registro especifico
uint8_t read_single_I2C(uint8_t address_I2C, uint8_t add_Register) 
{
  Wire.beginTransmission(address_I2C);
  Wire.write(add_Register);
  Wire.endTransmission(false); // send repeated start
  Wire.requestFrom(address_I2C, (uint8_t) 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF; // error code
}