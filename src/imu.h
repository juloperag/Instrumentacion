#ifndef _IMU_H_
#define _IMU_H_

#include <Arduino.h>
#include <stdio.h>
#include <Wire.h>
#include <array>

//Macro de direccion del sensor
#define BNO055_ADDR_A 0x29
#define BNO055_ADDR_B 0x28

//Registro de calibracion
#define CALIB_STAT 0x35

//Macro de registro de chip
#define BNO055_CHIP_ID_ADDR 0x00

//Macros Power mode
#define PWR_MODE   0x3E
#define NORMALMODE 0b00
//Macros Operation Mode
#define OPR_MODE 0x3D    //Direccion registro de configuracion
#define CONFIGMODE 0b0000 //Config mode
#define ACCONLY 0b0001   //No-Fusion Mode
#define MAGONLY 0b0010
#define GYROONLY 0b0011
#define ACCMAG 0b00100
#define ACCGYRO 0b0101
#define MAGGYRO 0b0110
#define AMG 0b0111 
#define IMU 0b1000       //Fusion Mode
#define COMPASS 0b1001 
#define M4G 0b1010
#define NDOF_FMC_OFF 0b1011
#define NDOF 0b1100

//Macros DataOutput
#define ACC_DATA 0x08
#define MAG_DATA 0x0E
#define GYR_DATA 0x14
#define EUL_DATA 0x1A
#define QUA_DATA 0x20 //4 Axis
#define LIA_DATA 0x28
#define GRV_DATA 0x2E

//Macros Config y FactorConData
#define ACC_CONFIG 0x08   //Direccion registros de configuracion
#define MAG_CONFIG 0x09
#define GYR_CONFIG_0 0x0A
#define CONFIG_ACCEL_2G  0b00   /// EN modo Fusion solo se puede establecer el ACCEL,
#define CONFIG_ACCEL_4G   0b01  // los demas se ajustan automaticamente
#define CONFIG_MAG_2HZ   0b000
#define CONFIG_MAG_6HZ   0b001
#define CONFIG_MAG_8HZ   0b010
#define CONFIG_GYR_150   0b100
#define CONFIG_GYR_250   0b011  
#define FACTOR_ACCEL 100  //LSB
#define FACTOR_MAG 16   //LSB
#define FACTOR_GYR 16   //LSB
#define FACTOR_EUL 16   //LSB
#define FACTOR_QUA 16384 //LSB
#define FACTOR_LIA 100   //LSB
#define FACTOR_GRV 100   //LSB

//Macros OFFSET
#define ACC_OFFSET 0x55

class deviceBNO055{
public:
    //Atributos
    std::array<float, 4> listData = {0, 0, 0, 0}; //Arreglo para almacenar los datos de salida del IMU
    int mode_operation;                           //Define el modo de operacion del imu
    int add_I2C;                                  //Direccion I2C     
    std::array<uint8_t, 4> imu_status_cal = {0, 0, 0, 0};                             //Arreglo para almacenar la calibracion del IMU 
    std::array<uint8_t, 22> calData = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //Arreglo para almacenar la calibracion
    //Constructor
    deviceBNO055(int address_I2C);
    // Metodo para leer la calibracion del sensor
    void read_calibration_status(void);
    // Metodo para leer el offset de 22 registros
    void read_Data_Offset(void);
    // Metodo de actualizacion del array calData
    void update_Calibration(const uint8_t* newCalData, size_t len);
    // Metodo para establecer el modo
    void set_configuration_IMU(void);
    // Metodo para la lectura de datos por medio del imu
    void readDataOuput(uint8_t DataOuput, uint16_t factorConData, uint8_t numberAxis);
};

//Encabeceros de funciones
void write_I2C(uint8_t address_I2C, uint8_t add_Register, uint8_t value);
uint8_t read_single_I2C(uint8_t address_I2C, uint8_t add_Register);

#endif