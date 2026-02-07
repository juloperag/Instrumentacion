#ifndef _LDS_H_
#define _LDS_H_

#include "VL53L0X.h"

class LD_sensor{
public:
    //Atributos
    VL53L0X sensor_VL53L0X;  //Instancia del sensor VL53L0X para la medicion de distancias
    int address_I2C;         //Direccion del I2C del sensor
    //Contrutor de la clase
    LD_sensor(int addi2c);
    //Metodo para el inicializar el sensor
    void initSensor(int pin_XSHUT, int timeSampling);
};

#endif