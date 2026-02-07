#include "lds.h"

LD_sensor::LD_sensor(int addi2c)
{
    address_I2C = addi2c;
}
void LD_sensor::initSensor(int pin_XSHUT, int timeSampling)
{
    digitalWrite(pin_XSHUT, HIGH);
    delay(10);
    sensor_VL53L0X.setTimeout(500); //Espera por la respuesta del sensor 
    if (!sensor_VL53L0X.init()){ Serial.println("Failed to detect and initialize sensor!"); } //Inicializa sensor
    else{Serial.println("Successfull initialize VL53L0X!");}
    sensor_VL53L0X.setAddress(address_I2C);  // Assign new address
    sensor_VL53L0X.setMeasurementTimingBudget(timeSampling);  //Se configura el tiempo de medición (en microsegundos)
}