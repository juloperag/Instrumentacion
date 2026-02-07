#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "mqtt.h"
#include "imu.h"
#include "lds.h"
#include <EEPROM.h>

//Definicion de pines
#define LED_PIN 18
#define SDA_PIN 8
#define SCL_PIN 9
#define XSHUT1_PIN 13
#define XSHUT2_PIN 14
#define TEM_SENSOR_1_PIN 3
#define TEM_SENSOR_2_PIN 4

//Definicion de instancias 
//--------Comandos para MQTT y ping
//wsl hostname -I
//netsh interface portproxy add v4tov4 listenport=1883 listenaddress=0.0.0.0 connectport=1883 connectaddress=<IP_WSL>
//---------------------
//Mqtt_Wifi mqttwifi("Comunidad_UNMED", "wifi_med_213", "172.21.32.1", 1883); 
Mqtt_Wifi mqttwifi("CLARO-6B07", "Cl4r0@FE6B07", "192.168.1.5", 1883);
deviceBNO055 imu(BNO055_ADDR_B);
LD_sensor lds_1(0x33);
LD_sensor lds_2(0x34);
Adafruit_BME280 bme;
BH1750 lightMeter(0x23);
//Definicion de variables 
unsigned int mode_operation = 0;
//Variables del led
uint16_t time_led = 0;
uint8_t status = 0;

//Cabeceras funciones
void mqtt_callback_bridge(char* topic, byte* payload, unsigned int length);
void receptionSerial(void);
void runCommand(String prtcommand);

void setup() {
  //Configuracion de pines
  pinMode(LED_PIN, OUTPUT);
  pinMode(XSHUT1_PIN, OUTPUT);
  pinMode(XSHUT2_PIN, OUTPUT);
  //Estado inicial de pines digitales
  digitalWrite(LED_PIN, LOW);
  digitalWrite(XSHUT1_PIN, LOW);
  digitalWrite(XSHUT2_PIN, LOW);
  //Establecer resolucion de ADC
  analogReadResolution(12);
  //Configuracion puerto serial
  Serial.begin(115200);
  //-----------------Configuracion I2C--------------------
  //Inicializamos I2C con los pines especificos
  //Wire.setClock(100000);
  Wire.setClock(400000);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);  // Dale tiempo al sensor
  Serial.write("Successful configuration I2C \n");
  //--------------Configuracion Comunicacion WIFI-----------------
  //Inicializacion conexion wifi
  Serial.write("Init  connetion WIFI \n");
  mqttwifi.connectWifi();
  //Inicializar conexion con MQTT
  mqttwifi.mqttClient.setServer(mqttwifi.mqtt_server,mqttwifi.port);
  mqttwifi.mqttClient.setCallback(mqtt_callback_bridge);
  //---------------Configuracion sensores--------------------
  //--------VL53L0X---------
  //lds_1.initSensor(XSHUT1_PIN, 60000);
  //lds_2.initSensor(XSHUT2_PIN, 60000);
  //lds_1.sensor_VL53L0X.startContinuous(); //Se inicializa la Medicion del sensor de forma continua, luego por medio de un metodo se optiene la distancia
  //lds_2.sensor_VL53L0X.startContinuous();
  //-------IMU-----------
  imu.mode_operation = NDOF; //Establecemiento del modo
  loadFromEEPROM(&imu); //Cargamos los datos de calibracion de la memoria EEPROM
  imu.set_configuration_IMU();  
  //------BMP------------
  if (!bme.begin(0x76)) { Serial.println("Could not find a valid BME280");}
  else{ Serial.println("BME280 find successfully");}
  //------BH1750---------
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)){ Serial.println("BH1750 Advanced begin");} 
  else {Serial.println(F("Error initialising BH1750"));}
}

void loop(){

  //Verificacion de conexion de cliente MQTT, realizar reconexion si es necesario
  if(!mqttwifi.mqttClient.connected()){mqttwifi.reconnectMQTT();}
  //Entrada en un loop hasta que se lance el callback
  mqttwifi.mqttClient.loop();
  //Procesamiento del comando recibido
  if(mqttwifi.flag_command){ runCommand(mqttwifi.command); mqttwifi.flag_command = false;}
}

//---------------------------------Funciones-------------------------------
//Funcion auxiliar para generar puntero a la funcion callback
void mqtt_callback_bridge(char* topic, byte* payload, unsigned int length) {
    mqttwifi.callback(topic, payload, length);
}

//Funcion que ejecuta el comando ingresando
void runCommand(String prtcommand)
{
	//Variables para almacenar los elementos que entrega el comando luego de ser divididos por la funcion sscanf
	char cmd[64]= {0};
  //Variables Adicionales
  char buffermessage[100];

	//Funcion que lee la cadena de caracteres y la divide en los elementos 
	sscanf(prtcommand.c_str(), "%s ", cmd);
  
	//-------------comandos-----------------
  //-------LaserDist
  if (strcmp(cmd, "dist") == 0)
	{
    //Lectura del sensor
    uint32_t dis_1 = lds_1.sensor_VL53L0X.readRangeContinuousMillimeters();
    uint32_t dis_2 = lds_2.sensor_VL53L0X.readRangeContinuousMillimeters();
    if(lds_1.sensor_VL53L0X.timeoutOccurred() and lds_2.sensor_VL53L0X.timeoutOccurred()){Serial.write("TIMEOUT Sensor Laser \n");} 
    else{ 
      //Contruir y enviar mensaje por consola
      sprintf(buffermessage, "Distance_lds_1 = %d mm, Distance_lds_2 = %d mm \n", dis_1, dis_2);
      Serial.write(buffermessage);
      //Contruir y enviar mensaje por wifi
      sprintf(buffermessage, "Dist,%d, %d,-,-", dis_1, dis_2);    
      mqttwifi.mqttClient.publish("esp32/sensor/dist",buffermessage); //Topic: "esp32/sensor/imu"
    }
  }
  //---------Temperature
  else if (strcmp(cmd, "tem") == 0)
  {
    //Leer data output
    float temp_1 = ((analogRead(TEM_SENSOR_1_PIN)/4095)*5000); //V
    float temp_2 = ((analogRead(TEM_SENSOR_2_PIN)/4095)*5000);
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Temperature S1 = %.2f C° , Temperature S2 = %.2f C° \n", temp_1, temp_2);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Tem, %.2f ,%.2f ,-,-", temp_1, temp_2);    
    mqttwifi.mqttClient.publish("esp32/sensor/temp",buffermessage); //Topic: "esp32/sensor/imu"

  }
  //-------luminosity
  else if (strcmp(cmd, "lig") == 0)
  {
    if (lightMeter.measurementReady()){
      //Leer data output
      float lux = lightMeter.readLightLevel(); //lux
      //Contruir y enviar mensaje por consola
      sprintf(buffermessage, "Light level = %.2f lx \n", lux);
      Serial.write(buffermessage);
      //Contruir y enviar mensaje por wifi
      sprintf(buffermessage, "Lig, %.2f,-,-,-", lux);    
      mqttwifi.mqttClient.publish("esp32/sensor/lig",buffermessage); 
    }
    else{Serial.println("Forced measurement failed!");}
  }
  //--------BPM
  else if (strcmp(cmd, "tbme") == 0)
  {
    bme.takeForcedMeasurement();
    //Leer data output
    float temb = bme.readTemperature(); //C°
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Temperature bme = %.2f C° \n", temb);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Tbme, %.2f,-,-,-", temb);    
    mqttwifi.mqttClient.publish("esp32/sensor/bme",buffermessage); 
  }
  else if (strcmp(cmd, "patm") == 0)
  {
    bme.takeForcedMeasurement();
    //Leer data output
    float pre = bme.readPressure() / 100.0F; //hPa
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Pressure atmospheric = %.2f hPa \n", pre);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "PAtm, %.2f,-,-,-", pre);    
    mqttwifi.mqttClient.publish("esp32/sensor/bme", buffermessage); 
  }
  else if (strcmp(cmd, "hum") == 0)
  {
    bme.takeForcedMeasurement();
    //Leer data output
    float hum = bme.readHumidity(); //RH %
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Humidity realative = %.2f % \n", hum);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Hum, %.2f,-,-,-", hum);    
    mqttwifi.mqttClient.publish("esp32/sensor/bme", buffermessage); 
  }
  //--------IMU
  else if (strcmp(cmd, "save") == 0) {
    imu.read_Data_Offset();
    saveToEEPROM(&imu);
    Serial.println("Calibration saved to EEPROM");
  }
  else if (strcmp(cmd, "acc") == 0)
	{
    //Leer data output
    imu.readDataOuput(ACC_DATA, FACTOR_ACCEL, 3); //[m/s2]
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Acc_X = %.2f m/s2, Acc_Y = %.2f m/s2, Acc_Z = %.2f m/s2 \n", imu.listData[0], imu.listData[1], imu.listData[2]);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Acc,%.2f,%.2f,%.2f,-", imu.listData[0], imu.listData[1], imu.listData[2]);    
    mqttwifi.mqttClient.publish("esp32/sensor/imu",buffermessage); //Topic: "esp32/sensor/imu"
	}
  else if (strcmp(cmd, "mag") == 0)
	{
    //Leer data output
    imu.readDataOuput(MAG_DATA, FACTOR_MAG, 3); //[uT]
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Mag_X = %.2f uT, Mag_Y = %.2f uT, Mag_Z = %.2f uT \n", imu.listData[0], imu.listData[1], imu.listData[2]);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Mag,%.2f,%.2f,%.2f,-", imu.listData[0], imu.listData[1], imu.listData[2]);    
    mqttwifi.mqttClient.publish("esp32/sensor/imu",buffermessage); //Topic: "esp32/sensor/imu"
	}
  else if (strcmp(cmd, "gyr") == 0)
	{
    //Leer data output
    imu.readDataOuput(GYR_DATA, FACTOR_GYR, 3); //[Dps]
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Gyr_X = %.2f Dps, Gyr_Y = %.2f Dps, Gyr_Z = %.2f Dps \n", imu.listData[0], imu.listData[1], imu.listData[2]);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Gyr,%.2f,%.2f,%.2f,-", imu.listData[0], imu.listData[1], imu.listData[2]);    
    mqttwifi.mqttClient.publish("esp32/sensor/imu",buffermessage); //Topic: "esp32/sensor/imu"
	}
  else if (strcmp(cmd, "eul") == 0)
	{
    //Leer data output
    imu.readDataOuput(EUL_DATA, FACTOR_EUL, 3); //[Dps]
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Eul_Heading = %.2f degree, Eul_Roll = %.2f degree, Eul_Pitch = %.2f degree \n", imu.listData[0], imu.listData[1], imu.listData[2]);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Eul,%.2f,%.2f,%.2f,-", imu.listData[0], imu.listData[1], imu.listData[2]);    
    mqttwifi.mqttClient.publish("esp32/sensor/imu",buffermessage); //Topic: "esp32/sensor/imu"
	}
  else if (strcmp(cmd, "qua") == 0)
	{
    //Leer data output
    imu.readDataOuput(QUA_DATA, FACTOR_QUA, 4); //[Dps]
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Qua_W = %.2f u, Qua_X = %.2f u, Qua_Y = %.2f u, Qua_Z = %.2f \n", imu.listData[0], imu.listData[1], imu.listData[2], imu.listData[3]);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Qua,%.2f,%.2f,%.2f,%.2f", imu.listData[0], imu.listData[1], imu.listData[2], imu.listData[3]);    
    mqttwifi.mqttClient.publish("esp32/sensor/imu",buffermessage); //Topic: "esp32/sensor/imu"
	}
  else if (strcmp(cmd, "lia") == 0)
	{
    //Leer data output
    imu.readDataOuput(LIA_DATA, FACTOR_LIA, 3); //[Dps]
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Lia_X = %.2f m/s2, Lia_Y = %.2f m/s2, Lia_Z = %.2f m/s2 \n", imu.listData[0], imu.listData[1], imu.listData[2]);    
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Lia,%.2f,%.2f,%.2f,-", imu.listData[0], imu.listData[1], imu.listData[2]);    
    mqttwifi.mqttClient.publish("esp32/sensor/imu",buffermessage); //Topic: "esp32/sensor/imu"
	}
  else if (strcmp(cmd, "grv") == 0)
	{
    //Leer data output
    imu.readDataOuput(GRV_DATA, FACTOR_GRV, 3); //[Dps]
    //Contruir y enviar mensaje por consola
    sprintf(buffermessage, "Grv_X = %.2f m/s2, Grv_Y = %.2f m/s2, Grv_Z = %.2f m/s2 \n", imu.listData[0], imu.listData[1], imu.listData[2]);
    Serial.write(buffermessage);
    //Contruir y enviar mensaje por wifi
    sprintf(buffermessage, "Grv,%.2f,%.2f,%.2f,-", imu.listData[0], imu.listData[1], imu.listData[2]);    
    mqttwifi.mqttClient.publish("esp32/sensor/imu",buffermessage); //Topic: "esp32/sensor/imu"
	}
  else	
  {
    //Se imprime que el comando no fue valido
		Serial.write("Comando no correcto \n");
    //Envio de comando invalido
    mqttwifi.mqttClient.publish("esp32/sensor/imu","invalid,-,-,-,-"); //Topic: "esp32/sensor/imu"
	}
}

//-----------Funciones auxiliares para guardar leer en memoria EEPROM-------------------
void saveToEEPROM(deviceBNO055* imuPtr) {
  if (imuPtr == nullptr) return;  // validación
  for (int i = 0; i < 22; i++) {
    EEPROM.write(i, imuPtr->calData[i]);
  }
  EEPROM.commit();
}

void loadFromEEPROM(deviceBNO055* imuPtr) {
  if (imuPtr == nullptr) return;  // validación
  for (int i = 0; i < 22; i++) {
    imuPtr->calData[i] = EEPROM.read(i);
  }
}