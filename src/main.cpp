#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include "mqtt.h"
#include "imu.h"

//Definicion de instancia para la comunicacion
Mqtt_Wifi mqttwifi("CLARO-6B07", "Cl4r0@FE6B07", "192.168.1.7", 1883); //"CLARO-6B07", "Cl4r0@FE6B07", "192.168.1.2", 1883 //test.
//Definir instancia para el sensor IMU
deviceBNO055 imu;
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
  //Configuracion led de estado
  pinMode(PIN_LED, OUTPUT);
  //Configuracion puerto serial
  Serial.begin(115200);
  //-----------------Configuracion I2C--------------------
  //Inicializamos I2C con los pines especificos
  Wire.setClock(100000);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);  // Dale tiempo al sensor
  Serial.write("Successful configuration I2C \n");
  //--------------Configuracion Comunicacion WIFI-----------------
  //Inicializacion conexion wifi
  Serial.write("Init  connetion WIFI \n");
  mqttwifi.initWifi();
  //Inicializar conexion con MQTT
  mqttwifi.mqttClient.setServer(mqttwifi.mqtt_server,mqttwifi.port);
  mqttwifi.mqttClient.setCallback(mqtt_callback_bridge);
  //---------------Configuracion sensores--------------------
  //-------IMU-----------
  Serial.write("Init configuration IMU \n");
  imu.config_IMU(OPR_MODE, CONFIGMODE); //Iniciar Modo de configuracion
  imu.config_IMU(PWR_MODE, NORMALMODE); //Iniciar Modo de normal
  imu.config_IMU(OPR_MODE, NDOF);       //Establecer modo NDOF y fin de configuracion
  Serial.write("Successful configuration IMU \n");
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
  Serial.write(cmd);

	//comandos
  if (strcmp(cmd, "acc") == 0)
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

