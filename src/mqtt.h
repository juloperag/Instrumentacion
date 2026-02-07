#ifndef _MQTT_H_
#define _MQTT_H_

#include <Arduino.h>
#include <Wifi.h>
#include <PubSubClient.h>

class Mqtt_Wifi{
public:
    //Actributos
    WiFiClient esp32Client;      // Instancia para la comunicación por WiFi
    PubSubClient mqttClient;     // Instancia para la comunicación MQTT con el cliente de ESP32
    bool flag_command = false;   // Bandera que indica que hay un comando disponible
    String command;              // Bandera que guarda comando 
    const char* ssid;            // SSID y Password de la red WiFi
    const char* password;     
    const char* mqtt_server;     // Dirección del MQTT Broker (IP)
    int port;
    // Constructor de la clase que inicializa los valores necesarios
    Mqtt_Wifi(const char* wifi_ssid, const char* wifi_password, const char* broker, int mqtt_port);
    // Método para conectar a WiFi 
    void connectWifi(void);
    //Metodo para reconectar al MQTT
    void reconnectMQTT(void);
    //Metodo para llamar a callback
    void callback(char* topic, byte* message, unsigned int length);
};

#endif