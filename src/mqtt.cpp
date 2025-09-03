
#include "mqtt.h"

Mqtt_Wifi::Mqtt_Wifi(const char* wifi_ssid, const char* wifi_password, const char* broker, int mqtt_port)
: mqttClient(esp32Client) {  // Inicializa mqttClient con esp32Client
    ssid = wifi_ssid;
    password = wifi_password;
    mqtt_server = broker;
    port = mqtt_port;
}
//Método para conectar a WiFi 
void Mqtt_Wifi::initWifi(void){
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    //Intentar conectar al wifis
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(500);
    }
    //Conexion exitosa
    Serial.print("\n Connection to Wifi \n");
}

//Metodo para reconectar al MQTT
void Mqtt_Wifi::reconnectMQTT(void){
    while(!mqttClient.connected())
        {
            Serial.print("Trying connection MQTT.... \n");
            //Si se conecta sactisfatoriamente a MQTT se suscribe a los topic especificos
            if(mqttClient.connect("arduinoClient"))
            {
            Serial.println("Connection to MQTT ");
            mqttClient.subscribe("esp32/command");
            }
            //De lo contrario lo vuelve a intentar
            else
            { 
            Serial.print("Fallo, rc= ");
            Serial.print(mqttClient.state());
            Serial.println(" Try again in 5 seg ");
            delay(5000);
            }
        }
}   
//Metodo para llamar a callbak
void Mqtt_Wifi::callback(char* topic, byte* message, unsigned int length){
    //Variables auxiliares
    String messageTemp;               //Mensaje
    unsigned int index_command = 0;   //contador de indice de comando
    //Envio por consola 
    Serial.print("Msg receive in topic: ");
    Serial.println(topic);

    for (int i = 0; i < length; i++)
    {
        messageTemp += (char) message[i];
    }
    //Acciones a realizar con el mensaje
    if(String(topic)=="esp32/command")
    {
        while(true)
        {
            if(messageTemp[index_command] == '@'){ 
                    //Proceso para guardar comando
                    messageTemp[index_command] = '\0';
                    flag_command = true;
                    command = messageTemp;
                    break;
                }
            else if(messageTemp[index_command] == '\0'){ break; }
            //Aumentamos la variable
            index_command++;
        }
    }
}


