import rclpy 
from rclpy.node import Node
from interfaces_tutorial.srv import DataImu
import paho.mqtt.client as mqtt
import time

#------------Definicion de clases----------
class Service_ROS2(Node):
    def __init__(self, client_MQTT):
        super().__init__("service")
        #Definicion de varibales
        self.client_MQTT = client_MQTT
        self.data = None
        self.flag_msg = False
        #Creacion de un servicio
        self.service_ROS2 = self.create_service(DataImu,"sensor_data", self.execute_comunication_ESP32)
        # Pass self to MQTT client
        client_MQTT.service_instance = self

    def execute_comunication_ESP32(self, request, response):
        #Convertir a string
        command_str = "".join(chr(i) for i in request.command)
        print(f"Sending command: {command_str}")
        # Envio de comando por MQTT
        try:
            self.client_MQTT.publish("esp32/command", str(command_str))
        except KeyboardInterrupt:
            print("Error of Connetion")
            self.client_MQTT.loop_stop()
            self.client_MQTT.disconnect() 
        # Wait for message with timeout
        timeout_seconds = 5
        start_time = time.time()
        timeout_occurred = False
        while not self.flag_msg:
            if time.time() - start_time > timeout_seconds:
                timeout_occurred = True
                break
            time.sleep(0.01)  # avoid busy waiting
        if timeout_occurred:
            print("Timeout waiting for MQTT response")
            # You can set a default or error response here if needed
            response.type = [ord(c) for c in "timeout"]
            response.data_1 = []
            response.data_2 = []
            response.data_3 = []
            response.data_4 = []
            self.flag_msg = False
            return response
        # Use received data
        response.type = [ord(c) for c in self.data[0]]
        response.data_1 = [ord(c) for c in self.data[1]] 
        response.data_2 = [ord(c) for c in self.data[2]] 
        response.data_3 = [ord(c) for c in self.data[3]] 
        response.data_4 = [ord(c) for c in self.data[4]] 
        self.flag_msg = False  # Reset flag
        return response  

        
#-----------------Definicion de funciones---------------
#------------Funciones MQTT-------------
def on_connect(client, userdata, flag, rc):
    if rc == 0:
        print("Successfully connection at Broker MQTT")
        client.subscribe( "esp32/sensor/temp")
        client.subscribe( "esp32/sensor/bme")
        client.subscribe( "esp32/sensor/dist")
        client.subscribe( "esp32/sensor/imu")
        client.subscribe( "esp32/sensor/lig")
    else:
        print("There is error in connetion")

def on_message(client_MQTT, userdata, msg):
    # Recepcion de datos
    service_ROS2 = getattr(client_MQTT, "service_instance", None)
    if service_ROS2:
        service_ROS2.data = (msg.payload.decode()).split(",")
        service_ROS2.flag_msg = True

def main(args=None):
    #Parametros de conexion
    broker = "127.0.0.1"
    port = 1883 
    #----------------MQTT-----------------
    # Crear un cliente MQTT
    client_MQTT = mqtt.Client()
    # Asignamos las funciones callback
    client_MQTT.on_connect = on_connect
    client_MQTT.on_message = on_message
    # Conectar el broker
    client_MQTT.connect(broker, port, 60)
    #Dejar la conexion en loop
    client_MQTT.loop_start()
    #---------------nodo------------------
    # Inicializar el nodo
    rclpy.init(args=args)
    #Crear instancia para publicar 
    servROS2 = Service_ROS2(client_MQTT)
    rclpy.spin(servROS2)
    #----------------------
    # Finalizar MQTT
    client_MQTT.loop_stop()  
    # Finalizar nodo
    servROS2.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()