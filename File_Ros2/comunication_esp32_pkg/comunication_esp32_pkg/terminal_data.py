import rclpy
from rclpy.node import Node
from interfaces_tutorial.srv import DataImu
import paho.mqtt.client as mqtt

class ClientAsync(Node):
    def __init__(self):
        super().__init__('client')
        #Definir cliente
        self.client = self.create_client(DataImu, "sensor_data")
        #Se establece una condificion, se espera a que el servicio se ponga disponible
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        #Definimos request
        self.request = DataImu.Request()

    def send_request(self, command):
        #Se monta el comando
        self.request.command = command
        #Al llamar el cliente se manda el request
        self.future = self.client.call_async(self.request)
        #Poner a correr el nodo mientras se complete la peticion
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()
    
#-----------------Definicion de funciones---------------
def loop_terminal(client_ROS2):
    #Variables de la funcion
    dicc_unit = {"Dist": "mm", "Tem" : "C°", "Lig": "lx", "Tbme" : "C°", 
                "PAtm" : "hPa" ,"Hum" : "%","Acc": "m/s^2", "Mag": "uT", "Gyr": "dps", 
                 "Eul": "degree", "Qua": "u", "Lia": "m/s^2", "Grv": "m/s^2"}
    #Cliclo while de ejecucion
    while True:
        #Tomamos el comando
        command_str = input("Enter command to send to ESP32: ")
        #correr el nodo miestrar se espera por la respuesta
        command = [ord(c) for c in command_str]
        response = client_ROS2.send_request(command)
        #Verificamos si se obtuvo una respuesta validad
        type_str = "".join(chr(i) for i in response.type)
        if type_str == "invalid":
            print("Command no valid")
        elif type_str == "timeout":
            print("Timeout waiting for MQTT respose ")          
        else:
            data_1_str = "".join(chr(i) for i in response.data_1)
            data_2_str = "".join(chr(i) for i in response.data_2)
            data_3_str = "".join(chr(i) for i in response.data_3)
            data_4_str = "".join(chr(i) for i in response.data_4)
            unit = dicc_unit[type_str]
            print(f"{type_str}: ({data_1_str}, {data_2_str}, {data_3_str}, {data_4_str}) {unit}")

def main(args=None):
    #---------------nodo------------------
    # Inicializar el nodo
    rclpy.init(args=args)
    #Crear instancia de cliente
    client_ROS2 = ClientAsync()
    #Ejecucion de funcion de terminal
    loop_terminal(client_ROS2)
    # Finalizar nodo
    client_ROS2.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
