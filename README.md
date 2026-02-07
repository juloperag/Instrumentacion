# Rover Instrumentation System 🚀

Este proyecto consiste en el sistema de instrumentación y control para un Rover, basado en el microcontrolador **ESP32**. El sistema permite la adquisición de datos de múltiples sensores y la comunicación bidireccional con el usuario a través de protocolos Serial y MQTT, integrándose finalmente con un ecosistema **ROS2**.

## 🛠 Tecnologías y Herramientas
* **Hardware:** ESP32.
* **Entorno de Desarrollo:** Visual Studio Code + PlatformIO.
* **Framework:** Arduino / ESP-IDF.
* **Comunicación:** Serial, MQTT (vía Wi-Fi).
* **Middleware:** ROS2 (Robot Operating System 2).

## 📂 Estructura del Proyecto

El repositorio está organizado de la siguiente manera:

* **`Mains/`**: Contiene los algoritmos principales de ejecución. Se incluyen dos modos de operación para la interacción entre el usuario y el ESP32:
    * `Serial`: Comunicación directa por cable.
    * `MQTT`: Comunicación inalámbrica para telemetría remota.
* **`src/`**: Carpeta de controladores (drivers) de bajo nivel. Incluye archivos de cabecera (`.h`) y de implementación (`.cpp`) para los siguientes sensores:
    * **VL53L0X**: Sensor de distancia láser (ToF).
    * **BH1750**: Sensor de luminosidad (Lux).
    * **BME280**: Sensor de presión, humedad y temperatura.
    * **BNO055**: IMU (Unidad de Medición Inercial) de 9 ejes.
* **`File_Ros2/`**: Contiene la integración con ROS2.
    * Paquete de ROS2 para la suscripción y publicación de datos vía MQTT.
    * Definición de **Interfaces** personalizadas para el envío de mensajes entre nodos.

---

## 🛰 Integración con ROS2
La arquitectura está diseñada para incorporar estos elementos en un espacio de trabajo general de ROS2. Se debe modificar el **nodo terminal**, el cual actúa como interfaz de usuario para enviar comandos específicos y visualizar la telemetría procesada.



---

## 🕹 Comandos y Telemetría

El sistema responde a comandos específicos enviados desde el terminal de ROS2. A continuación se detallan los comandos disponibles y las unidades de medida:

| Comando | Descripción | Unidad |
| :--- | :--- | :--- |
| `dist` | Distancia medida por el sensor ToF | mm |
| `tem` | Información general de temperatura | °C |
| `lig` | Iluminación ambiental | lux |
| `tbme` | Temperatura específica del sensor BME280 | °C |
| `patm` | Presión atmosférica | hPa |
| `hum` | Humedad relativa | % |
| `acc` | Aceleración lineal (Ejes X, Y, Z) | $m/s^2$ |
| `mag` | Campo magnético medido por magnetómetro | $\mu T$ |
| `gvr` | Velocidad angular (Giroscopio) | dps |
| `el` | Orientación en ángulos de Euler (roll, pitch, yaw) | Grados |
| `qua` | Orientación en cuaterniones (qw, qx, qy, qz) | Adimensional |
| `lia` | Aceleración lineal pura | $m/s^2$ |
| `grv` | Vector de gravedad estimado | $m/s^2$ |

### Formato de Salida
Para mantener la consistencia en el parseo de datos dentro de ROS2, toda la información recibida sigue el siguiente formato de cadena:

`{type_str}: ({data_1_str}, {data_2_str}, {data_3_str}, {data_4_str}) {unit}`

**Ejemplos:**
* `Dist: (25, -, -, -) mm`
* `Acc: (-0.25, 0.01, 9.78, -) m/s^2`

---