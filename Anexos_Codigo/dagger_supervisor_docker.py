#!/usr/bin/env python3
import rospy
import math
import socket
import json
import csv  # Librería para gestionar la persistencia y escritura del archivo de datos (.csv)
from geometry_msgs.msg import Twist     # Mensaje estándar de ROS para velocidades lineales y angulares
from nav_msgs.msg import Odometry       # Mensaje estándar de ROS para estimación de posición y velocidad (Odometría)
from geometry_msgs.msg import Quaternion # Mensaje estándar de ROS para representar rotaciones en espacios 3D
import tf  # Biblioteca de ROS para gestionar y publicar transformaciones entre sistemas de coordenadas

# --- PARÁMETROS CONFIGURABLES ---
# Límite tolerado para el Error Cuadrático Medio posicional antes de que el experto tome el control total
UMBRAL_ERROR_MSE = 0.25  
# Frecuencia de ejecución del bucle de control (20 ciclos por segundo = cada 50ms)
FRECUENCIA_HZ = 20

class DaggerSupervisorBridge:
    def __init__(self):
        # Inicializa el nodo de ROS con un identificador único en el sistema
        rospy.init_node('dagger_supervisor_node', anonymous=True)
        
        # Configura el publicador para la odometría en el tópico '/odom'
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Instancia la herramienta para regular el tiempo del bucle principal
        self.rate = rospy.Rate(FRECUENCIA_HZ)
        
        # Emisor de transformaciones fijas/móviles para el árbol de coordenadas de ROS (TF)
        self.br = tf.TransformBroadcaster()

        # --- CONFIGURACIÓN DE RED HACIA WINDOWS (WSL) ---
        # Dirección IP del host (máquina Windows externa que ejecuta Webots)
        self.host = "172.28.96.1"  
        # Puerto TCP configurado en el servidor del controlador de Webots
        self.port = 1234
        # Creación del socket TCP bajo la familia de protocolos IPv4
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # --- INFRAESTRUCTURA DEL DATASET ---
        # Lista en memoria RAM que acumulará las tuplas [Estado, Acción_Experto] durante la simulación
        self.dataset_memoria = []
        # Ruta absoluta dentro del contenedor Docker para exportar el archivo del dataset final
        self.ruta_archivo = "/root/catkin_ws/src/sim2real_dagger/src/gym_tx90/scripts/dataset_dagger.csv"
        
        # Inyección de compatibilidad para evitar fallos de métodos obsoletos en ciertas versiones de ROS Noetic
        rospy.iteritems = lambda d, **kw: d.items(**kw) 

    def conectar_webots(self):
        """ Establece un canal de comunicación bidireccional TCP/IP con el simulador """
        rospy.loginfo(f"Intentando conectar con Webots en {self.host}:{self.port}...")
        try:
            self.sock.connect((self.host, self.port))
            rospy.loginfo("¡Conexión establecida con éxito con Webots (Windows)!")
            return True
        except Exception as e:
            rospy.logerr(f"No se pudo conectar a Webots: {e}")
            return False

    def obtener_trayectoria_ideal(self, x):
        """ 
        Calcula el estado deseado basándose en la posición X actual del robot.
        - Trayectoria: y = sin(x)
        - Orientación (derivada): dy/dx = cos(x) -> theta = atan2(cos(x), 1)
        """
        y_ideal = math.sin(x)
        theta_ideal = math.atan2(math.cos(x), 1.0)
        return y_ideal, theta_ideal

    def controlador_experto_mpc_backstepping(self, e_y, e_theta):
        """ 
        Controlador analítico robusto que actúa como 'Oráculo/Experto'.
        Calcula la velocidad angular de corrección basándose en los errores de desfase lateral y angular.
        """
        cmd = Twist()
        cmd.linear.x = 0.3  # Mantiene una velocidad lineal de avance constante (0.3 m/s)
        # Ley de control proporcional para corregir la orientación y el desvío lateral
        cmd.angular.z = 2.0 * e_y + 1.5 * e_theta  
        return cmd

    def guardar_dataset_a_disco(self):
        """ Vuelca los datos recolectados en la memoria RAM hacia un archivo físico CSV """
        if len(self.dataset_memoria) == 0:
            rospy.loginfo("No se recolectaron datos en esta sesión.")
            return
            
        rospy.loginfo(f"Escribiendo {len(self.dataset_memoria)} muestras en el disco...")
        try:
            with open(self.ruta_archivo, mode='w', newline='') as f:
                escritor = csv.writer(f)
                # Define las columnas del archivo: Entradas (Errores) -> Salidas Esperadas (Acciones del Experto)
                escritor.writerow(['error_y', 'error_theta', 'v_linear_experto', 'v_angular_experto'])
                # Escribe en bloque todas las filas acumuladas en la lista de memoria
                escritor.writerows(self.dataset_memoria)
            rospy.loginfo(f"¡Dataset guardado con éxito en: {self.ruta_archivo}!")
        except Exception as e:
            rospy.logerr(f"Error al escribir el archivo CSV: {e}")

    def run(self):
        # Verifica e interrumpe la ejecución si no hay enlace activo de red con Webots
        if not self.conectar_webots():
            return

        rospy.loginfo("Supervisor DAgger inicializado correctamente. Esperando datos...")

        # Bucle de control activo mientras el nodo maestro de ROS (Master) no ordene apagarse
        while not rospy.is_shutdown():
            try:
                # 1. SOLICITAR TELEMETRÍA A WEBOTS
                # Envía una petición serializada en formato JSON requiriendo datos de posición
                request = {"request": "get_telemetry"}
                self.sock.sendall((json.dumps(request) + "\n").encode('utf-8'))
                
                # 2. LEER RESPUESTA DEL SOCKET
                # Crea una interfaz de archivo virtual sobre el socket para leer cómodamente línea por línea
                fp = self.sock.makefile()
                data = fp.readline().strip()
                
                if not data:
                    break # Si la línea está vacía, denota una desconexión abrupta del servidor externo
                
                # Deserializa la cadena JSON para extraer las coordenadas reales del robot en el simulador
                telemetry = json.loads(data)
                x_actual = telemetry.get("x", 0.0)
                y_actual = telemetry.get("y", 0.0)
                theta_actual = telemetry.get("theta", 0.0)

                # 3. PUBLICAR EN ROS (ODOM Y TF)
                odom = Odometry()
                odom.header.stamp = rospy.Time.now()  # Registra la marca de tiempo de sincronización de ROS
                odom.header.frame_id = "odom"         # Sistema de coordenadas global inercial de referencia
                odom.child_frame_id = "base_link"    # Sistema de coordenadas local anclado al propio robot
                odom.pose.pose.position.x = x_actual
                odom.pose.pose.position.y = y_actual
                
                # Convierte el ángulo en radianes (Yaw) a un formato de Cuaternión (X, Y, Z, W) requerido por ROS
                q = tf.transformations.quaternion_from_euler(0, 0, theta_actual)
                odom.pose.pose.orientation = Quaternion(*q)
                
                # Publica el mensaje completo de odometría en el ecosistema ROS
                self.pub_odom.publish(odom)
                # Difunde dinámicamente la transformación espacial (TF) entre los sistemas de coordenadas
                self.br.sendTransform((x_actual, y_actual, 0.0), q, rospy.Time.now(), "base_link", "odom")

                # 4. CÁLCULO DE ERRORES GEOMÉTRICOS
                # Obtiene la posición e inclinación teóricas que debería tener el robot en su coordenada X actual
                y_ideal, theta_ideal = self.obtener_trayectoria_ideal(x_actual)
                error_y = y_ideal - y_actual
                error_theta = theta_ideal - theta_actual
                
                # Normaliza el error angular para acotarlo estrictamente en el rango [-pi, pi]
                error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))
                
                # Calcula el Error Cuadrático Medio posicional (MSE) instantáneo
                mse_actual = (error_y ** 2) / 2.0
                
                # 5. CÁLCULO DEL EXPERTO (Estrategia fundamental de DAgger)
                # Aunque el aprendiz maneje el robot, el experto SIEMPRE calcula la acción correcta para esa situación
                cmd_experto = self.controlador_experto_mpc_backstepping(error_y, error_theta)
                
                # RECOLECCIÓN DATASET: Almacena el estado actual y el consejo del experto
                self.dataset_memoria.append([error_y, error_theta, cmd_experto.linear.x, cmd_experto.angular.z])

                # 6. SELECCIÓN DE POLÍTICA DINÁMICA (Estrategia de Intervención DAgger)
                cmd_final = Twist()
                if mse_actual < UMBRAL_ERROR_MSE:
                    # MODO APRENDIZ: El modelo en desarrollo toma el mando (aquí emulado mediante un control simple)
                    # throttle limita los prints para no saturar la consola (imprime cada 2 segundos)
                    rospy.loginfo_throttle(2, f"Modo Aprendiz activo. MSE tolerable: {mse_actual:.4f}")
                    cmd_final.linear.x = 0.25
                    cmd_final.angular.z = 0.6 * error_y
                else:
                    # MODO EXPERTO: Se detectó Covariate Shift (el aprendiz desvió el robot a zonas desconocidas).
                    # El experto toma el control del actuador de inmediato para salvar y corregir la trayectoria.
                    rospy.logwarn_throttle(1, f"¡Alerta Covariate Shift! MSE Crítico: {mse_actual:.4f}. Experto activo.")
                    cmd_final = cmd_experto

                # 7. CONVERSIÓN CINEMÁTICA DIFERENCIAL
                # Convierte las velocidades abstractas (lineal/angular) a velocidades angulares independientes 
                # para las ruedas izquierdas y derechas. Distancia entre ejes asumida = 0.8 metros (0.4m por radio).
                v_izquierda = cmd_final.linear.x - (cmd_final.angular.z * 0.4)
                v_derecha = cmd_final.linear.x + (cmd_final.angular.z * 0.4)

                # 8. ENVIAR COMANDOS RESULTANTES A WEBOTS
                # Empaqueta y codifica los datos en formato JSON para transmitirlos por red
                command_data = {
                    "front_left": v_izquierda, "front_right": v_derecha,
                    "back_left": v_izquierda, "back_right": v_derecha
                }
                self.sock.sendall((json.dumps(command_data) + "\n").encode('utf-8'))

            except Exception as e:
                rospy.logerr(f"Error en el puente de red: {e}")
                break
            
            # Sincroniza la frecuencia de ejecución del bucle con los 20Hz configurados al inicio
            self.rate.sleep()
            
        # Cierre del descriptor del socket una vez finalizada la conexión o ante interrupción
        self.sock.close()
        # Vuelca la información recolectada de la memoria al disco antes de cerrar por completo la app
        self.guardar_dataset_a_disco()


if __name__ == '__main__':
    bridge = DaggerSupervisorBridge()
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        # Captura la excepción de parada limpia cuando el usuario presiona Ctrl+C o ejecuta rosnode kill
        pass
    finally:
        # Bloque de seguridad que garantiza el guardado definitivo del dataset ante cierres inesperados
        bridge.guardar_dataset_a_disco()
