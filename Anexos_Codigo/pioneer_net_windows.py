import socket  # Biblioteca para la comunicación de red (TCP/IP) con el contenedor Docker
import json    # Biblioteca para codificar y decodificar datos en formato JSON (telemetría y velocidades)
import math    # Biblioteca matemática para operaciones como el cálculo del seno (sin) y arcotangente (atan2)
from controller import Supervisor  # Clase de Webots que permite controlar y modificar el entorno de simulación

# Inicialización del robot en modo Supervisor (permite interactuar con el mundo de Webots, no solo con el robot)
robot = Supervisor()

# Obtiene el paso de tiempo básico del mundo de simulación (en milisegundos) necesario para sincronizar el bucle
timestep = int(robot.getBasicTimeStep())

# Obtiene la referencia al nodo del propio robot en la simulación (para leer su posición y orientación)
robot_node = robot.getSelf()


# ==============================================================================
# --- GENERADOR DE PUNTOS MATEMÁTICOS EN EL SUELO (TRAYECTORIA) ---
# ==============================================================================
try:
    print("Generando puntos guía para la trayectoria...")
    
    # Intenta buscar el nodo de la arena utilizando su nombre de definición DEF en Webots
    arena_node = robot.getFromDef("RectangleArena")
    if arena_node is None:
        # Si el nodo no tiene el DEF "RectangleArena", accede por índice al cuarto nodo hijo de la raíz de la escena
        arena_node = robot.getRoot().getField("children").getMFNode(3) 
    
    # Obtiene el campo general "children" de la raíz de la simulación para poder inyectar nuevos objetos visuales
    children_field = robot.getRoot().getField("children")
    
    # Genera 50 puntos distribuidos en el eje X desde -5 metros hasta 4.8 metros (pasos de 0.2m)
    for i in range(-25, 25):
        x_ponto = i * 0.2            # Coordenada X del punto actual
        y_ponto = math.sin(x_ponto)  # Coordenada Y basada en una función senoidal (genera una trayectoria en S)
        
        # Define una cadena en formato VRML para crear un objeto visual dinámicamente en Webots:
        # Se crea una transformación para posicionar una pequeña esfera roja flotando a 1 cm (0.01) sobre el suelo.
        vrml_sphere = f'''
        Transform {{
          translation {x_ponto} {y_ponto} 0.01
          children [
            Shape {{
              appearance PBRAppearance {{
                baseColor 1 0 0  # Color Rojo (RGB: 1, 0, 0)
                roughness 1      # Superficie opaca/rugosa sin brillos
                metalness 0      # Material no metálico
              }}
              geometry Sphere {{
                radius 0.02      # Radio de la esfera de 2 centímetros
              }}
            }}
          ]
        }}
        '''
        # Inyecta el nodo VRML creado al final de la lista de hijos del mundo (-1 indica la última posición)
        children_field.importMFNodeFromString(-1, vrml_sphere)
        
    print("¡Puntos generados con éxito!")
except Exception as e:
    # Captura cualquier error en caso de que la inyección de nodos falle para evitar que el controlador se detenga
    print(f"Nota: No se pudieron inyectar los puntos automáticamente: {e}")


# ==============================================================================
# --- CONFIGURACIÓN DE MOTORES ---
# ==============================================================================
# Se crea una lista con los objetos de los 4 motores del robot usando los nombres asignados en el archivo .wbt
motores = [
    robot.getDevice('front left wheel'), robot.getDevice('front right wheel'),
    robot.getDevice('back left wheel'), robot.getDevice('back right wheel')
]

for m in motores:
    # Configura los motores en modo de control de velocidad (poniendo la posición objetivo en infinito)
    m.setPosition(float('inf'))
    # Inicializa los motores en reposo (velocidad 0.0 rad/s)
    m.setVelocity(0.0)


# ==============================================================================
# --- CONFIGURACIÓN DEL SERVIDOR SOCKET PARA DOCKER ---
# ==============================================================================
# Crea un socket TCP (SOCK_STREAM) utilizando el protocolo IPv4 (AF_INET)
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Enlaza el socket a todas las interfaces de red disponibles ('0.0.0.0') en el puerto 1234
server.bind(('0.0.0.0', 1234))

# Pone al servidor en modo de escucha, permitiendo un máximo de 1 conexión en espera
server.listen(1)
print("Esperando conexión de ROS desde Docker en el puerto 1234...")

# El script se bloquea aquí hasta que el nodo de ROS (u otro cliente) se conecte
conn, addr = server.accept()
print(f"¡ROS Conectado desde {addr}!")


# ==============================================================================
# --- BUCLE PRINCIPAL DE SIMULACIÓN ---
# ==============================================================================
# Ejecuta el bucle mientras Webots continúe la simulación (robot.step devuelve -1 al detenerse)
while robot.step(timestep) != -1:
    try:
        # Recibe hasta 1024 bytes del socket, los decodifica a texto UTF-8 y elimina espacios/saltos de línea extras
        data = conn.recv(1024).decode('utf-8').strip()
        
        # Si no se recibieron datos (por ejemplo, buffer vacío), continúa con el siguiente paso de simulación
        if not data:
            continue
            
        # CONDICIÓN 1: El cliente solicita información sobre el estado del robot
        if "request" in data:
            # Obtiene la posición absoluta de la matriz tridimensional del robot [X, Y, Z]
            pos = robot_node.getPosition()
            
            # Obtiene la matriz de orientación 3x3 (rotación) del robot en el mundo
            rot = robot_node.getOrientation()
            
            # Calcula el ángulo Yaw (theta) en el plano XY usando componentes de la matriz de rotación
            theta = math.atan2(rot[3], rot[0])
            
            # Estructura el diccionario con la telemetría actual (X, Y, Orientación)
            telemetry = {"x": pos[0], "y": pos[1], "theta": theta}
            
            # Convierte el diccionario a un string JSON, añade un salto de línea explícito y lo envía por el socket
            conn.sendall((json.dumps(telemetry) + "\n").encode('utf-8'))
            
        # CONDICIÓN 2: El cliente envía comandos de velocidad para los motores
        else:
            # Deserializa el string JSON recibido para convertirlo en un diccionario de Python
            v = json.loads(data)
            
            # Asigna las velocidades correspondientes extraídas del JSON a cada motor.
            # Se usa .get("nombre", 0.0) para asegurar un valor por defecto de cero si la clave no viene en el JSON.
            motores[0].setVelocity(v.get("front_left", 0.0))
            motores[1].setVelocity(v.get("front_right", 0.0))
            motores[2].setVelocity(v.get("back_left", 0.0))
            motores[3].setVelocity(v.get("back_right", 0.0))
            
    except Exception as e:
        # Si ocurre un error en la conexión o en el parseo del JSON, se imprime y se rompe el bucle principal
        print("Error de lectura o desconexión:", e)
        break

# Cierra formalmente la conexión del socket al finalizar la simulación
conn.close()
