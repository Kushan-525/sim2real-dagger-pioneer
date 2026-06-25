# Sim2Real Trayectory Tracking usando DAgger y ROS Noetic

Este repositorio contiene la implementación de un sistema de **Aprendizaje por Imitación Interactivo** utilizando el algoritmo **DAgger (Dataset Aggregation)** para el control y seguimiento de trayectorias sinusoidales con un robot móvil **Pioneer 3-AT**.

## 📊 Arquitectura del Sistema

El proyecto utiliza una arquitectura **Sim2Real Bridge** síncrona dividida en dos entornos:
1. **Entorno Virtual (Windows):** Simulador **Webots R2025a** que gestiona la física 3D, el modelo del robot y la proyección visual de la ruta.
2. **Cerebro Algorítmico (Debian/Docker):** Contenedor con **ROS Noetic** encargado de la lógica de control, cálculo de errores ($MSE$), publicación de odometría (`/odom`, `tf`) y agregación del dataset.

La comunicación se realiza mediante **Sockets TCP/IP** crudos a través del puerto `1234` usando tramas estructuradas en **JSON** con delimitadores de salto de línea (`\n`).

## 🧠 Algoritmo DAgger
El sistema evalúa constantemente el Error Cuadrático Medio ($MSE$) del desvío lateral:
* **Modo Aprendiz:** Controla el robot mientras el error sea tolerable ($MSE < 0.25$).
* **Modo Experto:** Un controlador cinemático basado en **Backstepping / MPC** geométrico toma el control ante el fenómeno de *Covariate Shift* ($MSE \ge 0.25$), corrigiendo el rumbo y guardando las muestras de recuperación en un archivo `dataset_dagger.csv` para su posterior entrenamiento supervisado.

## 🚀 Instrucciones de Ejecución

### 1. En Webots (Windows)
* Cargar el mundo de la arena.
* Asegurar que el controlador del Pioneer apunte a `pioneer_net.py`.
* Presionar **Reset** y **Play**.

### 2. En el Contenedor Docker (ROS)
```bash
# Iniciar el Master de ROS
roscore &

# Configurar el entorno de trabajo
cd /root/catkin_ws
source devel/setup.bash

# Lanzar el supervisor DAgger
rosrun gym_tx90 dagger_supervisor.py
