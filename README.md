# TFM - Alternativas a la simulación de robots de exteriores basada en Gazebo para ROS 2

Este repositorio contiene todos los materiales desarrollados para el Trabajo Fin de Máster sobre análisis comparativo de simuladores de robótica utilizando ROS 2. El proyecto evalúa el rendimiento de diferentes simuladores (Webots, Ignition Gazebo, Isaac Sim) mediante métricas de Real-Time Factor (RTF) y consumo de recursos del sistema.

## 🎯 Objetivos del Proyecto

- Evaluar el rendimiento de simuladores alternativos a Gazebo para ROS 2
- Comparar métricas de Real-Time Factor (RTF) entre diferentes simuladores
- Analizar el consumo de recursos del sistema (CPU, GPU, RAM, Batería)
- Proporcionar guías de instalación y configuración para cada simulador
- Generar datos estadísticos comparativos para la toma de decisiones

## 🛠️ Simuladores Evaluados

- **Webots** - Simulador de código abierto desarrollado por Cyberbotics
- **Ignition Gazebo** - Nueva versión del ecosistema Gazebo
- **Isaac Sim** - Simulador fotorrealista de NVIDIA basado en Omniverse

## 📁 Estructura del Repositorio

### 🔧 [Scripts_Auxiliares/](Scripts_Auxiliares/)
Contiene los scripts desarrollados para la unión de CSV, monitorización de RTF y los archivos de lanzamiento propios de los simuladores Ignition y Webots modificados para automatizar el proceso.

#### [Scripts_Auxiliares/Scripts_CSV/](Scripts_Auxiliares/Scripts_CSV/)
Scripts para unificar y procesar las mediciones obtenidas de cada simulador:

- [`unir_mediciones_simuladores.py`](Scripts_Auxiliares/Scripts_CSV/unir_mediciones_simuladores.py) - Script principal que procesa las mediciones de todos los simuladores
- [`unir_mediciones_webots.py`](Scripts_Auxiliares/Scripts_CSV/unir_mediciones_webots.py) - Procesamiento específico para Webots
- [`unir_mediciones_ignition.py`](Scripts_Auxiliares/Scripts_CSV/unir_mediciones_ignition.py) - Procesamiento específico para Ignition Gazebo
- [`unir_mediciones_isaacsim.py`](Scripts_Auxiliares/Scripts_CSV/unir_mediciones_isaacsim.py) - Procesamiento específico para Isaac Sim
- [`powerstat_to_csv.py`](Scripts_Auxiliares/Scripts_CSV/powerstat_to_csv.py) - Powerstat.log a CSV

#### [Scripts_Auxiliares/RTF_Monitor/](Scripts_Auxiliares/RTF_Monitor/)
Herramientas para monitorización del Real-Time Factor:

- Monitor principal de RTF y recursos del sistema

#### [Scripts_Auxiliares/Scripts_Lanzamiento_Modificados/](Scripts_Auxiliares/Scripts_Lanzamiento_Modificados/)
Scripts de lanzamiento modificados para automatizar experimentos

#### Otros archivos
- [`maze.wbt`](Scripts_Auxiliares/maze.wbt) - Mundo de Webots utilizado en los experimentos

### 📏 [Scripts_Medicion/](Scripts_Medicion/)
Scripts específicos para la recolección de métricas en cada simulador:

- **[Scripts_Medicion/Ignition_Gazebo/](Scripts_Medicion/Ignition_Gazebo/)** - Scripts de medición para Ignition Gazebo
  - [`benchmark_ignition.py`](Scripts_Medicion/Ignition_Gazebo/benchmark_ignition.py) - Script principal de benchmarking automatizado
  - [`metrics_collector.py`](Scripts_Medicion/Ignition_Gazebo/metrics_collector.py) - Recolector de métricas del sistema con clases [`MetricsCollector`](Scripts_Medicion/Ignition_Gazebo/metrics_collector.py), [`RTFCollector`](Scripts_Medicion/Ignition_Gazebo/metrics_collector.py), [`CPUCollector`](Scripts_Medicion/Ignition_Gazebo/metrics_collector.py), [`RAMCollector`](Scripts_Medicion/Ignition_Gazebo/metrics_collector.py), [`GPUCollector`](Scripts_Medicion/Ignition_Gazebo/metrics_collector.py) y [`PowerCollector`](Scripts_Medicion/Ignition_Gazebo/metrics_collector.py)
  - [`navigation_controller.py`](Scripts_Medicion/Ignition_Gazebo/navigation_controller.py) - Controlador de navegación autónoma con waypoints predefinidos
  - [`simulator_launcher.py`](Scripts_Medicion/Ignition_Gazebo/simulator_launcher.py) - Lanzador del simulador con diferentes configuraciones (GPU/sin GPU, GUI/sin GUI)

- **[Scripts_Medicion/IsaacSim/](Scripts_Medicion/IsaacSim/)** - Scripts de medición para Isaac Sim
  - [`benchmark_isaac_sim.py`](Scripts_Medicion/IsaacSim/benchmark_isaac_sim.py) - Script principal de benchmarking para Isaac Sim
  - [`metrics_collector.py`](Scripts_Medicion/IsaacSim/metrics_collector.py) - Recolector de métricas específico para Isaac Sim con [`RTFCollector`](Scripts_Medicion/IsaacSim/metrics_collector.py) adaptado para el nodo `rtf_monitor`
  - [`simulator_launcher.py`](Scripts_Medicion/IsaacSim/simulator_launcher.py) - Lanzador específico para Isaac Sim con soporte para modos GUI y headless

- **[Scripts_Medicion/Webots/](Scripts_Medicion/Webots/)** - Scripts de medición para Webots
  - [`benchmark_webots.py`](Scripts_Medicion/Webots/benchmark_webots.py) - Script principal de benchmarking para Webots
  - [`metrics_collector.py`](Scripts_Medicion/Webots/metrics_collector.py) - Recolector de métricas con [`RTFCollector`](Scripts_Medicion/Webots/metrics_collector.py) específico para el nodo `webots_ros2_turtlebot`
  - [`navigation_controller.py`](Scripts_Medicion/Webots/navigation_controller.py) - Controlador de navegación adaptado para el mundo maze de Webots
  - [`simulator_launcher.py`](Scripts_Medicion/Webots/simulator_launcher.py) - Lanzador del simulador Webots con diferentes configuraciones de renderizado

#### Características Comunes de los Scripts de Medición

Todos los scripts de benchmarking (`benchmark_*.py`) comparten funcionalidades similares:

- **Argumentos de línea de comandos**: 
  - `--folder`: Carpeta de salida para los datos
  - `--mode`: Modo de ejecución del simulador
  - `--duration`: Duración de la simulación en segundos
  - `--interval`: Intervalo de muestreo de métricas
  - `--power-warmup`: Tiempo de warmup para PowerStat
  - `--no-power`: Opción para deshabilitar medición de batería

- **Métricas recolectadas**:
  - **CPU**: Porcentaje de uso, frecuencia, load average, número de procesos
  - **RAM**: Memoria total, usada, disponible, swap
  - **GPU**: Utilización, memoria, potencia, temperatura (vía nvidia-smi)
  - **RTF**: Real-Time Factor específico de cada simulador
  - **Batería**: Consumo de energía (vía PowerStat)

- **Manejo de señales**: Limpieza automática al recibir SIGINT/SIGTERM
- **Sincronización**: Coordinación entre PowerStat warmup y inicio del simulador
- **Logging**: Salida detallada del progreso del benchmark

### 📊 [Mediciones_CSV/](Mediciones_CSV/)
Datos experimentales en formato CSV organizados por simulador y escenario:

- **[Mediciones_CSV/Mediciones_Escenarios/](Mediciones_CSV/Mediciones_Escenarios/)** - Mediciones agrupadas por tipo de escenario
- **[Mediciones_CSV/Mediciones_Simulador_Webots/](Mediciones_CSV/Mediciones_Simulador_Webots/)** - Datos específicos de Webots
- **[Mediciones_CSV/Mediciones_Simulador_IgnitionGazebo/](Mediciones_CSV/Mediciones_Simulador_IgnitionGazebo/)** - Datos específicos de Ignition Gazebo
- **[Mediciones_CSV/Mediciones_Simulador_IsaacSim/](Mediciones_CSV/Mediciones_Simulador_IsaacSim/)** - Datos específicos de Isaac Sim

### 📈 [Mediciones_JASP/](Mediciones_JASP/)
Datos procesados y formateados para análisis estadístico con JASP:

- **[Mediciones_JASP/Mediciones_Escenarios/](Mediciones_JASP/Mediciones_Escenarios/)** - Datos por escenario para JASP
- **[Mediciones_JASP/Mediciones_Simulador_Webots/](Mediciones_JASP/Mediciones_Simulador_Webots/)** - Datos de Webots para JASP
- **[Mediciones_JASP/Mediciones_Simulador_IgnitionGazebo/](Mediciones_JASP/Mediciones_Simulador_IgnitionGazebo/)** - Datos de Ignition Gazebo para JASP
- **[Mediciones_JASP/Mediciones_Simulador_IsaacSim/](Mediciones_JASP/Mediciones_Simulador_IsaacSim/)** - Datos de Isaac Sim para JASP

### 📖 [Tutoriales_Instalacion_Simuladores/](Tutoriales_Instalacion_Simuladores/)
Guías completas de instalación y configuración para cada simulador:

- [`Webots.md`](Tutoriales_Instalacion_Simuladores/Webots.md) - Instalación y configuración de Webots con ROS 2
- [`Ignition_Gazebo.md`](Tutoriales_Instalacion_Simuladores/Ignition_Gazebo.md) - Instalación y configuración de Ignition Gazebo
- [`IsaacSim.md`](Tutoriales_Instalacion_Simuladores/IsaacSim.md) - Instalación y configuración de NVIDIA Isaac Sim
- [`Gazebo_Classic.md`](Tutoriales_Instalacion_Simuladores/Gazebo_Classic.md) - Instalación y configuración de Gazebo Classic

### 🔗 [webots_ros2_example/](webots_ros2_example/)
Ejemplo desarrolado en Webots con ROS 2 y Nav2 esplicado paso a paso.

## 📋 Escenarios de Prueba

Los experimentos se realizaron bajo diferentes configuraciones:

- **GUI**: Con y sin interfaz gráfica
- **GPU**: Con y sin aceleración por GPU
- **Estado de batería**: Con el ordenador enchufado a la corriente o sin enchufar
- **Repeticiones**: Múltiples ejecuciones para garantizar la reproducibilidad

## 🎓 Contexto Académico

Este trabajo forma parte del Trabajo Fin de Máster en el contexto de:
- **Universidad**: Universidad de León
- **Máster**: Máster en Robótica e Inteligencia Artificial
- **Autor**: Jorge Casatejada Santamarta