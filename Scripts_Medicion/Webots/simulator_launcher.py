import os
import signal
import subprocess
import time
import threading

class SimulatorLauncher:
    def __init__(self, mode):
        self.mode = mode
        self.process = None
        self.rtf_collector_process = None
        self._ready_event = threading.Event()
        
        # Definir comandos de lanzamiento según el modo
        self.sim_cmd_map = {
            "sin_gpu_gui": [
                "bash", "-c",
                "ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true world:=maze.wbt gui:=True"
            ],
            "con_gpu_gui": [
                "bash", "-c",
                "__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true world:=maze.wbt gui:=True"
            ],
            "sin_gpu_sin_gui": [
                "bash", "-c",
                "ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true world:=maze.wbt gui:=False"
            ],
            "con_gpu_sin_gui": [
                "bash", "-c",
                "__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true world:=maze.wbt gui:=False"
            ]
        }
    
    def launch(self):
        """Lanza el simulador en un proceso separado"""
        print("🚀 Lanzando simulador")
        
        # Validar que el modo existe
        if self.mode not in self.sim_cmd_map:
            raise ValueError(f"Modo desconocido: {self.mode}")
            
        # Lanzar proceso
        self.process = subprocess.Popen(
            self.sim_cmd_map[self.mode], 
            preexec_fn=os.setsid
        )
        
        # Iniciar hilo para detectar cuando el simulador está listo
        threading.Thread(target=self._wait_for_ready, daemon=True).start()
        
        return self.process
    
    def _wait_for_ready(self):
        """Espera a que el simulador esté listo (verifica solo existencia del topic)"""
        # Esperar a que exista el topic /initialpose
        for _ in range(30):
            try:
                # Obtener lista de topics
                topics = subprocess.check_output(["ros2", "topic", "list"], text=True).splitlines()
                if "/initialpose" in topics:
                    self._ready_event.set()
                    print("✅ Simulador listo (topic /initialpose disponible)")
                    return
            except subprocess.CalledProcessError:
                pass
            print("⏳ Esperando a que el topic /initialpose esté disponible...")
            time.sleep(1)
        
        print("⚠️ Tiempo de espera agotado esperando que el simulador esté listo")
    
    def wait_until_ready(self, timeout=None):
        """Espera hasta que el simulador esté listo para recibir comandos"""
        return self._ready_event.wait(timeout)
    
    def is_running(self):
        """Comprueba si el simulador está ejecutándose"""
        return self.process is not None and self.process.poll() is None
    
    def stop(self):
        """Detiene el simulador"""
        if self.is_running():
            print("🛑 Deteniendo simulador...")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=10)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                print("⚠️ Forzando terminación del simulador")
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            finally:
                self.process = None
                self._ready_event.clear()