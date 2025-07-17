import rclpy
import time
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
import threading

class NavigationController(Node):
    def __init__(self):
        # Verificar si ROS2 ya está inicializado
        if not rclpy.ok():
            rclpy.init()
        super().__init__('navigation_controller')
        self.navigator = TurtleBot4Navigator()
        self.initialized = False
        self.navigation_thread = None
        self.stop_requested = False

    def set_initial_pose(self):
        try:
            # Establecer la pose inicial
            initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
            self.navigator.setInitialPose(initial_pose)

            # Esperar a que Nav2 esté activo
            self.get_logger().info("Esperando a que Nav2 esté activo...")
            self.navigator.waitUntilNav2Active()
            self.initialized = True
            return True
        except Exception as e:
            self.get_logger().error(f"Error al establecer pose inicial: {e}")
            return False

    def start_navigation(self):
        if not self.initialized:
            self.get_logger().warn("La navegación no está inicializada. Llama a set_initial_pose() primero.")
            return False
            
        if self.navigation_thread and self.navigation_thread.is_alive():
            self.get_logger().warn("Ya hay una navegación en curso")
            return False
            
        self.stop_requested = False
        self.navigation_thread = threading.Thread(target=self._navigate_to_waypoints)
        self.navigation_thread.daemon = True  # Importante: thread daemon se termina cuando el programa principal termina
        self.navigation_thread.start()
        return True

    def _navigate_to_waypoints(self):
        waypoints = [
            ([5.16, 6.67], TurtleBot4Directions.NORTH),
            ([6.0, -2.0], TurtleBot4Directions.EAST),
            ([4.65, -8.5], TurtleBot4Directions.SOUTH),
            ([-6.2, -4.5], TurtleBot4Directions.WEST),
            ([-7.7, 7.5], TurtleBot4Directions.NORTH)
        ]

        for i, (coords, direction) in enumerate(waypoints):
            if self.stop_requested:
                self.get_logger().info("Navegación detenida por solicitud")
                break
                
            pose = self.navigator.getPoseStamped(coords, direction)
            self.get_logger().info(f"➡️ Navegando al waypoint {i+1}: {coords}")
            self.navigator.goToPose(pose)

            # Esperar hasta que llegue al destino
            while not self.navigator.isTaskComplete() and not self.stop_requested:
                time.sleep(1)
                
            if self.stop_requested:
                self.navigator.cancelTask()
                break
                
            self.get_logger().info(f"✅ Waypoint {i+1} alcanzado")

    def stop_navigation(self):
        self.get_logger().info("Deteniendo navegación")
        self.stop_requested = True
        
        # Cancelar tarea de navegación actual si existe
        if self.initialized and hasattr(self, 'navigator'):
            try:
                self.navigator.cancelTask()
            except Exception as e:
                self.get_logger().error(f"Error al cancelar tarea: {e}")
        
        # Esperar a que termine el hilo de navegación
        if self.navigation_thread and self.navigation_thread.is_alive():
            self.navigation_thread.join(timeout=5)
            
        # NO llamar a rclpy.shutdown() aquí, lo haremos al final del programa principal