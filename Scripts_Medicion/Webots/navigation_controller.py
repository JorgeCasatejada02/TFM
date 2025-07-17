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

    def wait_nav2(self):
        try:
            # Esperar a que Nav2 esté activo
            self.get_logger().info("Esperando a que Nav2 esté activo...")
            self.navigator.waitUntilNav2Active()
            self.initialized = True
            return True
        except Exception as e:
            self.get_logger().error(f"Error al esperar por Nav2: {e}")
            return False

    def start_navigation(self):
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
            ([2.58, 3.335], TurtleBot4Directions.NORTH),
            ([3.0, -1.0], TurtleBot4Directions.EAST),
            ([2.325, -4.25], TurtleBot4Directions.SOUTH),
            ([-3.1, -2.25], TurtleBot4Directions.WEST),
            ([-3.85, 3.75], TurtleBot4Directions.NORTH),
            ([2.58, 3.335], TurtleBot4Directions.NORTH),
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