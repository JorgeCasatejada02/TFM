import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
import time

class RTFMonitor(Node):
    def __init__(self):
        super().__init__('rtf_monitor')
        self.subscription = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        # Publisher para RTF
        self.rtf_publisher = self.create_publisher(Float64, '/rtf', 10)
        
        # Timer para publicar cada segundo
        self.timer = self.create_timer(1.0, self.publish_rtf)
        
        self.start_sim_time = None
        self.start_wall_time = time.perf_counter()
        self.current_sim_time = 0.0
        self.step_count = 0

    def clock_callback(self, msg):
        self.step_count += 1
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        
        if self.start_sim_time is None:
            self.start_sim_time = self.current_sim_time

    def publish_rtf(self):
        if self.start_sim_time is None:
            return
            
        sim_elapsed = self.current_sim_time - self.start_sim_time
        wall_elapsed = time.perf_counter() - self.start_wall_time
        rtf = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0.0
        
        # Publicar RTF
        rtf_msg = Float64()
        rtf_msg.data = rtf
        self.rtf_publisher.publish(rtf_msg)
        
        # Log cada segundo
        self.get_logger().info(f'RTF: {rtf:.3f} | Sim: {sim_elapsed:.2f}s | Real: {wall_elapsed:.2f}s | Steps: {self.step_count}')

def main(args=None):
    rclpy.init(args=args)
    node = RTFMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()