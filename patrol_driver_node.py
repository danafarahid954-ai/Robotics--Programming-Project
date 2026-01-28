import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PatrolDriver(Node):
    def __init__(self):
        super().__init__('patrol_driver_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.state = 'forward'
        self.t0 = time.time()
        
        self.max_range = 100.0
        self.min_dist = self.max_range
        self.min_left = self.max_range
        self.min_right = self.max_range
        self.min_front = self.max_range
        
        self.obstacle_detected = False
        self.obstacle_direction = 'center'

        self.safe_distance = 1.0    
        self.critical_distance = 0.6  
        self.very_critical_distance = 0.4   
        
        self.forward_secs = 3.0     
        self.turn_secs = 1.5        
        self.forward_speed = 0.5    
        self.turn_speed = 1.2    
        self.avoid_speed = 0.3      
        
        self.get_logger().info('Patrol driver started with FAST obstacle avoidance')

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
            
        num_ranges = len(msg.ranges)
        
        front_start = 0
        front_end = 45
        right_start = 45
        right_end = 135
        left_start = 225
        left_end = 315
        front_left_start = 315
        
        front_ranges = msg.ranges[front_start:front_end] + msg.ranges[front_left_start:]
        right_ranges = msg.ranges[right_start:right_end]
        left_ranges = msg.ranges[left_start:left_end]
        
        valid_front = [r for r in front_ranges if not math.isnan(r) and msg.range_min < r < msg.range_max]
        valid_right = [r for r in right_ranges if not math.isnan(r) and msg.range_min < r < msg.range_max]
        valid_left = [r for r in left_ranges if not math.isnan(r) and msg.range_min < r < msg.range_max]
        
        self.min_front = min(valid_front) if valid_front else msg.range_max
        self.min_right = min(valid_right) if valid_right else msg.range_max
        self.min_left = min(valid_left) if valid_left else msg.range_max
        self.min_dist = self.min_front
        
        if self.min_front < self.safe_distance:
            if self.min_left > self.min_right and self.min_left > 0.8:
                self.obstacle_direction = 'left'
            elif self.min_right > self.min_left and self.min_right > 0.8:
                self.obstacle_direction = 'right'
            else:
                self.obstacle_direction = 'left' if self.min_left > self.min_right else 'right'
        else:
            self.obstacle_direction = 'center'
            
        self.obstacle_detected = self.min_front < self.safe_distance

    def loop(self):
        now = time.time()
        dt = now - self.t0
        cmd = Twist()

        if self.min_front < self.very_critical_distance:
            cmd.linear.x = -0.4
            cmd.angular.z = 1.5 if self.obstacle_direction == 'right' else -1.5
            self.state = 'emergency'
            self.t0 = now
            self.get_logger().warn('VERY CRITICAL! Fast reverse and turn')
            
        elif self.min_front < self.critical_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0 if self.obstacle_direction == 'right' else -1.0
            self.state = 'emergency'
            self.t0 = now
            self.get_logger().warn('CRITICAL! Quick turn')
            
        elif self.obstacle_detected:
            cmd.linear.x = self.avoid_speed
            
            if self.obstacle_direction == 'right':
                cmd.angular.z = -0.8
                self.get_logger().debug('Fast turn LEFT - obstacle on RIGHT')
            elif self.obstacle_direction == 'left':
                cmd.angular.z = 0.8
                self.get_logger().debug('Fast turn RIGHT - obstacle on LEFT')
            else:
                if self.min_left > self.min_right:
                    cmd.angular.z = -0.8
                else:
                    cmd.angular.z = 0.8
                    
        elif self.state == 'emergency':
            if dt > 1.0:
                self.state = 'forward'
                self.t0 = now
                self.get_logger().debug('Quick recovery from emergency')
            else:
                cmd.linear.x = -0.2
                cmd.angular.z = 0.0
                
        elif self.state == 'forward':
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
            if dt > self.forward_secs:
                self.state = 'turn'
                self.t0 = now
                self.get_logger().debug('Starting fast turn')
                
        elif self.state == 'turn':
            cmd.linear.x = 0.1
            cmd.angular.z = self.turn_speed
            if dt > self.turn_secs:
                self.state = 'forward'
                self.t0 = now
                self.get_logger().debug('Starting fast forward movement')

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
