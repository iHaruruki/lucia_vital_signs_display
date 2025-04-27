#!/usr/bin/env python3
import rclpy, random
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

class RandomPub(Node):
    def __init__(self):
        super().__init__('random_vital_pub')
        self.topics = [
            ('/vital/A/hr',    Int32),
            ('/vital/A/spo2',  Float32),
            ('/vital/A/sys_bp',Int32),
            ('/vital/A/dia_bp',Int32),
            ('/vital/B/hr',    Int32),
            ('/vital/B/spo2',  Float32),
            ('/vital/B/sys_bp',Int32),
            ('/vital/B/dia_bp',Int32),
            ('/vital/C/hr',    Int32),
            ('/vital/C/spo2',  Float32),
            ('/vital/C/sys_bp',Int32),
            ('/vital/C/dia_bp',Int32),
        ]
        self.pubs = {
            topic: self.create_publisher(msg_type, topic, 10)
            for topic, msg_type in self.topics
        }
        # 200ms ごとに publish
        self.create_timer(0.2, self.publish_random)

    def publish_random(self):
        for topic, msg_type in self.topics:
            if msg_type is Float32:
                val = random.uniform(90.0, 100.0)
                msg = Float32(data=val)
            else:
                val = random.randint(60, 100)
                msg = Int32(data=val)
            self.pubs[topic].publish(msg)

def main():
    rclpy.init()
    node = RandomPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
