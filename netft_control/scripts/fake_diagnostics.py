#!/usr/bin/env python
import os
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rclpy.init_node(node_name)
  publish_rate = rclpy.get_param('~publish_rate', 1.0)
  frame_id = rclpy.get_param('~frame_id', 'base_link')
  ip_address = rclpy.get_param('~ip_address', '192.168.0.12')
  pub = rclpy.Publisher('/ft_sensor/diagnostics', DiagnosticArray, queue_size=3)
  msg = DiagnosticArray()
  msg.header.frame_id = frame_id
  status = DiagnosticStatus()
  status.level = DiagnosticStatus.OK
  status.name = 'NetFT RDT Driver'
  status.message = 'OK'
  status.hardware_id = 'ATI Gamma'
  status.values.append(KeyValue('IP Address', ip_address))
  msg.status.append(status)
  rate = rclpy.Rate(publish_rate)
  while not rclpy.is_shutdown():
    msg.header.stamp = rclpy.Time.now()
    pub.publish(msg)
    rate.sleep()
