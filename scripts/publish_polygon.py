#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from costmap_prohibition_layer.msg import ProhibitionAreas

class clickedToPolygon(object):
    """Add node description here"""

    def __init__(self):
        rospy.loginfo('init clicked_point to make polygon')
        self._init_pubsub()
        self.points = []

    def _init_pubsub(self):
        rospy.Subscriber("/clicked_point", PointStamped, self._clicked_cb, queue_size=4)

    def _clicked_cb(self, msg):
        point = Point32()
        rospy.loginfo('clicked callback')
        rospy.loginfo('click point %f' % msg.point.x)
        _click_point = np.array([msg.point.x, msg.point.y, 0.0])
        _click_point_float32 =  _click_point.astype(np.float32)
        point.x = _click_point_float32[0]
        point.y = _click_point_float32[1]
        point.z = _click_point_float32[2]
        print(point.x)
        self.points.append(point)
        print(self.points) 
        # import pdb; pdb.set_trace()

    def get_points(self):
        return self.points

    def reset_points(self):
        self.points= []

class PublishProhibitionArea(object):
    """docstring for PublishProhibitionArea"""
    def __init__(self):
        self.area_publisher = rospy.Publisher("/move_base/global_costmap/costmap_prohibition_layer/prohibition_areas_update", ProhibitionAreas, queue_size=1)
        self.click_to_polygon = clickedToPolygon()
        self.prohibition_msg = ProhibitionAreas()
        self.polygon = Polygon()
        self.polygons = []
    def main_loop(self):
        points = self.click_to_polygon.get_points()
        rospy.loginfo('get length %d' % len(points))
        if len(points) is 4:
            # import pdb; pdb.set_trace()
            print(points) 
            self.prohibition_msg.fill_polygons = False
            self.polygon.points = points
            self.polygons.append(self.polygon)
            self.prohibition_msg.polygons = self.polygons
            self.area_publisher.publish(self.prohibition_msg)
            self.polygons=[]
            self.click_to_polygon.reset_points()
        elif len(points) < 4:
            pass
        rospy.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('click_to_prohibition_area')
    node = PublishProhibitionArea()
    while not rospy.is_shutdown():
        node.main_loop()
    rospy.spin()
