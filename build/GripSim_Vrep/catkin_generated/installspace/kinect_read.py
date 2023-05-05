#!/usr/bin/env python2
import rospy
from  sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


output_file = "pc.pcd"

def save_pcd(msg):
        points = point_cloud2.read_points_list(msg, field_names= ("x", "y", "z"))
        with open("./src/see_and_touch/scripts/output.csv", "w+") as f:
                for point in points:
                        f.write("{}, {}, {}\n".format(point.x, point.y, point.z))
        #print("save one")
        


def main():
        
        rospy.init_node("ros_client", anonymous=True)
        rospy.Subscriber("/kinect/depth", PointCloud2, save_pcd)
        rospy.spin()

if __name__ == "__main__":
        try:
                main()
        except KeyboardInterrupt:
                print("Break down.")
