import rclpy
import cv2
import cv2.aruco as aruco
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

twist_msg = Twist()
bridge = CvBridge()
last_marker_id = None
# We know that there are 5 consecutive markers in total
total_markers = 5
markers = []
current_marker_index = 0
list_complete = False

def marker_callback(msg):
    global current_marker_index, twist_msg, last_marker_id, list_complete,lower_marker_id
    if msg.marker_ids[0] not in markers:
        twist_msg.angular.z = 0.5 
        markers.append(msg.marker_ids[0])
        print(f'Marker {msg.marker_ids[0]}')
    if len(markers) == total_markers:
        list_complete = True
        lower_marker_id = markers[0]
        print(f'List complete {markers}')
        markers.sort()
        print(f'Sorted list {markers}')
    # First if to ensure to be in the range of the number of markers
    if current_marker_index < len(markers) and list_complete == True: 
        # Get the current marker id to find
        current_marker_id = markers[current_marker_index]
        # If the current marker id is in the list of markers found (returned by the aruco node)
        if current_marker_id in msg.marker_ids:
            print(f'Marker {current_marker_id} found')
            current_marker_index += 1  
            last_marker_id = current_marker_id  
            if current_marker_index >= len(markers):
                stop_rotation()
        else:
            twist_msg.angular.z = 0.5  
            print(f'Rotating to find marker {current_marker_id}')
    publisher.publish(twist_msg)

def image_callback(msg):
    global bridge, last_marker_id, last_marker_id,lower_marker_id
    if last_marker_id is not None:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        dictionary = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()

        corners, ids, _ = aruco.detectMarkers(cv_image, dictionary, parameters=parameters)
        
        if corners is not None:
            x_centerPixel = int((corners[0][0][0][0] + corners[0][0][2][0]) / 2)
            y_centerPixel = int((corners[0][0][0][1] + corners[0][0][2][1]) / 2)
            center = np.array([x_centerPixel, y_centerPixel])
            radius = int(np.linalg.norm(corners[0][0][0][0] - center))
            print(f'Center: {center}, Radius: {radius}')
        else:
            x_centerPixel = 0
            y_centerPixel = 0
            print('No markers found')

        cv2.circle(cv_image, (x_centerPixel, y_centerPixel), radius, (0, 0, 255), 1)
        cv2.imshow('Image', cv_image)
        if last_marker_id == lower_marker_id:
            cv2.waitKey(100)
        else:
            cv2.waitKey(1)
        last_marker_id = None

        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        image_publisher.publish(ros_image)

def stop_rotation():
    global twist_msg
    twist_msg.angular.z = 0.0  
    publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    global node, publisher, image_publisher
    node = rclpy.create_node('aruco_marker_subscriber')
    node.create_subscription(ArucoMarkers, 'aruco_markers', marker_callback, 10)
    node.create_subscription(Image, '/camera/image_raw', image_callback, 10)
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    image_publisher = node.create_publisher(Image, '/img_markers', 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
