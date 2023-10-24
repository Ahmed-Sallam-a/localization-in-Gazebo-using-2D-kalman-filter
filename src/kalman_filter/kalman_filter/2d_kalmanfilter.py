'''import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables

        
        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)
        
        #publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        
        
        # Prediction step

        
        # Update step

        
        #publish the estimated reading

        pass    

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''
'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables
        self.x = np.array([[0], [0]])  # initial state (location and velocity)
        self.P = np.array([[1000, 0], [0, 1000]])  # initial uncertainty
        self.u = np.array([[0], [0]])  # external motion
        self.F = np.array([[1, 1], [0, 1]])  # next state function
        self.H = np.array([[1, 0]])  # measurement function
        self.R = np.array([[1]])  # measurement uncertainty
        self.I = np.array([[1, 0], [0, 1]])  # identity matrix
        
        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)
        
        # Publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        Z = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])
        
        # Prediction step
        self.x = np.dot(self.F, self.x) + self.u
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose())
        
        # Update step
        Y = Z.transpose() - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.transpose())) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), inv(S))
        self.x = self.x + np.dot(K, Y)
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)
        
        # Publish the estimated reading
        estimated_odom_msg = Odometry()
        estimated_odom_msg.pose.pose.position.x = float(self.x[0][0])
        estimated_odom_msg.pose.pose.position.y = float(self.x[1][0])
        estimated_odom_msg.header.stamp = msg.header.stamp
        estimated_odom_msg.header.frame_id = msg.header.frame_id
        estimated_odom_msg.child_frame_id = msg.child_frame_id
        
        self.estimated_pub.publish(estimated_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables
        self.x = np.array([[0], [0]])  # initial state (location and velocity)
        self.P = np.array([[1000, 0], [0, 1000]])  # initial uncertainty
        self.u = np.array([[0], [0]])  # external motion
        self.F = np.array([[1, 1], [1, 1]])  # next state function
        self.H = np.array([[1, 0]])  # measurement function
        self.R = np.array([[1]])  # measurement uncertainty
        self.I = np.array([[1, 0], [0, 1]])  # identity matrix
        
        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)
        
        # Publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        Z = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])
        
        # Prediction step
        self.x = np.dot(self.F, self.x) + self.u
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose())
        
        # Update step
        Y = Z.transpose() - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.transpose())) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), inv(S))
        self.x = self.x + np.dot(K, Y)
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)
        
        # Publish the estimated reading
        estimated_odom_msg = Odometry()
        estimated_odom_msg.pose.pose.position.x = float(self.x[0][0])
        estimated_odom_msg.pose.pose.position.y = float(self.x[1][0])
        estimated_odom_msg.header.stamp = msg.header.stamp
        estimated_odom_msg.header.frame_id = msg.header.frame_id
        estimated_odom_msg.child_frame_id = msg.child_frame_id
        
        self.estimated_pub.publish(estimated_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables
        self.x = np.array([[0], [0]])  # initial state (location)
        self.P = np.array([[1, 0], [0, 1]])  # initial uncertainty
        self.F = np.array([[1, 0], [0, 1]])  # next state function
        self.H = np.array([[1, 0], [0, 1]])  # measurement function
        self.R = np.array([[0.01, 0], [0, 0.01]])  # measurement uncertainty
        self.I = np.array([[1, 0], [0, 1]])  # identity matrix
        
        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)
        
        # Publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        Z = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])
        
        # Prediction step
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose())
        
        # Update step
        Y = Z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.transpose())) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), inv(S))
        self.x = self.x + np.dot(K, Y)
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)
        
        # Publish the estimated reading
        estimated_odom_msg = Odometry()
        estimated_odom_msg.pose.pose.position.x = float(self.x[0][0])
        estimated_odom_msg.pose.pose.position.y = float(self.x[1][0])
        estimated_odom_msg.header.stamp = msg.header.stamp
        estimated_odom_msg.header.frame_id = msg.header.frame_id
        estimated_odom_msg.child_frame_id = msg.child_frame_id
        
        self.estimated_pub.publish(estimated_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables
        self.x = np.array([[0], [0]])  # initial state (location and velocity)
        self.P = np.array([[1000, 0], [0, 1000]])  # initial uncertainty
        self.u = np.array([[0], [0]])  # external motion
        self.F = np.array([[1, 0], [0, 1]])  # next state function
        self.H = np.array([[1, 0], [0, 1]])  # measurement function
        self.R = np.array([[1, 0], [0, 1]])  # measurement uncertainty
        self.I = np.array([[1, 0], [0, 1]])  # identity matrix
        
        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)
        
        # Publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        Z = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])
        
        # Prediction step
        self.x = np.dot(self.F, self.x) + self.u
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose())
        
        # Update step
        Y = Z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.transpose())) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), inv(S))
        self.x = self.x + np.dot(K, Y)
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)
        
        # Publish the estimated reading
        estimated_odom_msg = Odometry()
        estimated_odom_msg.pose.pose.position.x = float(self.x[0][0])
        estimated_odom_msg.pose.pose.position.y = float(self.x[1][0])
        estimated_odom_msg.header.stamp = msg.header.stamp
        estimated_odom_msg.header.frame_id = msg.header.frame_id
        estimated_odom_msg.child_frame_id = msg.child_frame_id
        
        self.estimated_pub.publish(estimated_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from numpy.linalg import inv

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')
        # Initialize kalman variables
        self.x = np.array([[0], [0]])  # initial state (location and velocity)
        self.P = np.array([[1, 0], [0, 1]])  # initial uncertainty
        self.F = np.array([[1, 0], [0, 1]])  # next state function
        self.H = np.array([[1, 0], [0, 1]])  # measurement function
        self.R = np.array([[1, 0], [0, 1]])  # measurement uncertainty
        self.Q = np.array([[0.01, 0], [0, 0.01]])  # process noise covariance
        self.I = np.array([[1, 0], [0, 1]])  # identity matrix
        
        # Subscribe to the /odom_noise topic
        self.subscription = self.create_subscription(Odometry,
                                                     '/odom_noise',
                                                     self.odom_callback,
                                                     1)
        
        # Publish the estimated reading
        self.estimated_pub=self.create_publisher(Odometry,
                                                 "/odom_estimated",1)

    def odom_callback(self, msg):
        # Extract the position measurements from the Odometry message
        Z = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y]])
        
        # Prediction step
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose()) + self.Q
        
        # Update step
        Y = Z - np.dot(self.H, self.x)
        S = np.dot(self.H, np.dot(self.P, self.H.transpose())) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), inv(S))
        self.x = self.x + np.dot(K, Y)
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)
        
        # Publish the estimated reading
        estimated_odom_msg = Odometry()
        estimated_odom_msg.pose.pose.position.x = float(self.x[0][0])
        estimated_odom_msg.pose.pose.position.y = float(self.x[1][0])
        estimated_odom_msg.header.stamp = msg.header.stamp
        estimated_odom_msg.header.frame_id = msg.header.frame_id
        estimated_odom_msg.child_frame_id = msg.child_frame_id
        
        self.estimated_pub.publish(estimated_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

