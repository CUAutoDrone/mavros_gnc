# -*- coding: utf-8 -*-
#from PrintColours import *
import rclpy
from rclpy.node import Node

from math import atan2, pow, sqrt, degrees, radians, sin, cos
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

"""Control Functions
    This module is designed to make high level control programming simple.
"""


class gnc_api(Node):
    def __init__(self):
        """This function is called at the beginning of a program and will start of the communication links to the FCU.
        """
        super().__init__('gnc_api_node')

        #drone pose and state 
        self.current_state_g = State()
        self.current_pose_g = Odometry() #current position of drone in ENU frame
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0 #angular offset (in degrees) between the ENU frame and local frame
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0

        #Namespace and logging 
        self.ns = rclpy.get_namespace()
        if self.ns == "/":
            self.get_logger().info("Using default namespace")
        else:
            self.get_logger().info("Using {} namespace".format(self.ns))

        #Publishers
        self.local_pose_pub = self.create_publisher(PoseStamped, f'{self.ns}mavros/setpoint_position/local', 10)
        '''
        self.local_pos_pub = rclpy.Publisher(
            name="{}mavros/setpoint_position/local".format(self.ns),
            data_class=PoseStamped,
            queue_size=10,
        )
        '''
        #Subscribers
        self.create_subscription(Odometry, f'{self.ns}mavros/global_position/local', self.pose_cb, 10)
        self.create_subscription(State, f'{self.ns}mavros/state', self.state_cb, 10)
        '''
        self.currentPos = rclpy.Subscriber(
            name="{}mavros/global_position/local".format(self.ns),
            data_class=Odometry,
            queue_size=10,
            callback=self.pose_cb,
        )

        self.state_sub = rclpy.Subscriber(
            name="{}mavros/state".format(self.ns),
            data_class=State,
            queue_size=10,
            callback=self.state_cb,
        )
        '''
        # Service clients
        self.arming_client = self.create_client(CommandBool, f'{self.ns}mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, f'{self.ns}mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, f'{self.ns}mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, f'{self.ns}mavros/set_mode')
        self.command_client = self.create_client(CommandLong, f'{self.ns}mavros/cmd/command')

        self.wait_for_services()
        self.get_logger().info("Initialization Complete")

        def wait_for_services(self):
            self.arming_client.wait_for_service()
            self.takeoff_client.wait_for_service()
            self.land_client.wait_for_service()
            self.set_mode_client.wait_for_service()
            self.command_client.wait_for_service()

    def state_cb(self, message):
        self.current_state_g = message
        self.get_logger().info(f"Current:Mode: {self.current_state_g.mode}, Armed: {self.current_state_g.armed}")

    def pose_cb(self, msg):
        """Gets the raw pose of the drone and processes it for use in control.

        Args:
                msg (geometry_msgs/Pose): Raw pose of the drone.
        """
        self.current_pose_g = msg
        self.enu_2_local()

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.pose.orientation.w,
            self.current_pose_g.pose.pose.orientation.x,
            self.current_pose_g.pose.pose.orientation.y,
            self.current_pose_g.pose.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

        self.current_heading_g = degrees(psi) - self.local_offset_g

    # converts global coordinates (ENU) by transforming global positions into a local frame relative to the drone's reference
    def enu_2_local(self):
        """Returns a transformed point (a Point object) representing the position in the local coordinate frame

        Returns : 
            Position (Point): Returns position of current_pos_local with:
            x: local x-coordinate
            y: local y-coordinate
            z: same z-coordinate as in ENU frame
        """
        x, y, z = (
            self.current_pose_g.pose.pose.position.x,
            self.current_pose_g.pose.pose.position.y,
            self.current_pose_g.pose.pose.position.z,
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.local_offset_g - 90))) - y * sin(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.local_offset_g - 90))) + y * cos(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.z = z

        return current_pos_local

    def get_current_heading(self):
        """Returns the current heading of the drone.

        Returns:
            Heading (Float): θ in is degrees.
        """
        return self.current_heading_g

    def get_current_location(self):
        """Returns the current position of the drone.

        Returns:
            Position (geometry_msgs.Point()): Returns position of type geometry_msgs.Point().
        """
        return self.enu_2_local()

    def land(self):
        """The function changes the mode of the drone to LAND.

        Returns:
                True (boolean): LAND successful
                False (boolean): LAND unsuccessful.
        """
        srv_land = CommandTOL.Request(0, 0, 0, 0, 0)
        try: 
            response = self.land_client.call(srv_land)
            if response.success:
                self.get_logger().info("Land Sent {}".format(str(response.success)))
                return True
            else:
                self.get_logger().info("Landing failed")
                return False
        except Exception as e:
            self.get_logger().error("An error occurred while sending Land Command: {}".format(e))
            return False
        
    def wait4connect(self):
        """Wait for connect is a function that will hold the program until communication with the FCU is established.

        Returns:
                True (bool): Connected to FCU.
                False (bool): Failed to connect to FCU.
        """
        rate = rclpy.rate.Rate(100)

        self.get_logger().info("Waiting for FCU connection")
        while rclpy.ok() and not self.current_state_g.connected:
            rate.sleep()
        else:
            if self.current_state_g.connected:
                self.get_logger().info("FCU connected")
                return True
            else:
                self.get_logger().error("Error connecting to drone's FCU")
                return False

    def wait4start(self):
        """This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.

        Returns:
                True (bool): Mission started successfully.
                False (bool): Failed to start mission.
        """
        self.get_logger().info(
                      "Waiting for user to set mode to GUIDED")
        while rclpy.ok() and self.current_state_g.mode != "GUIDED":
            rclpy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                self.get_logger().info(
                    "Mode set to GUIDED. Starting Mission...")
                return True
            else:
                self.get_logger().error("Error startting mission")
                return False

    def set_mode(self, mode):
        """This function changes the mode of the drone to a user specified mode. This takes the mode as a string. Ex. set_mode("GUIDED").

        Args:
                mode (String): Can be set to modes given in https://ardupilot.org/copter/docs/flight-modes.html

        Returns:
                True (bool): Mode Set successful.
                False (bool): Mode Set unsuccessful.
        """
        SetMode_srv = SetMode.Request(0, mode)
        response = self.set_mode_client.call(SetMode_srv)
        if response.mode_sent:
            self.get_logger().info("SetMode Was successful")
            return True
        else:
            self.get_logger().error("SetMode has failed")
            return False

    def set_speed(self, speed_mps):
        """This function is used to change the speed of the vehicle in guided mode. It takes the speed in meters per second as a float as the input.

        Args:
                speed_mps (Float): Speed in m/s.

        Returns:
                True (bool): Speed set successful.
                False(bool): Speed set unsuccessful.
        """
        speed_cmd = CommandLong.Request()
        speed_cmd.command = 178
        speed_cmd.param1 = 1
        speed_cmd.param2 = speed_mps
        speed_cmd.param3 = -1
        speed_cmd.param4 = 0

        self.get_logger().info(
            "Setting speed to {}m/s".format(str(speed_mps)))
        response = self.command_client.call(speed_cmd)

        if response.success:
            self.get_logger().info(
                "Speed set successfully with code {}".format(str(response.success)))
            self.get_logger().info(
                "Change Speed result was {}".format(str(response.result)))
            return True
        else:
            self.get_logger().error(
                "Speed set failed with code {}".format(str(response.success)))
            self.get_logger().info(
                "Speed set result was {}".format(str(response.result)))
            return False

    def set_heading(self, heading):
        """This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                heading (Float): θ(degree) Heading angle of the drone.
        """
        self.local_desired_heading_g = heading
        adjusted_heading = heading + self.correction_heading_g + self.local_offset_g

        self.get_logger().info("The desired heading is {} degrees".format(
            self.local_desired_heading_g))
        
        self.get_logger().info("The adjusted heading with offsets is {} degrees".format(
            self.local_desired_heading_g))
        
        yaw = radians(adjusted_heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        #set waypoint orientation
        self.waypoint_g.pose.orientation = Quaternion(qx, qy, qz, qw)
        self.get_logger().info("Heading quaternion set to: "
                           f"x={qx}, y={qy}, z={qz}, w={qw}")

    def set_destination(self, x, y, z, psi):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                x (Float): x(m) Distance with respect to your local frame.
                y (Float): y(m) Distance with respect to your local frame.
                z (Float): z(m) Distance with respect to your local frame.
                psi (Float): θ(degree) Heading angle of the drone.
        """
        self.set_heading(psi)

        #finding the rotation angle for local frame correction
        theta = radians((self.correction_heading_g + self.local_offset_g - 90))

        #convert local coordinates to global frame
        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        #Applying global offset corrections
        x_global = Xlocal + self.correction_vector_g.position.x + self.local_offset_pose_g.x
        y_global = Ylocal + self.correction_vector_g.position.y + self.local_offset_pose_g.y
        z_global = Zlocal + self.correction_vector_g.position.z + self.local_offset_pose_g.z

        yaw = radians(psi)

        #set wapoint orientation
        self.waypoint.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=sin(yaw / 2),
            w=cos(yaw / 2)
        )

        self.get_logger().info(
            "Destination set to x:{} y:{} z:{} origin frame".format(x_global, y_global, z_global))

        self.waypoint_g.pose.position = Point(x_global, y_global, z_global)

        self.local_pos_pub.publish(self.waypoint_g)
    
    def arm(self):
        """Arms the drone for takeoff.

        Returns:
                True (boolean): Arming successful.
                False (boolean): Arming unsuccessful.
        """
        self.set_destination(0,0,0,0)

        for _ in range(100):
            self.local_pos_pub.publish(self.waypoint_g)
            rclpy.sleep(0.01)

        self.get_logger().info("Arming Drone")

        arm_request = CommandBool.Request(value = True)

        while rclpy.ok() and not self.current_state_g.armed:
            rclpy.sleep(0.1)
            response = self.arming_client(arm_request)
            self.local_pos_pub.publish(self.waypoint_g)

        if not response.success:
            self.get_logger().error("Arming Failed")
            return False
        else:
            self.get_logger().info("Arming successful")
            return True
        
    def takeoff(self, takeoff_alt):
        """The takeoff function will arm the drone and put the drone in a hover above the initial position.

        Args:
                takeoff_alt (Float): The altitude at which the drone should hover.

        Returns:
                True (boolean): Takeoff successful.
                False (boolean): Takeoff unsuccessful.
        """
        self.arm()
        takeoff_srv = CommandTOL.Request(altitude = takeoff_alt)
        response = self.takeoff_client(takeoff_srv)
        rclpy.sleep(3)
        if response.success:
            self.get_logger().info("Takeoff successful")
            return True
        else:
            self.get_logger().error("Takeoff failed")
            return False

    def initialize_local_frame(self):
        """This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to."""
        self.local_offset_g = 0.0

        for i in range(30):
            rclpy.sleep(0.1)

            q0, q1, q2, q3 = (
                self.current_pose_g.pose.pose.orientation.w,
                self.current_pose_g.pose.pose.orientation.x,
                self.current_pose_g.pose.pose.orientation.y,
                self.current_pose_g.pose.pose.orientation.z,
            )

            psi = atan2((2 * (q0 * q3 + q1 * q2)),
                        (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

            self.local_offset_g += degrees(psi)
            self.local_offset_pose_g.x += self.current_pose_g.pose.pose.position.x
            self.local_offset_pose_g.y += self.current_pose_g.pose.pose.position.y
            self.local_offset_pose_g.z += self.current_pose_g.pose.pose.position.z

        self.local_offset_pose_g.x /= 30.0
        self.local_offset_pose_g.y /= 30.0
        self.local_offset_pose_g.z /= 30.0
        self.local_offset_g /= 30.0

        self.get_logger().info("Coordinate offset set")
        self.get_logger().info("The X-Axis is facing: {}".format(self.local_offset_g))

    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01):
        """This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.

        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.

        Returns:
                True (bool): Waypoint reached successfully.
                False (bool): Failed to reach Waypoint.
        """
        self.local_pos_pub.publish(self.waypoint_g)

        dx = abs(
            self.waypoint_g.pose.position.x - self.current_pose_g.pose.pose.position.x
        )
        dy = abs(
            self.waypoint_g.pose.position.y - self.current_pose_g.pose.pose.position.y
        )
        dz = abs(
            self.waypoint_g.pose.position.z - self.current_pose_g.pose.pose.position.z
        )

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = cos(radians(self.current_heading_g)) - cos(
            radians(self.local_desired_heading_g)
        )

        sinErr = sin(radians(self.current_heading_g)) - sin(
            radians(self.local_desired_heading_g)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return False
        else:
            return True
        
    #setting the velocity of the drone
    def velocity (self, velocity) :
        if velocity > 0:
             self.set_speed(self, abs(velocity))
             self.set_heading(self.get_current_heading(self))
        
        else: 
            self.set_speed(self, abs(velocity))
            self.set_heading(-1*self.get_current_heading(self))

def main():
    rclpy.init()
    gnc = gnc_api()

    try:
        gnc.wait4connect()
        gnc.set_mode("GUIDED")
        gnc.takeoff(10.0)
        gnc.set_destination(10, 10, 10, 90)
        rclpy.spin(gnc)
    except KeyboardInterrupt:
        pass
    finally:
        gnc.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()  