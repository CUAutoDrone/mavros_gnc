# -*- coding: utf-8 -*-
#!/user/bin/env python3
#from PrintColours import *
import rclpy
import time
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy import spin_once
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
    def __init__(self, node_name: str, namespace: str):
        """This function is called at the beginning of a program and will start of the communication links to the FCU.
        """
        print("start init")
        super().__init__(node_name)
        print("super")
        #drone pose and state 
        self.node = rclpy.create_node('gnc_api_node')
        self.current_state_g = State()
        self.current_pose_g = Odometry() #current position of drone in ENU frame
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()
        self.connected = False

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0 #angular offset (in degrees) between the ENU frame and local frame
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0
        print("set drone pos")
        #Namespace and logging 
        self.ns = namespace
        if self.ns == "/":
            self.get_logger().info("Using default namespace")
        else:
            self.get_logger().info("Using {} namespace".format(self.ns))
        print("set namespace")
        #Publishers
        self.local_pose_pub = self.create_publisher(PoseStamped, f'{self.ns}mavros/setpoint_position/local', 10)
        print("set publisher")
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
        print("set subscribers")
        # Service clients
        self.arming_client = self.node.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_client = self.node.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.land_client = self.node.create_client(CommandTOL, 'mavros/cmd/land')
        self.set_mode_client = self.node.create_client(SetMode, 'mavros/set_mode')
        self.command_client = self.node.create_client(CommandLong, 'mavros/cmd/command')
        print("set clients")
        def wait_for_services(self):
            self.arming_client.wait_for_service()
            self.takeoff_client.wait_for_service()
            self.land_client.wait_for_service()
            self.set_mode_client.wait_for_service()
            self.command_client.wait_for_service()
        print("define wait for services")
        wait_for_services(self)
        self.connected = True
        print("done wait for services")
        self.get_logger().info("Initialization Complete")
        print("init finished")
        

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
        
    # def wait4connect(self):
        """Wait for connect is a function that will hold the program until communication with the FCU is established.

        Returns:
                True (bool): Connected to FCU.
                False (bool): Failed to connect to FCU.
        """
        # rate = rclpy.rate.Rate(100)

        # self.get_logger().info("Waiting for FCU connection")
        # while rclpy.ok() and not self.current_state_g.connected:
        #     rate.sleep()
        # else:
        #     if self.current_state_g.connected:
        #         self.get_logger().info("FCU connected")
        #         return True
        #     else:
        #         self.get_logger().error("Error connecting to drone's FCU")
        #         return False
    def wait4connect(self, timeout_sec=10):
        self.get_logger().info("Waiting for FCU connection...")
        start_time = self.get_clock().now()

        # Wait for connection with timeout
        while not self.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time).nanoseconds > timeout_sec * 1e9:
                self.get_logger().error("Timeout waiting for FCU connection.")
                raise TimeoutError("Failed to connect to FCU within timeout.")

        self.get_logger().info("FCU is connected!")
    def wait4start(self):
        """This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.

        Returns:
                True (bool): Mission started successfully.
                False (bool): Failed to start mission.
        """
        self.get_logger().info(
                      "Waiting for user to set mode to GUIDED")
        while rclpy.ok() and self.current_state_g.mode != "GUIDED":
            time.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                self.get_logger().info(
                    "Mode set to GUIDED. Starting Mission...")
                return True
            else:
                self.get_logger().error("Error startting mission")
                return False

    def set_mode(self, mode: str) -> bool:
        """
        Change the mode of the drone using the /mavros/set_mode service.

        Args:
            mode (str): Desired flight mode (e.g., 'OFFBOARD', 'AUTO.LOITER').

        Returns:
            bool: True if mode change was successful, False otherwise.
        """
        self.get_logger().info(f"Attempting to change mode to {mode}")

        # Create the request
        request = SetMode.Request()
        request.base_mode = 0  # Base mode (set to 0 unless specific base_mode is needed)
        request.custom_mode = mode  # Desired mode

        # Send the request asynchronously
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Process the response
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f"Mode successfully changed to {mode}.")
            return True
        else:
            self.get_logger().error(f"Failed to change mode to {mode}.")
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
        """This function is used to specify the drone's heading in the local reference frame. 
        Psi is a counterclockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone 
        with the y axis through the front of the drone.

        Args:
                heading (float): θ(degree) Heading angle of the drone.
        """
        self.local_desired_heading_g = heading
        adjusted_heading = heading + self.correction_heading_g + self.local_offset_g

        self.get_logger().info("The desired heading is {} degrees".format(self.local_desired_heading_g))
        self.get_logger().info("The adjusted heading with offsets is {} degrees".format(adjusted_heading))

        # Convert adjusted heading from degrees to radians
        yaw = radians(adjusted_heading)
        pitch = 0.0
        roll = 0.0

        # Convert Euler angles to quaternion
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

        # Set waypoint orientation
        self.waypoint_g.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)  # Fixed: Quaternion constructor
        self.get_logger().info(f"Heading quaternion set to: x={qx}, y={qy}, z={qz}, w={qw}")


    def set_destination(self, x, y, z, psi):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counterclockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.

        Args:
                x (float): x (m) Distance with respect to your local frame.
                y (float): y (m) Distance with respect to your local frame.
                z (float): z (m) Distance with respect to your local frame.
                psi (float): θ (degree) Heading angle of the drone.
        """
        # Set the desired heading (psi)
        self.set_heading(psi)

        # Calculate the rotation angle for local frame correction
        theta = radians(self.correction_heading_g + self.local_offset_g - 90)

        # Convert local coordinates to global frame
        x_local = x * cos(theta) - y * sin(theta)
        y_local = x * sin(theta) + y * cos(theta)
        z_local = z

        # Apply global offset corrections
        x_global = x_local + self.correction_vector_g.position.x + self.local_offset_pose_g.x
        y_global = y_local + self.correction_vector_g.position.y + self.local_offset_pose_g.y
        z_global = z_local + self.correction_vector_g.position.z + self.local_offset_pose_g.z

        # Convert psi (heading) to radians for quaternion
        yaw = radians(psi)

        # Set the waypoint orientation as a quaternion
        self.waypoint_g.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=sin(yaw / 2),
            w=cos(yaw / 2)
        )

        # Log the destination set in the global frame
        self.get_logger().info(
            "Destination set to x:{} y:{} z:{} (global frame)".format(x_global, y_global, z_global)
        )

        # Set the position of the waypoint in the global frame using keyword arguments
        self.waypoint_g.pose.position = Point(x=x_global, y=y_global, z=z_global)

        # Publish the waypoint to the local position publisher
        self.local_pose_pub.publish(self.waypoint_g)
        
    def arm(self):
        """Arms the drone for takeoff.

        Returns:
                True (boolean): Arming successful.
                False (boolean): Arming unsuccessful.
        """
        # Set the destination (you should have defined set_destination elsewhere)
        self.set_destination(0, 0, 0, 0)

        # Publish the waypoint repeatedly (if needed)
        for _ in range(100):
            self.local_pose_pub.publish(self.waypoint_g)
            time.sleep(0.01)

        self.node.get_logger().info("Arming Drone")

        # Create the arm request
        arm_request = CommandBool.Request()
        arm_request.value = True  # True to arm the drone

        # Send the arm request asynchronously
        future = self.arming_client.async_send_request(arm_request)

        # Wait until the drone is armed or timeout occurs
        while rclpy.ok() and not self.current_state_g.armed:
            time.sleep(0.1)  # Sleep for a short duration
            self.local_pose_pub.publish(self.waypoint_g)  # Publish the waypoint again

        # Wait for the future response (to confirm arming)
        rclpy.spin_until_future_complete(self.node, future)

        # Check the response
        response = future.result()

        if response is None or not response.success:
            self.node.get_logger().error("Arming Failed")
            return False
        else:
            self.node.get_logger().info("Arming successful")
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
        time.sleep(3)
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
            time.sleep(0.1)

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
        self.local_pose_pub.publish(self.waypoint_g)

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

def main(args = None):
    print("starting gnc")
    rclpy.init(args = args)

    node_name = "gnc_api_node"
    namespace = "gnc_namespace"

    gnc = gnc_api(node_name=node_name, namespace=namespace)
    print("done")
    try:
        print("inside try")
        gnc.wait4connect()
        print("connect gnc")
        # gnc.wait_for_services()
        # print("wait gnc")
        gnc.set_mode("GUIDED")
        print("set mode")
        gnc.arm()
        gnc.takeoff(10.0)
        gnc.set_destination(10, 10, 10, 90)
        rclpy.spin(gnc)
    except KeyboardInterrupt:
        pass
    finally:
        gnc.destroy_node()
        rclpy.shutdown()
        print("shutting down")
    


if __name__ == "__main__":
    main()  