import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

from tf_transformations import euler_from_quaternion


class ActionManager(Node):
    def __init__(self):
        Node.__init__(self, 'controller_manager')
        self.cli = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

    def ask_for_takeoff(self):
        self.req.cmd = "takeoff"
        self.future = self.cli.call_async(self.req)

    def ask_for_landing(self):
        self.req.cmd = "land"
        self.future = self.cli.call_async(self.req)


class Gate:
    """
    Description
    -----------
        This class will contain the location and orientation of one of the gates through which the UAV will have to pass.
    
    Attributes
    ----------
        x : float
            Global x coordinate of the gate.
        y : float
            Global y coordinate of the gate.
        z : float
            Global z coordinate of the gate.
        yaw : float
            Orientation (heading) of the gate.
    """
    def __init__(self, x : float, y : float, z : float, yaw : float):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
    

    def __repr__(self):
        return f"Pose: ({self.x}, {self.y}, {self.z}, {self.yaw})"


class Agent(Node):
    def __init__(self):
        Node.__init__(self, "nav_node")

        self.config_gates()
        self.x, self.y, self.z, self.yaw = 0, 0, 0, 0 #UAV's pose
        #TODO: define your attributes here
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/drone1/cmd_vel", 1)   #Publisher to send velocity messages to the topic /drone1/cmd_vel
        self.create_subscription(Odometry, "/drone1/odom", self.odom_cb, 10)    #Subscribe to /drone1/odom topic
        self.create_timer(timer_period_sec=0.1, callback=self.run)  #Periodically call the method 'self.run' at 10Hz
    

    def config_gates(self):
        """
        Description
        ------------
            Create a list that will contain the coordinates and orientation of each gate through which the UAV will have to pass.

        Returns
        -------
            None
        """
        self.gates = []
        self.gates.append(Gate(x=2.0, y=0.0, z=0.5, yaw=0.0))
        self.gates.append(Gate(x=5.0, y=3.0, z=1.0, yaw=0.79))
        self.gates.append(Gate(x=3.0, y=6.0, z=0.75, yaw=-0.79))
        self.gates.append(Gate(x=0.0, y=9.0, z=1.0, yaw=0.0))
        self.gates.append(Gate(x=-3.0, y=6.0, z=0.75, yaw=0.79))
        self.gates.append(Gate(x=-2.0, y=0.0, z=1.25, yaw=1.57))


    def odom_cb(self, msg : Odometry):
        """
        Description
        ------------
            Get agent 1 position and heading. 
            This method is automatically called whenever a new message is published on the topic /drone1/odom.
        
        Parameters
        ----------
            msg : Odometry
                Message received on the topic /drone1/odom.
        
        Returns
        -------
            None
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        self.yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]


    def set_velocities(self, vx : float, vy : float, vz : float, vyaw : float):
        """
        Description
        ------------
            Publish a Twist message on the topic /drone1/cmd_vel. Make sure to set velocity commands in FLOAT on each axis.

        Parameters
        ----------
            vx : float
                Target linear velocity along the UAV's x axis.
            vy : float
                Target linear velocity along the UAV's y axis.
            vz : float
                Target linear velocity along the UAV's z axis.
            vyaw : float
                Target angular velocity around the UAV's z axis.
                
        Returns
        ------
            None
        """
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = vyaw
        self.cmd_vel_pub.publish(msg)
    

    def run(self):
        """
        Description
        ------------
            This method will define the UAV's behavior.
        
        Returns
        -------
            None
        """
        self.get_logger().info(f"Drone's pose: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f}, {self.yaw:.2f})") #Print drone's position and heading

        for i, gate in enumerate(self.gates):   #Print gates locations
            self.get_logger().info(f"Gate {i+1} - {gate}")
        self.get_logger().info("")

        self.set_velocities(0.0, 0.0, 0.0, 0.0) #Make the drone static





def main(args=None):
    rclpy.init(args=args)

    # Wait for take off service to be ready
    action_manager = ActionManager()
    action_manager.ask_for_takeoff()
    ready_to_continue_mission = False

    # Try to takeoff, wait for the return of the service
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    action_manager.get_logger().info("Take off is a success !")
                    ready_to_continue_mission = True
                else:  # NOT OK
                    action_manager.get_logger().warn("Something is wrong with the takeoff. LANDING NOW")
            except Exception as e:
                action_manager.get_logger().error(f"Service call failed: {e}")
            break
    
    #Start the Agent node if the drone took-off
    if ready_to_continue_mission:
        controller = Agent()
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
        except KeyboardInterrupt:
            controller.get_logger().info("Stopping the control. Ask for landing.")
        controller.destroy_node()
    
    # Ask for landing
    action_manager.ask_for_landing()
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    action_manager.get_logger().info("Landing is a success !")
                    break  # Only if landing is ok
            except Exception as e:
                action_manager.get_logger().error(f"Service call failed: {e}")
            break

    rclpy.shutdown()