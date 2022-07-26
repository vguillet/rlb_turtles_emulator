
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math
import numpy as np # Scientific computing library for Python


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    sim = Turtles_emulator()

    rclpy.spin(sim)

    sim.destroy_node()
    rclpy.shutdown()


class Turtles_emulator(Node):
    def __init__(self):
        Node.__init__(self, 'Turtles_emulator')

        # -> Setup sim variables
        self.agent_count = 1
        self.agents_dict = {}

        self.timer_step = 0.000001
        self.realtime_factor = 1.0 

        self.auto_dt = True

        for i in range(self.agent_count):
            self.add_agent(name=f"Turtle_{i+2}")

        self.sim_timer = self.create_timer(
            timer_period_sec=self.timer_step,
            callback=self.sim_step
        )

    def sim_step(self):
        print("\n-> Stepping")
        for agent in self.agents_dict.keys():
                self.agents_dict[agent].step(
                    dt=self.timer_step,
                    auto_dt=self.auto_dt
                    )

    def add_agent(self, name):
        new_agent = Agent(name=name, sim_node=self)
        self.agents_dict[name] = new_agent

class Agent(Node):
    def __init__(self, name, sim_node):
        Node.__init__(self, name)

        self.sim_node = sim_node
        self.robot_id = name
        self.last_stamp = sim_node.get_clock().now()

        # -> Setup agent states
        self.pose = {
            "position": {
                "x": 0.,
                "y": 0.,
                "z": 0.
                },

            "orientation": {
                "phi": 0.,
                "theta": 0.,
                "psi": 0.,
                "w": 0.
            }
        }

        self.vel = {
            "linear": {
                "x": 0.,
                "y": 0.,
                "z": 0.
            },

            "angular": {
                "p": 0.,
                "q": 0.,
                "r": 0.
            }
        }

        # ----------------------------------- Pose publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.pose_publisher = sim_node.create_publisher(
            msg_type=PoseStamped,
            topic=f"/{self.robot_id}/pose",
            qos_profile=qos
            )

        # ----------------- timer
        timer_period = .000001  # seconds

        self.pose_timer = sim_node.create_timer(
            timer_period, 
            self.publish_pose
            )

        # ----------------------------------- Instruction subscription
        qos = QoSProfile(depth=10)
        
        self.cmd_vel_subscriber = sim_node.create_subscription(
            msg_type=Twist,
            topic=f"/{self.robot_id}/cmd_vel",
            callback=self.cmd_vel_callback,
            qos_profile=qos
            )

    def set_pose(self, pose):
        self.pose["position"]["x"] = pose.position.x
        self.pose["position"]["y"] = pose.position.y
        self.pose["position"]["z"] = pose.position.z
        
        self.pose["orientation"]["phi"] = pose.orientation.x
        self.pose["orientation"]["theta"] = pose.orientation.y
        self.pose["orientation"]["psi"] = pose.orientation.z

    def publish_pose(self):
        msg = PoseStamped()

        msg.pose.position.x = self.pose["position"]["x"]
        msg.pose.position.y = self.pose["position"]["y"]
        msg.pose.position.z = self.pose["position"]["z"]

        qx, qy, qz, qw = self.get_quaternion_from_euler(
            roll=self.pose["orientation"]["phi"],
            pitch=self.pose["orientation"]["theta"],
            yaw=self.pose["orientation"]["psi"]
        )

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_publisher.publish(msg=msg)

    def cmd_vel_callback(self, twist):
        from rlb_controller.robot_parameters import BURGER_MAX_LIN_VEL, BURGER_MAX_ANG_VEL
        from rlb_controller.simulation_parameters import lin_vel_scaling_factor, ang_vel_scaling_factor

        def check_lin_vel(lin_vel):
            if lin_vel > BURGER_MAX_LIN_VEL:
                return BURGER_MAX_LIN_VEL * lin_vel_scaling_factor
            else:
                return lin_vel * lin_vel_scaling_factor

        def check_ang_vel(ang_vel):
            if ang_vel > BURGER_MAX_ANG_VEL:
                return BURGER_MAX_ANG_VEL * ang_vel_scaling_factor
            elif ang_vel < - BURGER_MAX_ANG_VEL:
                return - BURGER_MAX_ANG_VEL * ang_vel_scaling_factor
            else:
                return ang_vel * ang_vel_scaling_factor

        # -> Linear velocities
        self.vel["linear"]["x"] = check_lin_vel(self.vel["linear"]["x"] + twist.linear.x)
        self.vel["linear"]["y"] = check_lin_vel(self.vel["linear"]["y"] + twist.linear.y)
        self.vel["linear"]["z"] = check_lin_vel(self.vel["linear"]["z"] + twist.linear.z)

        # -> Angular velocities
        self.vel["angular"]["p"] = check_ang_vel(self.vel["angular"]["p"] + twist.angular.x)
        self.vel["angular"]["q"] = check_ang_vel(self.vel["angular"]["q"] + twist.angular.y)
        self.vel["angular"]["r"] = check_ang_vel(self.vel["angular"]["r"] + twist.angular.z)

    def step(self, dt, auto_dt=False):
        # -> Calculate dt based on ROS clock
        if auto_dt:
            new_stamp = self.sim_node.get_clock().now()
            dt = new_stamp - self.last_stamp
            dt = dt.nanoseconds * 10**(-8)
            self.last_stamp = new_stamp

        # -> Calculate new position
        th = self.pose["orientation"]["psi"] + self.vel["angular"]["r"]/2 * dt
        self.pose["position"]["x"] += (self.vel["linear"]["x"] * math.cos(th) - self.vel["linear"]["y"] * math.sin(th)) * dt
        self.pose["position"]["y"] += (self.vel["linear"]["x"] * math.sin(th) - self.vel["linear"]["y"] * math.cos(th)) * dt

        # -> Calculate new orientation
        self.pose["orientation"]["psi"] += dt * self.vel["angular"]["r"]

        if self.pose["orientation"]["psi"] > 180:
            self.pose["orientation"]["psi"] -= 360

            self.pose["orientation"]["psi"] = abs(self.pose["orientation"]["psi"])

        elif self.pose["orientation"]["psi"] < -180:
            self.pose["orientation"]["psi"] += 360

            self.pose["orientation"]["psi"] = -self.pose["orientation"]["psi"] 

        print("-", self.robot_id + ":")
        print("         x:", self.pose["position"]["x"], self.vel["linear"]["x"])
        print("         y:", self.pose["position"]["y"], self.vel["linear"]["y"])
        print("         psi:", self.pose["orientation"]["psi"], self.vel["angular"]["r"])
    
    @staticmethod
    def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return qx, qy, qz, qw