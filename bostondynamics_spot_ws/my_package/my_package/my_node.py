import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import signal
import sys
from math import pi, sin

class EffortPublisher(Node):
    def __init__(self):
        super().__init__('effort_publisher')
        
        # Initialize publishers and subscribers
        self.effort_pub = self.create_publisher(
            Float64MultiArray, 
            '/effort_controller/commands', 
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Joint control parameters
        self.initial_joint_angles = np.zeros(12)
        self.current_joint_angles = np.zeros(12)
        self.goal_joint_angles = np.zeros(12)
        self.setup_joint_goals()
        
        # Trajectory parameters
        self.trajectory_start_time = None
        self.trajectory_duration = 1.0  # seconds
        self.trajectory_active = False
        
        # PID control parameters
        self.setup_pid_parameters()
        self.integral_max = 50.0
        
        # State variables
        self.current_joint_positions = np.zeros(12)
        self.current_joint_velocities = np.zeros(12)
        self.error_integrals = np.zeros(12)
        self.prev_errors = np.zeros(12)
        self.prev_time = self.get_clock().now()
        
        # System initialization
        self.start_time = self.get_clock().now()
        signal.signal(signal.SIGINT, self.signal_handler)
        self.initialized = False

    def setup_joint_goals(self):
        """Set the target joint angles for the robot."""
        x = -0.3  # Adjustment parameter
        self.goal_joint_angles = np.array([
            0.0, -0.0, 1.2,     # FLH, FRH, FRT
            0.0, 1.3, -2.0+x,    # BLH, BLT, BLC
            1.2, -0.0, -2.1+x,   # FLT, BRH, FLC
            -2.1+x, 1.3, -2.0+x  # FRC, BRT, BRC
        ])

    def setup_pid_parameters(self):
        """Configure PID control gains for each joint type."""
        # Proportional gains (scaled by 0.15)
        self.kp_cf = 225.0 * 0.15  # Calf front
        self.kp_tf = 262.5 * 0.15  # Thigh front
        self.kp_hf = 300.0 * 0.15  # Hip front
        self.kp_cb = 315.0 * 0.15  # Calf back
        self.kp_tb = 352.5 * 0.15  # Thigh back
        self.kp_hb = 390.0 * 0.15  # Hip back
        
        # Derivative gains (scaled by 0.2)
        self.kd_tf = 10.4 * 0.2
        self.kd_hf = 10.4 * 0.2
        self.kd_cf = 10.4 * 0.2
        self.kd_cb = 10.4 * 0.2
        self.kd_tb = 10.4 * 0.2
        self.kd_hb = 10.4 * 0.2

        # Integral gains (scaled by 0.1)
        self.ki_cf = 12.0 * 0.1
        self.ki_tf = 12.0 * 0.1
        self.ki_hf = 12.0 * 0.1
        self.ki_cb = 12.0 * 0.1
        self.ki_tb = 12.0 * 0.1
        self.ki_hb = 12.0 * 0.1

    def joint_state_callback(self, msg):
        """Handle incoming joint state messages and compute control efforts."""
        if not self.initialized:
            self.initialize_controller(msg)
        
        self.update_current_states(msg)
        
        if self.trajectory_active:
            self.update_trajectory()
        
        efforts = self.compute_pid_efforts()
        self.publish_efforts(efforts)

    def initialize_controller(self, msg):
        """Initialize the controller with current joint states."""
        self.initial_joint_angles = np.array(msg.position)
        self.current_joint_angles = np.array(msg.position)
        self.trajectory_start_time = self.get_clock().now()
        self.trajectory_active = True
        self.initialized = True

    def update_current_states(self, msg):
        """Update current joint positions and velocities."""
        self.current_joint_positions = np.array(msg.position)
        self.current_joint_velocities = np.array(msg.velocity)

    def update_trajectory(self):
        """Update the current joint angles based on trajectory planning."""
        current_time = (self.get_clock().now() - self.trajectory_start_time).nanoseconds * 1e-9
        
        if current_time <= self.trajectory_duration:
            self.current_joint_angles = self.cycloidal_trajectory(
                self.initial_joint_angles,
                self.goal_joint_angles,
                current_time,
                self.trajectory_duration
            )
        else:
            self.trajectory_active = False
            self.current_joint_angles = self.goal_joint_angles

    def cycloidal_trajectory(self, q0, qf, t, T):
        """Compute smooth trajectory using cycloidal motion profile.
        
        Args:
            q0: Initial joint angles
            qf: Final joint angles
            t: Current time
            T: Total trajectory duration
            
        Returns:
            Current target joint angles
        """
        if t >= T:
            return qf
        delta_q = qf - q0
        t_ratio = t / T
        return q0 + delta_q * (t_ratio - (1/(2*pi)) * sin(2*pi*t_ratio))

    def compute_pid_efforts(self):
        """Compute PID control efforts for all joints.
        
        Returns:
            numpy array of computed efforts
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time
        
        # Handle invalid time steps
        if dt <= 0 or dt > 0.1:
            dt = 0.01
        
        # Calculate errors
        position_errors = self.current_joint_angles - self.current_joint_positions
        velocity_errors = -self.current_joint_velocities 
        
        # Update and clamp integral terms
        self.error_integrals += position_errors * dt
        self.error_integrals = np.clip(
            self.error_integrals, 
            -self.integral_max, 
            self.integral_max
        )
        
        # Compute PID efforts for each joint
        efforts = np.array([
            # Front Left Hip (FLH)
            self.kp_hf * position_errors[0] + self.kd_hf * velocity_errors[0] + self.ki_hf * self.error_integrals[0],
            # Front Right Hip (FRH)
            self.kp_hf * position_errors[1] + self.kd_hf * velocity_errors[1] + self.ki_hf * self.error_integrals[1],
            # Front Right Thigh (FRT)
            self.kp_tf * position_errors[2] + self.kd_tf * velocity_errors[2] + self.ki_tf * self.error_integrals[2],
            # Back Left Hip (BLH)
            self.kp_hb * position_errors[3] + self.kd_hb * velocity_errors[3] + self.ki_hb * self.error_integrals[3],
            # Back Left Thigh (BLT)
            self.kp_tb * position_errors[4] + self.kd_tb * velocity_errors[4] + self.ki_tb * self.error_integrals[4],
            # Back Left Calf (BLC)
            self.kp_cb * position_errors[5] + self.kd_cb * velocity_errors[5] + self.ki_cb * self.error_integrals[5],
            # Front Left Thigh (FLT)
            self.kp_tf * position_errors[6] + self.kd_tf * velocity_errors[6] + self.ki_tf * self.error_integrals[6],
            # Back Right Hip (BRH)
            self.kp_hb * position_errors[7] + self.kd_hb * velocity_errors[7] + self.ki_hb * self.error_integrals[7],
            # Front Left Calf (FLC)
            self.kp_cf * position_errors[8] + self.kd_cf * velocity_errors[8] + self.ki_cf * self.error_integrals[8],
            # Front Right Calf (FRC)
            self.kp_cf * position_errors[9] + self.kd_cf * velocity_errors[9] + self.ki_cf * self.error_integrals[9],
            # Back Right Thigh (BRT)
            self.kp_tb * position_errors[10] + self.kd_tb * velocity_errors[10] + self.ki_tb * self.error_integrals[10],
            # Back Right Calf (BRC)
            self.kp_cb * position_errors[11] + self.kd_cb * velocity_errors[11] + self.ki_cb * self.error_integrals[11]
        ])
        
        return efforts

    def publish_efforts(self, efforts):
        """Publish computed efforts to the controller.
        
        Args:
            efforts: numpy array of effort values
        """
        effort_msg = Float64MultiArray()
        effort_msg.data = efforts.tolist()
        self.effort_pub.publish(effort_msg)
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully."""
        self.get_logger().info("Shutting down...")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    effort_controller = EffortPublisher()
    rclpy.spin(effort_controller)
    effort_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()