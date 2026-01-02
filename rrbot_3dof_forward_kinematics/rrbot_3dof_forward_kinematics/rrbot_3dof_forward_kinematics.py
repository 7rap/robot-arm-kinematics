# joint_commander_server.py
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
import sys
import math


def forward_kinematics(j1, j2, j3, L1=1.0, L2=1.0, L3=1.0, BASE_Z=0.5):
    """
    Forward kinematics for your Z-Y-Y 3-DOF arm.
    Returns (x, y, z) of end-effector in world frame.
    """
    
    x = -1 * L1 * math.cos(math.pi / 2) * math.sin(j1) + \
        -1 * L2 * math.cos(math.pi / 2 + j2) * math.sin(j1) + \
        -1 * L3 * math.cos(math.pi / 2 + j2 + j3) * math.sin(j1)
    
    y = L1 * math.cos(math.pi / 2) * math.cos(j1) + \
        L2 * math.cos(math.pi / 2 + j2) * math.cos(j1) + \
        L3 * math.cos(math.pi / 2 + j2 + j3) * math.cos(j1)
    
    z = BASE_Z + L1 * math.sin(math.pi / 2) + \
        L2 * math.sin(math.pi / 2 + j2) + \
        L3 * math.sin(math.pi / 2 + j2 + j3)
    return x, y, z


class JointCommanderServer(Node):
    def __init__(self):
        super().__init__('joint_commander_server')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/position_trajectory_controller/joint_trajectory',
            10
        )
        self.get_logger().info("Joint Commander Server started. Ready for input.")

    def send_joint_command(self, j1, j2, j3, duration_sec=2.0):
        traj = JointTrajectory()
        traj.joint_names = ['j1', 'j2', 'j3']
        
        point = JointTrajectoryPoint()
        point.positions = [j1, j2, j3]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        
        traj.points.append(point)
        self.publisher.publish(traj)
        self.get_logger().info(f"Sent command: j1={j1:.3f}, j2={j2:.3f}, j3={j3:.3f} (time={duration_sec}s)")

def input_loop(node):
    j1 = 0.0
    j2 = 0.0
    j3 = 0.0
    position = forward_kinematics(j1, j2, j3)
    position = tuple(float(f"{p:.2f}") for p in position)
    while rclpy.ok():
        try:
            user_input = input(f"\nCurent position {position}. Enter joint angles (j1 j2 j3 [time=2]) or 'q' to quit:\n> ").strip()
            if user_input.lower() == 'q':
                break

            parts = user_input.split()
            if len(parts) < 3:
                print("Usage: j1 j2 j3 [time]")
                continue

            j1 = float(parts[0])
            j2 = float(parts[1])
            j3 = float(parts[2])
            position = forward_kinematics(j1, j2, j3)
            position = tuple(float(f"{p:.2f}") for p in position)
            print(f"Moving to {position}")
            time_sec = float(parts[3]) if len(parts) > 3 else 2.0

            node.send_joint_command(j1, j2, j3, time_sec)

        except ValueError:
            print("Invalid number. Try again.")
        except KeyboardInterrupt:
            break

    print("Shutting down...")

def main():
    rclpy.init()
    node = JointCommanderServer()

    # Run input loop in a separate thread so ROS can spin
    input_thread = threading.Thread(target=input_loop, args=(node,), daemon=True)
    input_thread.start()

    try:
        # Keep ROS node alive
        while input_thread.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()