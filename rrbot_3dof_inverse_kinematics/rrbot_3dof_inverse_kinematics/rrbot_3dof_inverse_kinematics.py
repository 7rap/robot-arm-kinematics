# joint_commander_server.py
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
import sys
import math
import numpy as np
from scipy.optimize import minimize


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


def inverse_kinematics(
    target_x,
    target_y,
    target_z,
    L1=1.0,
    L2=1.0,
    L3=1.0,
    BASE_Z=0.5,
    joint_limits=[(-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57)],
    tolerance=0.1  # 1 cm
):
    """
    Numerical IK using your exact forward_kinematics.
    Returns [j1, j2, j3] in radians.
    """
    def objective(q):
        j1, j2, j3 = q
        x, y, z = forward_kinematics(j1, j2, j3, L1, L2, L3, BASE_Z)
        return (x - target_x)**2 + (y - target_y)**2 + (z - target_z)**2

    # Common initial guesses
    initial_guesses = [
        [0.0, 0.0, 0.0],
        [0.0, 0.5, -0.5],
        [0.0, -0.5, 0.5],
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [math.atan2(target_y, target_x), 0.0, 0.0]  # smart j1 guess
    ]

    best_result = None
    best_error = float('inf')

    for guess in initial_guesses:
        try:
            res = minimize(
                objective,
                guess,
                method='L-BFGS-B',
                bounds=joint_limits,
                options={'ftol': 1e-9, 'gtol': 1e-9, 'maxiter': 1000}
            )
            if res.success and res.fun < best_error:
                best_error = res.fun
                best_result = res
        except Exception as e:
            continue

    if best_result is not None and best_error <= tolerance**2:
        j1, j2, j3 = best_result.x

        # Final validation with your FK
        x_fk, y_fk, z_fk = forward_kinematics(j1, j2, j3, L1, L2, L3, BASE_Z)
        error = math.sqrt((x_fk - target_x)**2 + (y_fk - target_y)**2 + (z_fk - target_z)**2)
        if error <= tolerance:
            return [float(j1), float(j2), float(j3)]

    raise ValueError(
        f"IK failed for target ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}). "
        f"Best error: {math.sqrt(best_error):.3f}m"
    )



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
            user_input = input(f"\nCurent position {position}. Enter coordinates (x y z [time=2]) or 'q' to quit:\n> ").strip()
            if user_input.lower() == 'q':
                break

            parts = user_input.split()
            if len(parts) < 3:
                print("Usage: x y z [time]")
                continue

            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            j1, j2, j3 = inverse_kinematics(x, y, z)
            position = forward_kinematics(j1, j2, j3)
            position = tuple(float(f"{p:.2f}") for p in position)
            print(f"Moving to {(x, y, z)}")
            time_sec = float(parts[3]) if len(parts) > 3 else 2.0

            node.send_joint_command(j1, j2, j3, time_sec)

        except ValueError as e:
            print(f"Invalid number. Try again: {e}")
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