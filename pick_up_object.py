import numpy as np
import matplotlib.pyplot as plt
from roboter import Roboter

def inverse_kinematics(robot, x, y):
    l1 = robot.l1 
    l2 = robot.l2

    d = np.sqrt(x**2 + y**2)
    if d > (l1 + l2) or d < abs(l1 - l2):
        raise ValueError("Target position is outside the reachable workspace.")

    cos_beta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    sin_beta2 = np.sqrt(1 - cos_beta2**2)

    beta2 = np.arctan2(sin_beta2, cos_beta2)

    k1 = l1 + l2 * cos_beta2
    k2 = l2 * sin_beta2

    beta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    beta1_deg = np.degrees(beta1)
    beta2_deg = np.degrees(beta2)

    if not (robot.beta_1_limits[0] <= beta1_deg <= robot.beta_1_limits[1]):
        raise ValueError("Beta1 exceeds joint limits.")

    return beta1_deg, beta2_deg

def simulate_pickup_and_placement(robot):
    pickup_position = (0.9, -0.05)
    place_position = (-0.2, 0.25)

    # Inverse kinematics for both positions
    beta1_pickup, beta2_pickup = inverse_kinematics(robot, *pickup_position)
    beta1_place, beta2_place = inverse_kinematics(robot, *place_position)

    # Generate alpha trajectory (no limits for alpha)
    alpha_trajectory = np.linspace(0, 360, 100)

    # Generate joint trajectories (linear interpolation)
    beta1_trajectory = np.linspace(beta1_pickup, beta1_place, 100)
    beta2_trajectory = np.linspace(beta2_pickup, beta2_place, 100)

    # Plot arm positions
    plt.figure(figsize=(10, 6))

    # Pickup position
    plt.subplot(1, 2, 1)
    plot_arm(robot, beta1_pickup, beta2_pickup, "Pickup Position")

    # Placement position
    plt.subplot(1, 2, 2)
    plot_arm(robot, beta1_place, beta2_place, "Placement Position")

    plt.tight_layout()
    plt.show()

    # Plot joint trajectories
    plt.figure(figsize=(8, 5))
    plt.plot(alpha_trajectory, label="Alpha")
    plt.plot(beta1_trajectory, label="Beta1")
    plt.plot(beta2_trajectory, label="Beta2")
    plt.xlabel("Time Steps")
    plt.ylabel("Angle (Degrees)")
    plt.title("Joint Angle Trajectories")
    plt.legend()
    plt.grid()
    plt.show()

def plot_arm(robot, beta1, beta2, title):
    """Plot the arm configuration for given joint angles."""
    l1, l2 = robot.l1, robot.l2

    beta1_rad = np.radians(beta1)
    beta2_rad = np.radians(beta2)

    joint1_x = l1 * np.cos(beta1_rad)
    joint1_y = l1 * np.sin(beta1_rad)

    end_effector_x = joint1_x + l2 * np.cos(beta1_rad + beta2_rad)
    end_effector_y = joint1_y + l2 * np.sin(beta1_rad + beta2_rad)

    plt.plot([0, joint1_x, end_effector_x], [0, joint1_y, end_effector_y], marker="o")
    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid()
    plt.axis("equal")

if __name__ == "__main__":
    robot = Roboter()
    simulate_pickup_and_placement(robot)