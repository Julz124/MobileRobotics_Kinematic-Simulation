import numpy as np
import matplotlib.pyplot as plt
from roboter import Roboter
from workspace import Workspace

class Inverse_Kinematics:
    @staticmethod
    def inverse_kinematics(robot, x, z, elbow_up=True):
        l1, l2 = robot.l1, robot.l2

        base_x = x - robot.l / 2
        base_z = z - robot.h
        
        # Ceck if point within reach
        distance = np.sqrt(np.power(base_x, 2) + np.power(base_z, 2))
        if distance > (l1 + l2) or distance < abs(l1 - l2):
            return None, None

        '''
        Elbow-Up
        epsilon = -1 (beta2, b < 0)

        Elbow-Down
        epsilon = 1 (beta2, b > 0)
        '''
        # define epsilon for elbow up/down configuration
        epsilon = -1
        if not elbow_up:
            epsilon = 1

        # calculate beta over trigonometry
        c = (np.power(base_x, 2) + np.power(base_z, 2) - np.power(l1, 2) - np.power(l2, 2)) / (2 * l1)
        # b = epsilon * np.sqrt(np.power(l2, 2) - np.power(c, 2))
        b_squared = np.power(l2, 2) - np.power(c, 2)
        if b_squared < 0:
            return None, None
        b = epsilon * np.sqrt(max(b_squared, 0))

        beta2 = np.arctan2(b, c)

        beta1 = np.arctan2(base_z, base_x) - np.arctan2(b, l1 + c)

        beta1_deg = np.degrees(beta1)
        beta2_deg = np.degrees(beta2)

        if not (robot.beta_1_limits[0] <= beta1_deg <= robot.beta_1_limits[1]):
            return None, None

        return beta1_deg, beta2_deg

    @staticmethod
    def decide_elbow_configuration(self, robot, x, z, current_beta1, current_beta2):
        beta1_up, beta2_up = self.inverse_kinematics(robot, x, z, elbow_up=True)
        beta1_down, beta2_down = self.inverse_kinematics(robot, x, z, elbow_up=False)
        
        if beta1_up is None and beta1_down is None:
            raise ValueError("Target point is not reachable.")
        
        if beta1_up is not None and beta1_down is not None:
            delta_up = abs(beta1_up - current_beta1) + abs(beta2_up - current_beta2)
            delta_down = abs(beta1_down - current_beta1) + abs(beta2_down - current_beta2)
            
            if delta_up <= delta_down:
                return beta1_up, beta2_up, "Elbow-Up"
            else:
                return beta1_down, beta2_down, "Elbow-Down"
        elif beta1_up is not None:
            return beta1_up, beta2_up, "Elbow-Up"
        else:
            return beta1_down, beta2_down, "Elbow-Down"

def plot():
    robot = Roboter()
    ik = Inverse_Kinematics()
    ws = Workspace()

    workspace, _, _ = ws.calculate_workspace(ws, robot, 200)

    with open("output/workspace.txt", "a") as f:
        print(workspace, file=f)

    elbow_up_points = []
    elbow_down_points = []
    common_points = []

    for x, z in workspace:
        beta1_up, beta2_up = ik.inverse_kinematics(robot, x, z, elbow_up=True)
        beta1_down, beta2_down = ik.inverse_kinematics(robot, x, z, elbow_up=False)

        if beta1_up is not None and beta1_down is not None:
            common_points.append((x, z))
        elif beta1_up is not None:
            elbow_up_points.append((x, z))
        elif beta1_down is not None:
            elbow_down_points.append((x, z))

    with open("output/common_workspace.txt", "a") as f:
        print(common_points, file=f)
    with open("output/elbow_up_workspace.txt", "a") as f:
        print(elbow_up_points, file=f)
    with open("output/elbow_down_workspace.txt", "a") as f:        
        print(elbow_down_points, file=f)

    elbow_up_points = np.array(elbow_up_points)
    elbow_down_points = np.array(elbow_down_points)
    common_points = np.array(common_points)
    workspace = np.array(workspace)

    plt.figure(figsize=(8, 8))
    plt.scatter(workspace[:, 0], workspace[:, 1], color="grey", label="Calculated Workspace")
    plt.scatter(common_points[:, 0], common_points[:, 1], color="green", label="Common Workspace")
    plt.scatter(elbow_up_points[:, 0], elbow_up_points[:, 1], color="blue", label="Elbow-Up Only")
    plt.scatter(elbow_down_points[:, 0], elbow_down_points[:, 1], color="red", label="Elbow-Down Only")

    plt.title("Workspace of the Roboter with Elbow-Up and Elbow-Down Configurations")
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    plt.show()


def decide_conf():
    robot = Roboter()
    ik = Inverse_Kinematics()

    current_beta1, current_beta2 = 90, 45
    
    target_x, target_y = 0.3, 0.4
    
    try:
        beta1, beta2, configuration = ik.decide_elbow_configuration(
            ik, robot, target_x, target_y, current_beta1, current_beta2
        )
        print(f"Selected Configuration: {configuration}")
        print(f"Joint Angles: Beta1 = {beta1:.2f}, Beta2 = {beta2:.2f}")
    except ValueError as e:
        print(e)

if __name__ == "__main__":
    plot()
    # decide_conf()

'''
A possible solution to automatically decide whether to use the elbow-up or 
elbow-down configuration could be based on a cost function that evaluates 
the suitability of each configuration for the desired target.

Possible Criteria:
1. Proximity to Current Joint Angles
2. Preferred Arm Orientation
3. Energy or Efficiency
4. Task-Specific Constraints
'''