import numpy as np
import matplotlib.pyplot as plt
from roboter import Roboter
from workspace import Workspace

class Inverse_Kinematics:
    @staticmethod
    def inverse_kinematics(robot, x, y, z, elbow_up=True):
        l1, l2 = robot.l1, robot.l2

        base_x = x - robot.l / 2
        base_y = y
        base_z = z - robot.h
        
        distance = np.sqrt(np.power(base_x, 2) + 
                           np.power(base_y, 2) + 
                           np.power(base_z, 2))
        if distance >= (l1 + l2) or distance <= abs(l1 - l2):
            return None, None

        '''
        Elbow-Up
        epsilon = -1 (beta2, b < 0)

        Elbow-Down
        epsilon = 1 (beta2, b > 0)
        '''
        epsilon = -1
        if not elbow_up:
            epsilon = 1

        c = (np.power(base_x, 2) + 
             np.power(base_z, 2) - 
             np.power(l1, 2) - 
             np.power(l2, 2)) / (2 * l1)
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
    def decide_elbow_configuration(self, robot, x, y, z):
        beta1_up, beta2_up = self.inverse_kinematics(robot, x, y, z, elbow_up=True)
        beta1_down, beta2_down = self.inverse_kinematics(robot, x, y, z, elbow_up=False)
        
        if beta1_up is None and beta1_down is None:
            raise ValueError("Target point is not reachable.")
        
        if beta1_up is not None and beta1_down is not None:
            delta_up = abs(beta1_up + beta2_up)
            delta_down = abs(beta1_down + beta2_down)
            
            if delta_up <= delta_down:
                return beta1_up, beta2_up, "Elbow-Up"
            else:
                return beta1_down, beta2_down, "Elbow-Down"
            
        elif beta1_up is not None:
            return beta1_up, beta2_up, "Elbow-Up"
        else:
            return beta1_down, beta2_down, "Elbow-Down"

    @staticmethod
    def calculate_workspaces(self, robot):
        ws = Workspace()
        workspace, _, _ = ws.calculate_workspace(ws, robot, 0, 200)

        elbow_up_points = []
        elbow_down_points = []
        common_points = []

        for x, z in workspace:
            beta1_up, beta2_up = self.inverse_kinematics(robot, x, 0, z, elbow_up=True)
            beta1_down, beta2_down = self.inverse_kinematics(robot, x, 0, z, elbow_up=False)

            if beta1_up is not None and beta1_down is not None:
                common_points.append((x, z))
            elif beta1_up is not None:
                elbow_up_points.append((x, z))
            elif beta1_down is not None:
                elbow_down_points.append((x, z))

        elbow_up_points = np.array(elbow_up_points)
        elbow_down_points = np.array(elbow_down_points)
        common_points = np.array(common_points)
        workspace = np.array(workspace)

        return workspace, common_points, elbow_down_points, elbow_up_points

def plot():
    robot = Roboter()
    ik = Inverse_Kinematics()

    workspace, common_points, elbow_down_points, elbow_up_points = ik.calculate_workspaces(ik, robot)

    plt.figure(figsize=(8, 8))
    plt.scatter(workspace[:, 0], workspace[:, 1], color="grey", label="Calculated Workspace")
    plt.scatter(common_points[:, 0], common_points[:, 1], color="green", label="Common Workspace")
    plt.scatter(elbow_up_points[:, 0], elbow_up_points[:, 1], color="blue", label="Elbow-Up Only")
    plt.scatter(elbow_down_points[:, 0], elbow_down_points[:, 1], color="red", label="Elbow-Down Only")

    plt.title("Workspace of the Roboter with Alpha 0°, Elbow-Up and Elbow-Down Configuration")
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