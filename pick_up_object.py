import numpy as np
import matplotlib.pyplot as plt
from roboter import Roboter
from workspace import Workspace
from inverse_kinematic import Inverse_Kinematics

class PickAndPlace:
    def __init__(self, robot):
        self.robot = robot
        self.ws = Workspace()
        self.ik = Inverse_Kinematics()

    def plot_arm_positions(self, positions, title):
        plt.figure(figsize=(8, 8))
        colors = ['blue', 'red']
        labels_added = set()

        workspace, common_points, elbow_down_points, elbow_up_points = self.ik.calculate_workspaces(self.ik, robot)

        plt.scatter(workspace[:, 0], workspace[:, 1], color="lightgrey", label="Calculated Workspace")
        plt.scatter(common_points[:, 0], common_points[:, 1], color="lightgreen", label="Common Workspace")
        plt.scatter(elbow_up_points[:, 0], elbow_up_points[:, 1], color="cornflowerblue", label="Elbow-Up Only")
        plt.scatter(elbow_down_points[:, 0], elbow_down_points[:, 1], color="lightcoral", label="Elbow-Down Only")
        
        for idx, pos in enumerate(positions):
            x, z, beta1, beta2, config = pos
            base_x = self.robot.l / 2
            base_z = self.robot.h

            x1 = base_x + self.robot.l1 * np.cos(np.radians(beta1))
            z1 = base_z + self.robot.l1 * np.sin(np.radians(beta1))
            x2 = x1 + self.robot.l2 * np.cos(np.radians(beta1 + beta2))
            z2 = z1 + self.robot.l2 * np.sin(np.radians(beta1 + beta2))

            if 'Arm Part 1' not in labels_added:
                plt.plot([base_x, x1], [base_z, z1], 'r-', label='Arm Part 1')
                labels_added.add('Arm Part 1')
            else:
                plt.plot([base_x, x1], [base_z, z1], 'r-')
                
            if 'Arm Part 2' not in labels_added:
                plt.plot([x1, x2], [z1, z2], 'g-', label='Arm Part 2')
                labels_added.add('Arm Part 2')
            else:
                plt.plot([x1, x2], [z1, z2], 'g-')
            
            plt.plot(x2, z2, 'o', color=colors[idx], label=f'{config} @({x2:.2f}, {z2:.2f})')

        plt.title(title)
        plt.xlabel('X Position (Roboter-KS)')
        plt.ylabel('Z Position (Roboter-KS)')
        plt.grid(True)
        plt.axis('equal')
        plt.legend(loc='upper right')
        plt.show()

    def simulate_pick_and_place(self, pick_pos, place_pos):
        pick_x, pick_z = pick_pos
        place_x, place_z = place_pos

        beta1_pick, beta2_pick, config_pick = self.ik.decide_elbow_configuration(
            self.ik, self.robot, pick_x, 0, pick_z)
        print(f'''Pick Configuration: {config_pick}, 
              Angles: β1={beta1_pick:.1f}°, β2={beta2_pick:.1f}°''')

        self.current_beta1 = beta1_pick
        self.current_beta2 = beta2_pick

        beta1_place, beta2_place, config_place = self.ik.decide_elbow_configuration(
            self.ik, self.robot, place_x, 0, place_z)
        print(f'''Place Configuration: {config_place}, 
              Angles: β1={beta1_place:.1f}°, β2={beta2_place:.1f}°''')

        self.plot_arm_positions([
            (pick_x, pick_z, beta1_pick, beta2_pick, config_pick),
            (place_x, place_z, beta1_place, beta2_place, config_place)
        ], "Automatic Elbow Configuration Selection")

if __name__ == "__main__":
    robot = Roboter()
    pick_pos = (0.9, -0.05)
    place_pos = (-0.2, 0.25)

    pap = PickAndPlace(robot)
    pap.simulate_pick_and_place(pick_pos, place_pos)