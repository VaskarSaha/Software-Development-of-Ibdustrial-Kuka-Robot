import numpy as np
from ...postypes.configuration import configuration
from ...postypes.SixDPos import SixDPos


class FwKinematics:
    def get_fw_kinematics(self, config: configuration) -> SixDPos:
        try:
            # Extract joint angles from the configuration
            new_theta1, new_theta2, new_theta3, new_theta4, new_theta5, new_theta6 = config

            # Initialize Parameters
            DHC_Matrix = np.array([
                [0, np.pi, 0, 0.645],                        # Theta (CS0 -> CS1)
                [new_theta1, np.pi/2, 0.330, 0],             # CS1 -> CS2
                [new_theta2, 0, 1.150, 0],                  # CS2 -> CS3
                [-np.pi/2 + new_theta3, np.pi/2, 0.115, 0],  # CS3 -> CS4
                [new_theta4, -np.pi/2, 0, -1.220],            # CS4 -> CS5
                [new_theta5, np.pi/2, 0, 0],                 # CS5 -> CS6
                [np.pi + new_theta6, np.pi, 0, -0.215]         # CS6 -> CS7
            ])

            # Calculate transformation matrices
            result_matrix = np.eye(4)

            for i in range(7):
                theta, alpha, r, d = DHC_Matrix[i, 0], DHC_Matrix[i,1], DHC_Matrix[i, 2], DHC_Matrix[i, 3]
                transformation_matrix = self.construct_transformation(
                    theta, alpha, r, d)

                result_matrix = np.dot(result_matrix, transformation_matrix)

            x, y, z = result_matrix[:3, 3]
            print(f"x: {x}, y: {y}, z: {z}")
            
            if DHC_Matrix[5, 0]== 1.5707963267949:
                print("Gimble lock arrised")
            else:   
                roll, pitch, yaw = self.calculate_orientation(result_matrix)
                print(f"roll: {roll}, pitch: {pitch}, yaw: {yaw}")
                        
            return SixDPos(x, y, z, roll, pitch, yaw)
        
        except Exception as e:
            print(f"Error in forward kinematics computation: {e}")
            # Return a default position in case of an error
            return SixDPos(1.757, 0.0, 1.91, 0, np.pi, 0)

    def construct_transformation(self, theta, alpha, r, d):
        # Implementation of transformation matrix construction
        transformation_matrix = np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha),
             np.sin(theta) * np.sin(alpha), r * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -
             np.cos(theta) * np.sin(alpha), r * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

        return transformation_matrix

    def calculate_orientation(self, matrix):
        # Implementation of orientation calculation
        roll, pitch, yaw = None, None, None

        if matrix[2, 0] == -1:
            roll = -np.arctan2(-matrix[1, 2], matrix[1, 1])
            pitch = np.pi / 2
            yaw = 0
        elif matrix[2, 0] == 1:
            roll = np.arctan2(-matrix[1, 2], matrix[1, 1])
            pitch = -np.pi / 2
            yaw = 0
        else:
            roll = np.arctan2(matrix[1, 0], matrix[0, 0])
            pitch = np.arctan2(-matrix[2, 0],
                               np.sqrt(matrix[2, 1]**2 + matrix[2, 2]**2))
            yaw = np.arctan2(matrix[2, 1], matrix[2, 2])

        return roll, pitch, yaw
