from ...postypes.configuration import configuration
import math
import numpy as np

# Constants
PI = math.pi
d_6 = 0.215
m = 0.33
n = 0.645
o = 0.115
a = 1.15
b = 1.22

class InvKinematics:

   
    R0_6 = np.zeros((3, 3))

    def get_inv_kinematics(self, _pos):
        
        # TODO: Implement inverse kinematics
         
        new_X = _pos[0]
        new_Y = _pos[1]
        new_Z = _pos[2]
        new_roll = _pos[3]
        new_pitch = _pos[4]
        new_yaw = _pos[5]
            
        r11 = math.cos(new_roll) * math.cos(new_pitch)
        r12 = (math.cos(new_roll) * math.sin(new_pitch) *
                   math.sin(new_yaw)) - (math.sin(new_roll) * math.cos(new_yaw))
        r13 = (math.cos(new_roll) * math.sin(new_pitch)* math.cos(new_yaw))+ (math.sin(new_roll) * math.sin(new_yaw))
        r21 = math.sin(new_roll) * math.cos(new_pitch)
        r22 = (math.sin(new_roll) * math.sin(new_pitch) * math.sin(new_yaw)) +(math.cos(new_roll) * math.cos(new_pitch))
        r23 = (math.sin(new_roll) * math.sin(new_pitch) *math.cos(new_yaw)) - (math.cos(new_roll) * math.sin(new_yaw))
        r31 = -math.sin(new_pitch)
        r32 = math.cos(new_pitch) * math.sin(new_yaw)
        r33 = math.cos(new_pitch) * math.cos(new_yaw)

        R0_6 = np.array([[r11, r12, r13],
                            [r21, r22, r23],
                            [r31, r32, r33]])
        
        # Compute the wrist center position Oc [Xc, Yc, Zc]
        X_c = new_X - (d_6 * R0_6[0, 2])
        Y_c = new_Y - (d_6 * R0_6[1, 2])
        Z_c = new_Z - (d_6 * R0_6[2, 2])

        d1 = np.sqrt(X_c ** 2 + Y_c ** 2)
        Px_dash = None
        Py_dash = Z_c - n

        #Joint 5 points
        print(f"P_x: {X_c}, P_y: {Y_c}, P_z: {Z_c}")
            
          
        theta1_arr, theta2_arr, theta3_arr, theta4_arr, theta5_arr, theta6_arr = [], [], [], [], [], []
        theta1_f, theta2_f, theta3_f = [], [], []
        
        if X_c==0 and Y_c==0:
             print("Shoulder Singularity detected")
        else:
            theta1_arr = find_theta1(X_c, Y_c)       

            for i in range(len(theta1_arr) ):
                d1 = np.abs(Y_c)

                if (-175 < theta1_arr[i] < 175 and theta1_arr[i] != -90 and theta1_arr[i] != 90):
                    if d1 > m:
                        Px_dash = d1 - m
                        theta2_arr, theta3_arr= forward_calc(Px_dash, Py_dash)    

                        for j in range(len(theta2_arr)):
                            if -140 < theta2_arr[j] < -5 and -120 < theta3_arr[j] < 168:
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                        Px_dash = m + d1

                        if -185 < theta1_arr[i] < 185:
                            theta2_arr, theta3_arr= backward_calc(Px_dash, Py_dash)
                            if len(theta2_arr) == 2 or len(theta2_arr) == 1:
                                break

                            #k = len(theta2_arr) - 1 if len(theta2_arr) == 3 else len(theta2_arr) - 2

                            for j in range(len(theta2_arr)):
                                if -140 < theta2_arr[j] < -5 and -120 < theta3_arr[j] < 168:
                                    print(f"Theta1: {theta1_arr[i+1]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                    theta1_f.append(theta1_arr[i+1])
                                    theta2_f.append(theta2_arr[j])
                                    theta3_f.append(theta3_arr[j])

                    

                    elif d1 < m:
                        Px_dash = m - d1
                        theta2_arr, theta3_arr= backward_calc(Px_dash, Py_dash)

                        for j in range(len(theta2_arr)):
                            if -140 < theta2_arr[j] < -5 and -120 < theta3_arr[j] < 168:
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                        Px_dash = m + d1

                        if -185 < theta1_arr[i] < 185:
                            theta2_arr, theta3_arr=backward_calc(Px_dash, Py_dash)
                            if len(theta2_arr) == 2 or len(theta2_arr) == 1:
                                break

                            #k = len(theta2_arr) - 1 if len(theta2_arr) == 3 else len(theta2_arr) - 2

                            for j in range(len(theta2_arr)):
                                if -140 < theta2_arr[j] < -5 and -120 < theta3_arr[j] < 168:
                                    print(f"Theta1: {theta1_arr[i+1]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                    theta1_f.append(theta1_arr[i+1])
                                    theta2_f.append(theta2_arr[j])
                                    theta3_f.append(theta3_arr[j])

                elif theta1_arr[i] == -90:
                    if Y_c > m:
                        d1 = Y_c
                        Px_dash = d1 - m
                        theta2_arr, theta3_arr=forward_calc(Px_dash, Py_dash)
            
                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                    
                    elif Y_c < m:
                        d1 = Y_c
                        Px_dash = m - d1
                        theta2_arr, theta3_arr=backward_calc(Px_dash, Py_dash)
        
                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                    
            
                elif theta1_arr[i] == 90:
                    if Y_c > m:
                        d1 = Y_c
                        Px_dash = d1 - m
                        theta2_arr, theta3_arr=forward_calc(Px_dash, Py_dash)
            
                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])                  
        
                    elif Y_c < m:
                        d1 = Y_c
                        Px_dash = m - d1
                        theta2_arr, theta3_arr=backward_calc(Px_dash, Py_dash )
            
                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])



                elif -185 < theta1_arr[i] < -175:
                    if d1 > m:
                        Px_dash = d1 - m
                        theta2_arr, theta3_arr=forward_calc(Px_dash, Py_dash )
            
                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                        if -185 < theta1_arr[i] < 185:
                            theta2_arr, theta3_arr=forward_calc(Px_dash, Py_dash )
                            if len(theta2_arr) == 2 or len(theta2_arr) == 1:
                                break

                            #k = len(theta2_arr) - 1 if len(theta2_arr) == 3 else len(theta2_arr) - 2
                            for j in range(len(theta2_arr)):
                                if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                    print(f"Theta1: {theta1_arr[i+1]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                    theta1_f.append(theta1_arr[i+1])
                                    theta2_f.append(theta2_arr[j])
                                    theta3_f.append(theta3_arr[j])

                    

                    elif d1 < m:
                        Px_dash = m - d1
                        theta2_arr, theta3_arr=backward_calc(Px_dash, Py_dash)

                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                        if -185 < theta1_arr[i] < 185:
                            theta2_arr, theta3_arr=backward_calc(Px_dash, Py_dash)
                            if len(theta2_arr) == 2 or len(theta2_arr) == 1:
                                break

                            #k = len(theta2_arr) - 1 if len(theta2_arr) == 3 else len(theta2_arr) - 2
                            for j in range(len(theta2_arr)):
                                if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                    print(f"Theta1: {theta1_arr[i+1]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                    theta1_f.append(theta1_arr[i+1])
                                    theta2_f.append(theta2_arr[j])
                                    theta3_f.append(theta3_arr[j])

                elif 175 < theta1_arr[i] < 185:
                    if d1 > m:
                        Px_dash = d1 - m
                        theta2_arr, theta3_arr=forward_calc(Px_dash, Py_dash )
            
                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                        if -185 < theta1_arr[i] < 185:
                            theta2_arr, theta3_arr=forward_calc(Px_dash, Py_dash )
                            if len(theta2_arr) == 2 or len(theta2_arr) == 1:
                                break

                            #k = len(theta2_arr) - 1 if len(theta2_arr) == 3 else len(theta2_arr) - 2
                            for j in range(len(theta2_arr)):
                                if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                    print(f"Theta1: {theta1_arr[i+1]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                    theta1_f.append(theta1_arr[i+1])
                                    theta2_f.append(theta2_arr[j])
                                    theta3_f.append(theta3_arr[j])


                    elif d1 < m:
                        Px_dash = m - d1
                        theta2_arr, theta3_arr=backward_calc(Px_dash, Py_dash)

                        for j in range(len(theta2_arr)):
                            if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                print(f"Theta1: {theta1_arr[i]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                theta1_f.append(theta1_arr[i])
                                theta2_f.append(theta2_arr[j])
                                theta3_f.append(theta3_arr[j])

                        if -185 < theta1_arr[i] < 185:
                            theta2_arr, theta3_arr=backward_calc(Px_dash, Py_dash)
                            if len(theta2_arr) == 2 or len(theta2_arr) == 1:
                                break

                            #k = len(theta2_arr) - 1 if len(theta2_arr) == 3 else len(theta2_arr) - 2
                            for j in range(len(theta2_arr)):
                                if (-140 < theta2_arr[j] < -5) and (-120 < theta3_arr[j] < 168):
                                    print(f"Theta1: {theta1_arr[i+1]}, Theta2: {theta2_arr[j]}, Theta3: {theta3_arr[j]}")
                                    theta1_f.append(theta1_arr[i+1])
                                    theta2_f.append(theta2_arr[j])
                                    theta3_f.append(theta3_arr[j])

            
        
        inv_R0_3 = np.zeros((3, 3))
        R3_6 = np.zeros((3, 3))

        a_x, a_y, a_z, n_z, s_z = 0, 0, 0, 0, 0
        th_1, th_2, th_3 = 0, 0, 0

        solutions=[]
        
        for j in range(len(theta1_f)):
        #Here we are finding base rotational matrix R0_3
            inv_R0_3 = find_rotational_matrix_R0_3(theta1_f[j], theta2_f[j], theta3_f[j])

            R3_6 = np.zeros((3, 3))

            # Here we are multiplying first two rotational matrices R0_3 and R0_6
            R3_6 = np.dot(inv_R0_3, R0_6)
            
            # Here are finding the angles theta 4, theta 5, theta 6
            a_x = R3_6[0][2]
            a_y = R3_6[1][2]
            a_z = R3_6[2][2]
            n_z = R3_6[2][0]
            s_z = R3_6[2][1]
            
            theta4_arr, theta5_arr, theta6_arr = find_theta4_theta5_theta6(a_x, a_y, a_z, n_z, s_z)           
            
            for t in range(len(theta5_arr)):
              if theta5_arr[t]==0:
                print("wrist singularity detected")
              else:
                print("*****CONFIGURATION******\n")
                for i in range(0, len(theta4_arr)):
                     print(f"[ {theta1_f[j] * np.pi / 180} {theta2_f[j] * np.pi / 180} {theta3_f[j] * np.pi / 180} "
                       f"{theta4_arr[i] * np.pi / 180} {theta5_arr[i] * np.pi / 180} {theta6_arr[i] * np.pi / 180} ]")
                     solutions.append(configuration([theta1_f[j] * np.pi / 180, theta2_f[j] * np.pi / 180, theta3_f[j] * np.pi / 180,
                                 theta4_arr[i] * np.pi / 180, theta5_arr[i] * np.pi / 180, theta6_arr[i] * np.pi / 180]))
              break
        return solutions


        # solutions = []
        # solutions.append(configuration([0, 0, 1, 0, 0, 0]))
        # solutions.append(configuration([1/8. * math.pi, 0, 1, 0, 0, 0]))
        # solutions.append(configuration([2/8. * math.pi, 0, 1, 0, 0, 0]))
        # solutions.append(configuration([3/8. * math.pi, 0, 1, 0, 0, 0]))
        # solutions.append(configuration([4/8. * math.pi, 0, 1, 0, 0, 0]))
        # solutions.append(configuration([5/8. * math.pi, 0, 1, 0, 0, 0]))
        # solutions.append(configuration([6/8. * math.pi, 0, 1, 0, 0, 0]))
        # solutions.append(configuration([7/8. * math.pi, 0, 1, 0, 0, 0]))

        #return solutions

def find_theta1(X_c, Y_c):
        
    theta1 = -math.atan2(Y_c, X_c) * 180 / math.pi
    arr = []

    if -175 < theta1 < 175:
        if X_c > 0 and Y_c < 0:
            arr.extend([theta1, theta1 + 180, theta1 - 180])
        elif X_c < 0 and Y_c < 0:
            arr.extend([theta1, theta1 + 180, theta1 - 180])
        elif X_c < 0 and Y_c > 0:
            arr.extend([theta1, theta1 + 180, theta1 - 180])
        elif X_c > 0 and Y_c > 0:
            arr.extend([theta1, theta1 + 180, theta1 - 180])
        elif X_c == 0 and Y_c > 0:
            arr.extend([theta1, theta1 + 180])
        elif X_c == 0 and Y_c < 0:
            arr.extend([theta1, theta1 - 180])
        elif -185 < theta1 < -175:
            case1, case2, case3 = theta1, theta1 + 180, theta1 - 180
            arr.extend([case1, case2, case3])
        elif 175 < theta1 < 185:
            case1, case2, case3 = theta1, theta1 + 180, theta1 - 180
            arr.extend([case1, case2, case3])

    return arr
    

      
def forward_calc(Px_dash, Py_dash):
    arr2 = []
    arr3 = []
    d_2 = math.sqrt(b**2 + o**2)   #d2 is the new link instead of b
    d_3 = math.sqrt(Px_dash**2 + Py_dash**2)
    if a+d_2>d_3:
        beta1 = math.acos(((d_3**2) - (a**2) - (d_2**2)) /(-2 * a * d_2)) * 180 / PI     #angle between d_2 and a
        alpha1 = math.asin(math.sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI          #angle between d_3 and Px_dash
        alpha2 = math.asin(Py_dash / d_3) * 180 / PI                                     #angle between d_3 and a
        # Forward elbow down
        theta2 = -(alpha2 - alpha1)
        theta3 = beta1 - (math.asin(b / d_2) * 180 / PI) - 90

        if (-140 < theta2 < -5) and (-120 < theta3 < 168):
            arr2.append(theta2)
            arr3.append(theta3)


        # Forward elbow up
        theta2 = -(alpha1 + alpha2)
        theta3 = 360 - beta1 - (math.asin(b / d_2) * 180 / PI) - 90

        if (-140 < theta2 < -5) and (-120 < theta3 < 168):
            arr2.append(theta2)
            arr3.append(theta3)
        else: 
            print("***VALUE ERROR***")
    return arr2, arr3

def backward_calc(Px_dash, Py_dash):
    arr4 = []
    arr5 = []
    d_2 = math.sqrt(b**2 + o**2)
    d_3 = math.sqrt(Px_dash**2 + Py_dash**2)
    if a+d_2>d_3:
        beta1 = math.acos(((d_3**2) - (a**2) - (d_2**2)) /(-2 * a * d_2)) * 180 / PI
        alpha1 = math.asin(math.sin(beta1 * PI / 180) * (d_2 / d_3)) * 180 / PI
        alpha2 = math.asin(Py_dash / d_3) * 180 / PI
        # Backward elbow down
        theta2 = (alpha2 - alpha1) - 180
        theta3 = 270 - beta1 - (math.asin(b / d_2) * 180 / PI)
        if (-140 < theta2 < -5) and (-120 < theta3 < 168):
            arr4.append(theta2)
            arr5.append(theta3)
        # Backward elbow up
        theta2 = (alpha1 + alpha2) - 180
        theta3 = -(90 - (beta1 - (math.asin(b / d_2) * 180 / PI)))
        if (-140 < theta2 < -5) and (-120 < theta3 < 168):
            arr4.append(theta2)
            arr5.append(theta3)
    else:
      print("***VALUE ERROR***")
    
    return arr4, arr5


def find_rotational_matrix_R0_3(theta_1, theta_2, theta_3):
    alpha = [180, 90, 0, 90]                       #Represents the twist angle about the Z axis between consecutive joint axes.
    theta = [0, theta_1, theta_2, theta_3 - 90]    #Represents the joint angle about the common normal between consecutive joint axes.
    R0_3 = np.eye(3)
    for th, al in zip(theta, alpha):
        c = math.cos(th * math.pi / 180)
        s = math.sin(th * math.pi / 180)
        nx, sx, ax = c, -s * math.cos(al * math.pi / 180), s * math.sin(al * math.pi / 180)
        ny, sy, ay = s, c * math.cos(al * math.pi / 180), -c * math.sin(al * math.pi / 180)
        nz, sz, az = 0, math.sin(al * math.pi / 180), math.cos(al * math.pi / 180)
        Ri = np.array([[nx, sx, ax], [ny, sy, ay], [nz, sz, az]])                       #Denavit-Hartenberg Convention
        R0_3 = np.dot(R0_3, Ri)
        
    print("***********Rotational Matrix R0_3***********:\n", R0_3)
    return R0_3

def find_theta4_theta5_theta6(a_x, a_y, a_z, n_z, s_z):
    arr4, arr5, arr6 = [], [], []
    # Computing theta4
    theta_4 = math.atan2(-a_y, -a_x) * 180 / math.pi
    if (theta_4 - 360) > -350:
        arr4.extend([theta_4, theta_4, theta_4 - 360, theta_4 - 360])
    elif (theta_4 + 360) < 350:
        arr4.extend([theta_4, theta_4, theta_4 + 360, theta_4 + 360])
    # Computing theta5 and theta6
    theta_5 = math.atan2(math.sqrt(1 - (a_z * a_z)), -a_z) * 180 / math.pi
    theta_6 = math.atan2(s_z, n_z) * 180 / math.pi
    if (theta_6 - 360) > -350:
        arr5.extend([theta_5] * 4)
        arr6.extend([theta_6, theta_6 - 360, theta_6, theta_6 - 360])
    elif (theta_6 + 360) < 350:
        arr5.extend([theta_5] * 4)
        arr6.extend([theta_6, theta_6 + 360, theta_6, theta_6 + 360])

    theta_5 = -theta_5

    if theta_4 > 0:
        theta_4 -= 180
    else:
        theta_4 += 180

    if (theta_4 - 360) > -350:
        arr4.extend([theta_4, theta_4, theta_4 - 360, theta_4 - 360])
    elif (theta_4 + 360) < 350:
        arr4.extend([theta_4, theta_4, theta_4 + 360, theta_4 + 360])

    if theta_6 > 0:
        theta_6 -= 180
    else:
        theta_6 += 180

    if (theta_6 - 360) > -350:
        arr5.extend([theta_5] * 4)
        arr6.extend([theta_6, theta_6 - 360, theta_6, theta_6 - 360])
    elif (theta_6 + 360) < 350:
        arr5.extend([theta_5] * 4)
        arr6.extend([theta_6, theta_6 + 360, theta_6, theta_6 + 360])

    return arr4, arr5, arr6
