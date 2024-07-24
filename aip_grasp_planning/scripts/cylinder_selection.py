from aip_grasp_planning_interfaces.msg import CylinderCombination
from utils_transform import Affine

class CylinderSelection:
    def __init__(self):
        pass
    
    # To-Do: 
        # Vorverarbeitung:
        
        # Affine Offsets wieder integrieren und ausgeben für die Greifpunktplanung 
        
    def choose_cylinder(packages_weights, packages_length, packages_width): #packages_height
        # Original source code from former student projects but adapted to match our ROS2 integration

        # piston stroke: 150 mm (Festo ADNGF-16-150-P-A)
        gripper_extension = 0.15 
        
        # cylinder messages:
        # msg_data type: [cylinder_back_left, cylinder_back_right, cylinder_front_left, cylinder_front_right]
        global msg_cylinder
        global index_msgs
        global tcps_cylinder
        msg0 = [0.0, 0.0, 0.0, 0.0]
        msg1 = [gripper_extension, 0.0, 0.0, 0.0]
        msg2 = [0.0, gripper_extension, 0.0, 0.0]
        msg3 = [0.0, 0.0, gripper_extension, 0.0]
        msg4 = [0.0, 0.0, 0.0, gripper_extension]
        msg5 = [gripper_extension, gripper_extension, 0.0, 0.0]
        msg6 = [0.0, gripper_extension, 0.0, gripper_extension]
        msg7 = [gripper_extension, gripper_extension, 0.0, gripper_extension]
        msg_cylinder = [msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7]

        # choice of gripper cylinders (assumption: package_length > package_width)
        # index_msg = index for msg_cylinder
        # tcp_cylinder = definition of the new tcp which is at the bottom of the extended cylinder (in the middle of the combination)
        
        cylinder_combination = CylinderCombination()
        cylinder_ids = []

        index_msgs = []
        tcps_cylinder = []
        # cylinder = [] #old version

        
        for i in range(len(packages_weights)): #packplan
            str_i = str(i)
            if packages_width[i] <= 0.051 and packages_weights[i] < 0.09:
                # index_msg = 3
                # index_msgs.append(index_msg)
                # cylinder.append("3")
                cylinder_combination.cylinder_ids = [3]
                cylinder_ids.append(cylinder_combination)
                tcp_cylinder =  Affine(translation = [-0.075, 0.106, 0.281], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)
            
            elif packages_width[i] > 0.051 and packages_width[i] <= 0.092 and packages_length[i] > 0.051 and packages_length[i] <= 0.129 and packages_weights[i] < 1.14:
                # index_msg = 1
                # index_msgs.append(index_msg)
                # cylinder.append("1")
                cylinder_combination.cylinder_ids = [1]
                cylinder_ids.append(cylinder_combination)            
                tcp_cylinder =  Affine(translation = [0.0485, 0.0815, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)

            elif packages_width[i] > 0.051 and packages_width[i] < 0.151 and packages_length[i] > 0.129 and packages_length[i] <= 0.232 and packages_weights[i] < 2.27:
                # index_msg = 2
                # index_msgs.append(index_msg)
                # cylinder.append("2")
                cylinder_combination.cylinder_ids = [2]
                cylinder_ids.append(cylinder_combination)  
                tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)
            
            elif packages_width[i] > 0.035 and packages_width[i] <= 0.151 and packages_length[i] > 0.165 and packages_length[i] <= 0.232 and packages_weights[i] < 3.10:
                # index_msg = 4 
                # index_msgs.append(index_msg)
                # cylinder.append("4")
                cylinder_combination.cylinder_ids = [4]
                cylinder_ids.append(cylinder_combination)  
                tcp_cylinder =  Affine(translation = [-0.0585, -0.055, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)

            elif packages_width[i] > 0.053 and packages_width[i] <= 0.182 and packages_length[i] > 0.232 and packages_weights[i] < 3.41:
                # index_msg = 5
                # index_msgs.append(index_msg)
                # cylinder.append("1,2")
                cylinder_combination.cylinder_ids = [1, 2]
                cylinder_ids.append(cylinder_combination)  
                tcp_cylinder =  Affine(translation = [0.049, -0.009, 0.299], rotation=[0.0, 0.0, 0.0])
                tcps_cylinder.append(tcp_cylinder)

            elif packages_width[i] > 0.151:
                if packages_length[i] <= 0.165 and packages_weights[i] < 2.27:
                    # index_msg = 2
                    # index_msgs.append(index_msg)
                    # cylinder.append("2")
                    cylinder_combination.cylinder_ids = [2]
                    cylinder_ids.append(cylinder_combination)  
                    tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
                    tcps_cylinder.append(tcp_cylinder)
                elif packages_length[i] > 0.165 and packages_length[i] <= 0.243 and packages_weights[i] < 5.37:
                    # index_msg = 6
                    # index_msgs.append(index_msg)
                    # cylinder.append("2,4")
                    cylinder_combination.cylinder_ids = [2, 4]
                    cylinder_ids.append(cylinder_combination)  
                    tcp_cylinder =  Affine(translation = [-0.001, -0.054, 0.299], rotation=[0.0, 0.0, 0.0]) 
                    tcps_cylinder.append(tcp_cylinder)
                elif packages_length[i] > 0.243 and packages_weights[i] < 6.51:
                    # index_msg = 7
                    # index_msgs.append(index_msg)
                    # cylinder.append("1,2,4")
                    cylinder_combination.cylinder_ids = [1, 2, 4]
                    cylinder_ids.append(cylinder_combination)  
                    tcp_cylinder =  Affine(translation = [-0.001, -0.015, 0.299], rotation=[0.0, 0.0, 0.0])
                    tcps_cylinder.append(tcp_cylinder)
                else: print("Error: dimensions of package " + str_i + " not allowable")
            else: print("Error: dimensions of package " + str_i + " not allowable")

        #packplan.insert(13, 'cylinder', cylinder)
        #Packplan.set_packplan(packplan)
        
        return index_msgs, cylinder_ids, tcps_cylinder