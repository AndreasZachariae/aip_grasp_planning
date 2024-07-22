class CylinderSelection:
    def __init__(self, suction_injectors):
        self.suction_injectors = suction_injectors

    # To-Do: 
        # Vorverarbeitung:
            # Gewicht, Länge, Breite der Pakete in ein Array "Packplan" schreiben -> Herkunft: PackageSequence aus PackPlanning
        
        # Affine Offsets wieder integrieren und ausgeben für die Greifpunktplanung 
        
    def choose_cylinder(packplan):
                
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
        index_msgs = []
        tcps_cylinder = []
        cylinder = []
        
        for i in range(len(packplan)):
            str_i = str(i)
            if packplan.width[i] <= 0.051 and packplan.weight[i] < 0.09:
                index_msg = 3
                index_msgs.append(index_msg)
                cylinder.append("3")
                # tcp_cylinder =  Affine(translation = [-0.075, 0.106, 0.281], rotation=[0.0, 0.0, 0.0])
                # tcps_cylinder.append(tcp_cylinder)
            elif packplan.width[i] > 0.051 and packplan.width[i] <= 0.092 and packplan.length[i] > 0.051 and packplan.length[i] <= 0.129 and packplan.weight[i] < 1.14:
                index_msg = 1
                index_msgs.append(index_msg)
                cylinder.append("1")
                # tcp_cylinder =  Affine(translation = [0.0485, 0.0815, 0.299], rotation=[0.0, 0.0, 0.0])
                # tcps_cylinder.append(tcp_cylinder)
            elif packplan.width[i] > 0.051 and packplan.width[i] < 0.151 and packplan.length[i] > 0.129 and packplan.length[i] <= 0.232 and packplan.weight[i] < 2.27:
                index_msg = 2
                index_msgs.append(index_msg)
                cylinder.append("2")
                # tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
                # tcps_cylinder.append(tcp_cylinder)
            elif packplan.width[i] > 0.035 and packplan.width[i] <= 0.151 and packplan.length[i] > 0.165 and packplan.length[i] <= 0.232 and packplan.weight[i] < 3.10:
                index_msg = 4 
                index_msgs.append(index_msg)
                cylinder.append("4")
                # tcp_cylinder =  Affine(translation = [-0.0585, -0.055, 0.299], rotation=[0.0, 0.0, 0.0])
                # tcps_cylinder.append(tcp_cylinder)
            elif packplan.width[i] > 0.053 and packplan.width[i] <= 0.182 and packplan.length[i] > 0.232 and packplan.weight[i] < 3.41:
                index_msg = 5
                index_msgs.append(index_msg)
                cylinder.append("1,2")
                # tcp_cylinder =  Affine(translation = [0.049, -0.009, 0.299], rotation=[0.0, 0.0, 0.0])
                # tcps_cylinder.append(tcp_cylinder)
            elif packplan.width[i] > 0.151:
                if packplan.length[i] <= 0.165 and packplan.weight[i] < 2.27:
                    index_msg = 2
                    index_msgs.append(index_msg)
                    cylinder.append("2")
                    # tcp_cylinder =  Affine(translation = [0.0505, -0.0495, 0.299], rotation=[0.0, 0.0, 0.0])
                    # tcps_cylinder.append(tcp_cylinder)
                elif packplan.length[i] > 0.165 and packplan.length[i] <= 0.243 and packplan.weight[i] < 5.37:
                    index_msg = 6
                    index_msgs.append(index_msg)
                    cylinder.append("2,4")
                    # tcp_cylinder =  Affine(translation = [-0.001, -0.054, 0.299], rotation=[0.0, 0.0, 0.0]) 
                    # tcps_cylinder.append(tcp_cylinder)
                elif packplan.length[i] > 0.243 and packplan.weight[i] < 6.51:
                    index_msg = 7
                    index_msgs.append(index_msg)
                    cylinder.append("1,2,4")
                    # tcp_cylinder =  Affine(translation = [-0.001, -0.015, 0.299], rotation=[0.0, 0.0, 0.0])
                    # tcps_cylinder.append(tcp_cylinder)
                else: print("Error: dimensions of package " + str_i + " not allowable")
            else: print("Error: dimensions of package " + str_i + " not allowable")

        #packplan.insert(13, 'cylinder', cylinder)
        #Packplan.set_packplan(packplan)
        
        return index_msgs, tcps_cylinder