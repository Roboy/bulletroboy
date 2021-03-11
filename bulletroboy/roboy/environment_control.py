import pybullet as p
import pybullet_data
import rclpy
import math

import time

from threading import Timer

class EnvironmentCtrl():
    def __init__(self):
        """
        This class handles the simulation of the environment around roboy.
        """
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        self.castle = p.loadURDF("samurai.urdf")
        
        self.o = 0
        self.sig = 1

        #Size parameter
        self.size_id = p.addUserDebugParameter("Size " , 1, 25, 8.2)

        #object position parameters
        self.x = -0.5
        self.y = -0.8
        self.z = 0.5
        self.x_id = p.addUserDebugParameter("Position, x " , -5, 5, 0.6)
        self.y_id = p.addUserDebugParameter("Position, y " , -5, 5, 0.0)
        self.z_id = p.addUserDebugParameter("Position, z " , -5, 5, 1.8)
        
        #object orientation parameters
        self.or_x = 0
        self.or_y = 0
        self.or_z = 0
        self.or_x_id = p.addUserDebugParameter("Orientation, x-Axis " , -90, 90, 14)
        self.or_y_id = p.addUserDebugParameter("Orientation, y-Axis " , -90, 90, 0)
        self.or_z_id = p.addUserDebugParameter("Orientation, z-Axis " , -90, 90, 0)

        #object mass parameter
        self.mass = 0.1
        self.mass_id = p.addUserDebugParameter("Mass " , 0, 5, 3.0)

        #Number of times add cube button was clicked
        self.add_cube = 0
        self.add_cube_id = p.addUserDebugParameter("Add cube " , 1, 0, 0)
        
        #Adding fixed cube is needed to have same collision for debugging
        #Number of times add fixed cube button was clicked
        self.add_fixed_cube = 0
        self.add_fixed_cube_id = p.addUserDebugParameter("Add fixed cube " , 1, 0, 0)

        #Number of times delete cube button was clicked
        self.add_model = 0
        self.add_model_id = p.addUserDebugParameter("Add model from pybullet_data " , 1, 0, 0)

        #Number of times delete cube button was clicked
        self.delete_obj = 0
        self.delete_obj_id = p.addUserDebugParameter("Delete last object " , 1, 0, 0)

        #Number of times test1 button was clicked
        self.test1 = 0
        self.test1_id = p.addUserDebugParameter("Drop cube 1kg: " , 1, 0, 0)
        #Number of times test2 button was clicked
        self.test2 = 0
        self.test2_id = p.addUserDebugParameter("Drop cube 3kg: " , 1, 0, 0)
        #Number of times test2 button was clicked
        self.test3 = 0
        self.test3_id = p.addUserDebugParameter("Drop cube 5kg: " , 1, 0, 0)

        #Number of times test2 button was clicked
        self.test4 = 0
        self.test4_id = p.addUserDebugParameter("Simulate table " , 1, 0, 0)

        #Number of times test2 button was clicked
        self.test5 = 0
        self.test5_id = p.addUserDebugParameter("Simulate Wall " , 1, 0, 0)

        #The objects added
        self.objects = []

        self.t = Timer(1.0, self.update)
        self.t.start()

    def update(self):
        """
        Reads the debug parameters and updates environment accordingly.
        """
        count = p.readUserDebugParameter(self.test1_id)
        if p.readUserDebugParameter(self.test1_id) - self.test1 != 0:
            self.test1 = p.readUserDebugParameter(self.test1_id)
            self.weight_test(1.0)

        elif p.readUserDebugParameter(self.test2_id) - self.test2 != 0:
            self.test2 = p.readUserDebugParameter(self.test2_id)
            self.weight_test(3.0)

        elif p.readUserDebugParameter(self.test3_id) - self.test3 != 0:
            self.test3 = p.readUserDebugParameter(self.test3_id)
            self.weight_test(5.0)

        elif p.readUserDebugParameter(self.test4_id) - self.test4 != 0:
            self.test4 = p.readUserDebugParameter(self.test4_id)
            self.pos_ctrl_test([0.9, 0.0, 0.7])

        elif p.readUserDebugParameter(self.test5_id) - self.test5 != 0:
            self.test5 = p.readUserDebugParameter(self.test5_id)
            self.pos_ctrl_test([0.1, -1.0, 1.26])

        else:
            count = p.readUserDebugParameter(self.add_cube_id)
            #for each click done in last step simulation
            for i in range(int(count - self.add_cube)):
                self.add_cube = count
                scale = p.readUserDebugParameter(self.size_id) 
                pos = [p.readUserDebugParameter(self.x_id) , p.readUserDebugParameter(self.y_id) , p.readUserDebugParameter(self.z_id)]
                orn = p.getQuaternionFromEuler([p.readUserDebugParameter(self.or_x_id) * math.pi / 180, 
                                                p.readUserDebugParameter(self.or_y_id) * math.pi / 180, 
                                                p.readUserDebugParameter(self.or_z_id) * math.pi / 180])
                self.objects.append(p.loadURDF("cube_small.urdf", pos, orn, globalScaling = scale))
                mass = p.readUserDebugParameter(self.mass_id)
                p.changeDynamics(self.objects[-1], -1, mass=mass)

            count = p.readUserDebugParameter(self.add_fixed_cube_id)
            for i in range(int(count - self.add_fixed_cube)):
                self.add_fixed_cube = count
                scale = p.readUserDebugParameter(self.size_id) 
                pos = [p.readUserDebugParameter(self.x_id) , p.readUserDebugParameter(self.y_id) , p.readUserDebugParameter(self.z_id)]
                self.objects.append(p.loadURDF("cube_small.urdf", pos, globalScaling = scale, useFixedBase = 1))
            
            count = p.readUserDebugParameter(self.add_model_id)
            for i in range(int(count - self.add_model)):
                self.add_model = count
                scale = p.readUserDebugParameter(self.size_id) 
                pos = [p.readUserDebugParameter(self.x_id) , p.readUserDebugParameter(self.y_id) , p.readUserDebugParameter(self.z_id)]
                orn = p.getQuaternionFromEuler([p.readUserDebugParameter(self.or_x_id) * math.pi / 180, 
                                                p.readUserDebugParameter(self.or_y_id) * math.pi / 180, 
                                                p.readUserDebugParameter(self.or_z_id) * math.pi / 180])
                model = input("Please enter a model from pybullet_data: ")
                try:
                    self.objects.append(p.loadURDF(model, pos, orn, globalScaling = scale))
                except:
                    rclpy.logging._root_logger.error("The model " + model + " does not exist.")

            
            count = p.readUserDebugParameter(self.delete_obj_id)
            for i in range(int(count - self.delete_obj)):
                self.delete_obj = count
                if(self.objects):
                    p.removeBody(self.objects.pop())
                
            mass = p.readUserDebugParameter(self.mass_id)
            if mass != self.mass:
                self.mass = mass
                if(self.objects):
                    p.changeDynamics(self.objects[-1], -1, mass=mass)
        
            x = p.readUserDebugParameter(self.x_id)
            y = p.readUserDebugParameter(self.y_id)
            z = p.readUserDebugParameter(self.z_id)
            or_x = p.readUserDebugParameter(self.or_x_id)
            or_y = p.readUserDebugParameter(self.or_y_id)
            or_z = p.readUserDebugParameter(self.or_z_id)
            if x != self.x or y != self.y or z != self.z or or_x != self.or_x or or_y != self.or_y or or_z != self.or_z :
                self.x = x
                self.y = y
                self.z = z
                self.or_x = or_x
                self.or_y = or_y
                self.or_z = or_z
                if(self.objects):
                    p.resetBasePositionAndOrientation(self.objects[-1], [self.x, self.y, self.z], 
                                                            p.getQuaternionFromEuler([or_x * math.pi / 180, 
                                                            or_y * math.pi / 180, 
                                                            or_z * math.pi / 180]))
        self.update_castle_pos()

    def update_castle_pos(self):
        if abs(self.o) > math.pi:
            self.sig = -1 * self.sig
        p.resetBasePositionAndOrientation(self.castle, [0,2,0], (0,0,self.o,1))  
   
    def weight_test(self, mass, scale=8.2, pos=[0.6, 0.0, 1.8], orn=[14, 0.0, 0.0]): 
        for o in self.objects:
                p.removeBody(o)
                self.objects = []
        quat = p.getQuaternionFromEuler([orn[0] * math.pi / 180, 
                                            orn[1] * math.pi / 180, 
                                            orn[2] * math.pi / 180])
        self.objects.append(p.loadURDF("cube_small.urdf", pos, quat, globalScaling = scale))
        p.changeDynamics(self.objects[-1], -1, mass=mass)

    def pos_ctrl_test(self,pos, scale=23.0, orn=[0,0,0], mass=5.0): 
        for o in self.objects:
                p.removeBody(o)
                self.objects = []
        quat = p.getQuaternionFromEuler([orn[0] * math.pi / 180, 
                                            orn[1] * math.pi / 180, 
                                            orn[2] * math.pi / 180])
        self.objects.append(p.loadURDF("cube_small.urdf", pos, quat, useFixedBase=1, globalScaling = scale))
        # p.changeDynamics(self.objects[-1], -1, mass=mass)

    def stop(self):
        self.t.cancel()
