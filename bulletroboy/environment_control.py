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
        p.loadURDF("samurai.urdf")
        
        #Size parameter
        self.size_id = p.addUserDebugParameter("Size " , 1, 25, 20)

        #object position parameters
        self.x = -0.5
        self.y = -0.8
        self.z = 0.5
        self.x_id = p.addUserDebugParameter("Position, x " , -5, 5, -0.5)
        self.y_id = p.addUserDebugParameter("Position, y " , -5, 5, -0.8)
        self.z_id = p.addUserDebugParameter("Position, z " , -5, 5, 0.5)
        
        #object orientation parameters
        self.or_x = 0
        self.or_y = 0
        self.or_z = 0
        self.or_x_id = p.addUserDebugParameter("Orientation, x-Axis " , -90, 90, 0)
        self.or_y_id = p.addUserDebugParameter("Orientation, y-Axis " , -90, 90, 0)
        self.or_z_id = p.addUserDebugParameter("Orientation, z-Axis " , -90, 90, 0)

        #object mass parameter
        self.mass = 0.1
        self.mass_id = p.addUserDebugParameter("Mass " , 0, 5, 0.1)

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

        #The objects added
        self.objects = []

        self.t = Timer(1.0, self.update)
        self.t.start()

    def update(self):
        """
        Reads the debug parameters and updates environment accordingly.
        """
        count = p.readUserDebugParameter(self.add_cube_id)
        #for each click done in last step simulation
        for i in range(int(count - self.add_cube)):
            self.add_cube = count
            scale = p.readUserDebugParameter(self.size_id) 
            pos = [p.readUserDebugParameter(self.x_id) , p.readUserDebugParameter(self.y_id) , p.readUserDebugParameter(self.z_id)]
            self.objects.append(p.loadURDF("cube_small.urdf", pos, globalScaling = scale))

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
                rclpy.logging._root_logger.error("The model " , model, " does not exist.")

            
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
        

    def stop(self):
        self.t.cancel()