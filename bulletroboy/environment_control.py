import pybullet as p
import pybullet_data

import time

from threading import Timer

class EnvironmentCtrl():
    def __init__(self):
        """
        This class handles the simulation of the environment around roboy.
        """
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        #Size parameter
        self.size_id = p.addUserDebugParameter("Size " , 1, 25, 20)

        #object position parameters
        self.x_id = p.addUserDebugParameter("Position, x " , -5, 5, -0.5)
        self.y_id = p.addUserDebugParameter("Position, y " , -5, 5, -0.8)
        self.z_id = p.addUserDebugParameter("Position, z " , -5, 5, 0.5)

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
        self.delete_cube = 0
        self.delete_cube_id = p.addUserDebugParameter("Delete last cube " , 1, 0, 0)
        
        #The cobjects added
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
            model = input("Please enter a model from pybullet_data: ")
            try:
                self.objects.append(p.loadURDF(model, pos, globalScaling = scale))
            except:
                print("The model " , model, " does not exist.")

            
        count = p.readUserDebugParameter(self.delete_cube_id)
        for i in range(int(count - self.delete_cube)):
            self.delete_cube = count
            if(self.objects):
                p.removeBody(self.objects.pop())
                self.delete_cube = count


    def stop(self):
        self.t.cancel()
