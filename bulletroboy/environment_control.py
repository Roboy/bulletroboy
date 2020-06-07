import pybullet as p
import pybullet_data


class EnvironmentCtrl():
    def __init__(self):
        """
        This class handles the simulation of the environment.
        """
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        self.cube_size_id = p.addUserDebugParameter("Cube size " , 1, 25, 20)

        self.cube_x_id = p.addUserDebugParameter("Cube position, x " , -5, 5, -0.5)
        self.cube_y_id = p.addUserDebugParameter("Cube position, y " , -5, 5, -0.8)
        self.cube_z_id = p.addUserDebugParameter("Cube position, z " , -5, 5, 0.5)

        
        self.add_cube = 0
        self.add_cube_id = p.addUserDebugParameter("Add cube " , 1, 0, 0)
        
        self.add_fixed_cube = 0
        self.add_fixed_cube_id = p.addUserDebugParameter("Add fixed cube " , 1, 0, 0)

        self.delete_cube = 0
        self.delete_cube_id = p.addUserDebugParameter("Delete last cube " , 1, 0, 0)
        
        self.cubes = []

    def update(self):
    	
        
        count = p.readUserDebugParameter(self.add_cube_id)
        for i in range(int(count - self.add_cube)):
            self.add_cube = count
            scale = p.readUserDebugParameter(self.cube_size_id) 
            pos = [p.readUserDebugParameter(self.cube_x_id) , p.readUserDebugParameter(self.cube_y_id) , p.readUserDebugParameter(self.cube_z_id)]
            self.cubes.append(p.loadURDF("cube_small.urdf", pos, globalScaling = scale))

        count = p.readUserDebugParameter(self.add_fixed_cube_id)
        for i in range(int(count - self.add_fixed_cube)):
            self.add_fixed_cube = count
            scale = p.readUserDebugParameter(self.cube_size_id) 
            pos = [p.readUserDebugParameter(self.cube_x_id) , p.readUserDebugParameter(self.cube_y_id) , p.readUserDebugParameter(self.cube_z_id)]
            self.cubes.append(p.loadURDF("cube_small.urdf", pos, globalScaling = scale, useFixedBase = 1))

        count = p.readUserDebugParameter(self.delete_cube_id)
        for i in range(int(count - self.delete_cube)):
            self.delete_cube = count
            if(self.cubes):
                p.removeBody(self.cubes.pop())
                self.delete_cube = count




