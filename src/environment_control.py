import pybullet as p
import pybullet_data
import rospy
import math
from rospkg import RosPack
import time

from threading import Timer

URDF_PATH = "/src/"

RANDOM_URDF = [
    ["duck_vhacd.urdf", [0.6, 0.0, 1.8], 6],
    ["racecar/racecar.urdf", [0.6, 0.0, 1.8], 1],
    ["quadruped/minitaur.urdf", [0.6, 0.0, 1.8], 1],
    ["jenga/jenga.urdf", [0.6, 0.0, 1.8], 5],
    ["sphere_small.urdf", [0.6, 0.0, 1.8], 10],
]


class EnvironmentCtrl():
    def __init__(self):
        """
        This class handles the simulation of the environment around roboy.
        """
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
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

        #Number of times add sphere button was clicked
        self.add_sphere = 0
        self.add_sphere_id = p.addUserDebugParameter("Add sphere " , 1, 0, 0)     

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
        self.table = 0
        self.table_id = p.addUserDebugParameter("Simulate table " , 1, 0, 0)

        #Number of times add random button was clicked
        self.add_random = 0
        self.add_random_id = p.addUserDebugParameter("Add random object " , 1, 0, 0)   

        #Number of times test2 button was clicked
        self.wall = 0
        self.wall_id = p.addUserDebugParameter("Simulate Wall " , 1, 0, 0)

        #The objects added
        self.objects = []

        rp = RosPack()
        self.urdf_path = rp.get_path('bulletroboy') + URDF_PATH
        
        self.random = 0
        # auto_sim_collision_timer 
        #rospy.Timer(rospy.Duration(5), self.collision_callback)
    
    def collision_callback(self, event):
        p.loadURDF("cube_small.urdf", [0,0,1], globalScaling = 8.2, useFixedBase = 0)

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

        elif p.readUserDebugParameter(self.table_id) - self.table != 0:
            self.table = p.readUserDebugParameter(self.table_id)
            self.test_table()
            # self.pos_ctrl_test([0.9, 0.0, 0.8], name="table")

        elif p.readUserDebugParameter(self.wall_id) - self.wall != 0:
            self.wall = p.readUserDebugParameter(self.wall_id)
            self.test_wall()

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

            count = p.readUserDebugParameter(self.add_sphere_id)
            #for each click done in last step simulation
            for i in range(int(count - self.add_sphere)):
                self.add_sphere = count
                scale = p.readUserDebugParameter(self.size_id) 
                pos = [p.readUserDebugParameter(self.x_id) , p.readUserDebugParameter(self.y_id) , p.readUserDebugParameter(self.z_id)]
                orn = p.getQuaternionFromEuler([p.readUserDebugParameter(self.or_x_id) * math.pi / 180, 
                                                p.readUserDebugParameter(self.or_y_id) * math.pi / 180, 
                                                p.readUserDebugParameter(self.or_z_id) * math.pi / 180])
                self.objects.append(p.loadURDF("sphere_small.urdf", pos, orn, globalScaling = scale))
                mass = p.readUserDebugParameter(self.mass_id)
                p.changeDynamics(self.objects[-1], -1, mass=mass)

            count = p.readUserDebugParameter(self.add_fixed_cube_id)
            for i in range(int(count - self.add_fixed_cube)):
                self.add_fixed_cube = count
                scale = p.readUserDebugParameter(self.size_id) 
                pos = [p.readUserDebugParameter(self.x_id) , p.readUserDebugParameter(self.y_id) , p.readUserDebugParameter(self.z_id)]
                self.objects.append(p.loadURDF("cube_small.urdf", pos, globalScaling = scale, useFixedBase = 1))

            count = p.readUserDebugParameter(self.add_random_id)
            for i in range(int(count - self.add_random)):
                self.add_random = count
                self.add_random_obj()

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
                    rospy.logerr("The model " , model, " does not exist.")

            
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
        
    def weight_test(self, mass, scale=8.2, pos=[0.6, 0.0, 1.8], orn=[14, 0.0, 0.0]): 
        quat = p.getQuaternionFromEuler([orn[0] * math.pi / 180, 
                                            orn[1] * math.pi / 180, 
                                            orn[2] * math.pi / 180])
        self.objects.append(p.loadURDF("cube_small.urdf", pos, quat, globalScaling = scale))
        p.changeDynamics(self.objects[-1], -1, mass=mass)

    def pos_ctrl_test(self, pos, scale=23.0, orn=[0,0,0], mass=5.0, name=""): 
        for o in self.objects:
                p.removeBody(o)
                self.objects = []
        quat = p.getQuaternionFromEuler([orn[0] * math.pi / 180, 
                                            orn[1] * math.pi / 180, 
                                            orn[2] * math.pi / 180])
        urdf = "cube_small" if not name else  self.urdf_path + name +".urdf"
        self.objects.append(p.loadURDF("cube_small.urdf", pos, quat, useFixedBase=1, globalScaling = scale))
        # p.changeDynamics(self.objects[-1], -1, mass=mass)
    
    def test_table(self):
        quat = p.getQuaternionFromEuler([0, 0, 90 * math.pi / 180])

        self.objects.append(p.loadURDF("table/table.urdf", [1.1, 0, 0.5], quat, useFixedBase=1, globalScaling = 1.5))

    def test_wall(self):
        quat = p.getQuaternionFromEuler([0, 90 * math.pi / 180, 90 * math.pi / 180])

        self.objects.append(p.loadURDF("block.urdf", [0.9, 0, 1.5], quat, useFixedBase=1, globalScaling = 60))


    def add_random_obj(self):
        quat = p.getQuaternionFromEuler([0, 0, 90 * math.pi / 180])
        obj = RANDOM_URDF[self.random % len(RANDOM_URDF)]
        quat = [0,0,0,1]
        if obj[0] == "jenga/jenga.urdf":
            quat = p.getQuaternionFromEuler([0, 0, 90 * math.pi / 180])
        self.objects.append(p.loadURDF(obj[0], obj[1], [0,0,0,1], useFixedBase=0, globalScaling = obj[2]))
        self.random += 1
        p.changeDynamics(self.objects[-1], -1, mass=5.0)