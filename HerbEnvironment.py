import numpy as np
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.resolution = resolution
        self.discrete_env = DiscreteEnvironment(self.resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 0.7],
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = np.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def checkSucc(self, config):

        self.env = self.robot.GetEnv()
        robot_pose = self.robot.GetTransform()
        table = self.robot.GetEnv().GetBodies()[1]

        config = config.tolist()
        self.robot.SetActiveDOFValues(config)

        if self.env.CheckCollision(self.robot,table):
            return False

        for i in range(self.discrete_env.dimension):
            if not(self.lower_limits[i] <= config[i] <= self.upper_limits[i]):
                return False

        return True

    def GetSuccessors(self, node_id):

        successors = []
        successors_config = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        
        config = self.discrete_env.NodeIdToConfiguration(node_id)

        for i in range(2 * self.discrete_env.dimension):
            prim = np.zeros(self.discrete_env.dimension)
            prim[i/2] = self.resolution

            if np.mod(i,2):
                succ = np.asarray(config) + prim
            else:
                succ = np.asarray(config) - prim
                
            if self.checkSucc(succ):
                successors_config.append(succ)
            else:
                continue

        successors = [self.discrete_env.ConfigurationToNodeId(x) for x in successors_config]

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

        dist = np.linalg.norm(end_config - start_config)

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        # Keeping it as Euclidean distance for now

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = np.linalg.norm(end_config - start_config)

        return cost

