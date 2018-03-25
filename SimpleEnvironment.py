import numpy as np
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import pdb

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.resolution = resolution
        self.discrete_env = DiscreteEnvironment(self.resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 1.5],
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def checkSucc(self, config):

        self.env = self.robot.GetEnv()
        robot_pose = self.robot.GetTransform()
        table = self.robot.GetEnv().GetBodies()[1]
        # pdb.set_trace()
        # config = config.tolist()
        # self.robot.SetActiveDOFValues(config)

        robot_pose[0][3] = config[0];
        robot_pose[1][3] = config[1];
        self.robot.SetTransform(robot_pose);

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

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        
