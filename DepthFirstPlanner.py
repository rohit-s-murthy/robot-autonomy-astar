import sys
import time
sys.setrecursionlimit(50000)

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        # self.nodes = dict()
        self.parent = dict() #easier to relate
        self.expanded_nodes = 0 #counting total nodes expanded

    def DFS(self,current_nodeId,goal_nodeId):
        self.expanded_nodes = self.expanded_nodes+1
        succ = self.planning_env.GetSuccessors(current_nodeId)
        for idx in succ:
            #exit from function if we reach goal
            if idx == goal_nodeId:
                self.parent[goal_nodeId]=current_nodeId
                break
            #else perform recursion over successors
            elif idx not in self.parent:
                self.parent[idx]=current_nodeId
                self.DFS(idx,goal_nodeId)

    def Plan(self, start_config, goal_config):

        start_time = time.time()
        plan = []
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        visited_nodes = 0
        #convert to node id to pass to helper function
        start_nodeId = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_nodeId = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        #helper function to perform recursion
        # print(start_nodeId)
        self.DFS(start_nodeId,goal_nodeId)

        idx = goal_nodeId
        #generating plan reverse
        plan.append(goal_config)
        prev_config = goal_config
        #to stop backtracking later
        self.parent[start_nodeId] = None

        self.planning_env.InitializePlot(goal_config)

        while self.parent[idx] is not None:
            visited_nodes = visited_nodes+1
            #convert to config space to append to plan
            current_config = self.planning_env.discrete_env.NodeIdToConfiguration(self.parent[idx])
            plan.append(current_config)
            self.planning_env.PlotEdge(current_config,prev_config)
            prev_config = current_config
            idx = self.parent[idx]

        plan.append(start_config)
        self.planning_env.PlotEdge(start_config,prev_config)
        visited_nodes = visited_nodes + 1
        plan.reverse()
        stop_time = time.time()
        print(plan)
        print("path length = %s" %(visited_nodes*self.planning_env.resolution))
        print("time taken = %s" %(stop_time-start_time))
        print("total nodes expanded = %s" %(self.expanded_nodes))
        return plan
