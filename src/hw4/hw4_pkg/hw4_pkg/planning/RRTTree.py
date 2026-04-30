import operator
import numpy

class RRTTree(object):
    
    def __init__(self, problem):
        
        self.planning_env = problem
        self.vertices = []
        self.costs = []
        self.edges = dict()

    def GetRootID(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def GetNearestVertex(self, config):
        '''
        Returns the nearest state ID in the tree.
        @param config Sampled configuration.
        '''
        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.compute_distance_rrt(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]
            
    def GetKNN(self, config, k):
        '''
        Return k-nearest neighbors
        @param config Sampled configuration.
        @param k Number of nearest neighbors to retrieve.
        '''
        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.compute_distance_rrt(config, v))

        dists = numpy.array(dists)
        knnIDs = numpy.argpartition(dists, k)
        knnDists = [dists[i] for i in knnIDs]

        return knnIDs, [self.vertices[vid] for vid in knnIDs]

    def GetNNInRad(self, config, rad):
        '''
        Return neighbors within ball of radius
        @param config Sampled configuration.
        @param rad ball radius
        '''
        rad = numpy.abs(rad)
        vids = []
        vertices = []
        for idx, v in enumerate(self.vertices):
            if self.planning_env.compute_distance_rrt(config, v) < rad:
                vids.append(idx)
                vertices.append(v)

        return vids, vertices


    def AddVertex(self, config, cost=0):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices.append(config)
        self.costs.append(cost)
        return vid

    def AddEdge(self, sid, eid):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid