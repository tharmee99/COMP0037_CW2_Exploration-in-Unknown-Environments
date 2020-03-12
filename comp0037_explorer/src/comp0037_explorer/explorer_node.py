import rospy

from explorer_node_base import ExplorerNodeBase
from Queue import PriorityQueue
# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []

        self.frontierCells = []

    def updateFrontiers(self):
        
        print("-----------------------")

        for x in range(0, self.deltaOccupancyGrid.getWidthInCells()):
            for y in range(0, self.deltaOccupancyGrid.getHeightInCells()):
                if (self.deltaOccupancyGrid.getCell(x,y) == 1.0):
                    candidate = (x,y)
                    if (candidate in self.frontierCells) and (not self.isFrontierCell(x, y)):
                        self.frontierCells.remove(candidate)
                        self.occupancyGrid.frontierCell[x][y] = False

                    if (self.isFrontierCell(x, y)):
                        self.frontierCells.append(candidate)
                        self.occupancyGrid.frontierCell[x][y] = True

    def chooseNewDestination(self):
#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

        candidateGood = False
        destination = None
        smallestD2 = float('inf')
    
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                candidate = (x, y)
                if self.isFrontierCell(x, y) is True:
                    candidateGood = True
                    for k in range(0, len(self.blackList)):
                        if self.blackList[k] == candidate:
                            candidateGood = False
                            break
                    
                    if candidateGood is True:
                        # Compute the shortest distance to middle of the map in the y axis.
                        d2 = candidate[0]**2+(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())**2

                        if (d2 < smallestD2):
                            destination = candidate
                            smallestD2 = d2

        # If we got a good candidate, use it

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
