import rospy

from explorer_node_base import ExplorerNodeBase
from Queue import PriorityQueue
from collections import deque
import random 

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)
        self.blackList = []

    def updateFrontiers(self):
        self.frontiers = []
        self.occupancyGrid.frontierCell = [[False for y in range(self.occupancyGrid.heightInCells)] for x in range(self.occupancyGrid.widthInCells)]
        
        worldPose = (self.pose.x, self.pose.y)
        gridPose = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(worldPose)
        self.getFrontiers(gridPose, self.occupancyGrid)
        pass

    def updateFrontiersFFD(self):

        print(self.pose.x)

        for x in range(0, self.deltaOccupancyGrid.getWidthInCells()):
            for y in range(0, self.deltaOccupancyGrid.getHeightInCells()):
                candidate = (x,y)

                # If the current cell is labelled as a frontier cell but is no longer a frontier cell
                # Remove the cell from the frontierCells list
                if (candidate in self.frontierCells) and (not self.isFrontierCell(x, y)):
                    self.frontierCells.remove(candidate)
                    self.occupancyGrid.frontierCell[x][y] = False

                # If a change occured to this cell and the cell is now a frontier cell add the cell to
                # the frontierCells list
                if (self.deltaOccupancyGrid.getCell(x,y) == 1.0) and (self.isFrontierCell(x, y)):
                    self.frontierCells.append(candidate)
                    self.occupancyGrid.frontierCell[x][y] = True

        self.updateFrontierClusters()

    # Implemntation of the Wavefront Detection (WFD) algorithm 
    def getFrontiers(self, pose, occupancyGrid):
        cell_state = [[0 for y in range(occupancyGrid.heightInCells)] for x in range(occupancyGrid.widthInCells)]

        # 1 - Map Open List
        # 2 - Map Close List
        # 3 - Frontier Open List
        # 4 - Frontier Close List

        map_queue = deque()
        map_queue.append(pose)
        print(type(pose[0]))
        cell_state[pose[0]][pose[1]] = 1

        while map_queue:
            p = map_queue.popleft()

            if (cell_state[p[0]][p[1]] == 2):
                continue
            if (self.isFrontierCell(p[0],p[1],occupancyGrid)):
                frontier_queue = deque()
                newFrontier = []
                frontier_queue.append(p)
                cell_state[p[0]][p[1]] = 3

                while frontier_queue:
                    q = frontier_queue.popleft()
                    q_cell_state = cell_state[q[0]][q[1]]
                    if ((q_cell_state == 2) or (q_cell_state == 4)):
                        continue
                    if (self.isFrontierCell(q[0],q[1],occupancyGrid)):
                        newFrontier.append(q)
                        self.occupancyGrid.frontierCell[q[0]][q[1]] = True
                        adj_cells = self.getNeighbouringCells(q, occupancyGrid)
                        for cell in adj_cells:
                            current_cell_state = cell_state[cell[0]][cell[1]]
                            if ((current_cell_state == 0) or (current_cell_state == 1)):
                                frontier_queue.append(cell)
                                cell_state[cell[0]][cell[1]] = 3
                    cell_state[q[0]][q[1]] = 4
                
                self.frontiers.append((len(newFrontier),newFrontier))
                for cell in newFrontier:
                    current_cell_state = cell_state[cell[0]][cell[1]]
                    cell_state[cell[0]][cell[1]] = 2
            adj_cells = self.getNeighbouringCells(p, occupancyGrid)
            for cell in adj_cells:
                current_cell_state = cell_state[cell[0]][cell[1]]
                if not ((current_cell_state == 1) or (current_cell_state == 2)):
                    map_queue.append(cell)
                    cell_state[cell[0]][cell[1]] = 1
            cell_state[p[0]][p[1]] = 2

    # Get known neighbouring cells
    def getNeighbouringCells(self, cell, occupancyGrid):
        adj_cells = []
        increments = [(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1)]
        print("---------------------------------------------")
        print(cell)
        for d in increments:
            new_cell = (cell[0]+d[0], cell[1]+d[1])
            if (occupancyGrid.grid[new_cell[0]][new_cell[1]] != 0.5):
                print(new_cell)
                adj_cells.append(new_cell)

        return adj_cells

    # Choosing the next destination based purely on the WFD algorithm
    def chooseNewDestination(self):
        maxNum = 0
        bestFrontier = []

        for frontier in self.frontiers:
            if frontier[0] > maxNum:
                maxNum = frontier[0]
                bestFrontier = frontier[1]
        
        max_coords = (0,0)
        min_coords = (float('inf'), float('inf'))

        pos = random.randint(0, maxNum-1)



        return True, bestFrontier[pos]
        


    # Choosing the next destination based on the Fast Frontier Detection algorithm
    def chooseNewDestinationFFD(self):
        pass

    # The original implemented algorithm to choose next destination
    def chooseNewDestinationOld(self):
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
            print("-----------------------------------------------------")
            print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            self.occupancyGrid.blacklistCell[goal[0]][goal[1]] = True
            
