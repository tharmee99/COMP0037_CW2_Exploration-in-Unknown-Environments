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
        self.occupancyGrid.frontierCell = [[False for y in range(self.occupancyGrid.heightInCells)] for x in range(self.occupancyGrid.widthInCells)]
        
        worldPose = (self.pose.x, self.pose.y)
        gridPose = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates(worldPose)
        self.frontiers = self.getFrontiers(gridPose, self.occupancyGrid)

    # Implementation of the Wavefront Detection (WFD) algorithm 
    def getFrontiers(self, pose, occupancyGrid):
        # Instantiate empty priority queue to store frontiers ordered by size
        frontiers = PriorityQueue()

        # The state of each cell to be searched over. Four possible states:
        # 1 - Map Open List
        # 2 - Map Close List
        # 3 - Frontier Open List
        # 4 - Frontier Close List
        cell_state = [[0 for y in range(occupancyGrid.heightInCells)] for x in range(occupancyGrid.widthInCells)]

        # Queue to traverse over all cells in the map (outer BFS)
        map_queue = deque()
        map_queue.append(pose)
        cell_state[pose[0]][pose[1]] = 1

        while map_queue:
            p = map_queue.popleft()

            if (cell_state[p[0]][p[1]] == 2):
                continue
            if (self.isFrontierCell(p[0],p[1],occupancyGrid)):
                # Queue to traverse over neighbouring cells once frontier is found (inner BFS)
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
                
                frontiers.put((-1*len(newFrontier),newFrontier))

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
            
        return frontiers

    def getGoodCells(self, frontierList):
        goodCells = []

        for cell in frontierList:
            if cell not in self.blackList:
                goodCells.append(cell)

        return goodCells

    # Choosing the next destination based purely on the WFD algorithm
    def chooseNewDestination(self):
        destination = None
        nextCellValid = False

        loop_flg = True

        print('Is frontiers queue empty: {}'.format(self.frontiers.empty()))

        # While the frontier queue is not empty and a destination hasn't been found
        while (not self.frontiers.empty()) and (loop_flg):

            # Pop the largest frontier from the priority queue
            bestFrontier = self.frontiers.get()
            print('Length of bestFrontier[1]: {}'.format(len(bestFrontier[1])))

            # Retrieve all cells in the frontier that are not blacklisted
            goodCells = self.getGoodCells(bestFrontier[1])

            # If the current frontier has no good frontiers skip to next frontier
            if (len(goodCells) == 0):
                print('Is frontiers queue empty: {}'.format(self.frontiers.empty()))
                continue

            # set the middle cell in the frontier as the destination
            destination = self.getmiddleCell(goodCells)
            nextCellValid = True

            # Stop looping through frontiers
            loop_flg = False

            # Add frontier back to priority queue in case current destination is blacklisted
            self.frontiers.put((bestFrontier))
            break

        # TODO: Currently choosing first cell in largest frontier, Replace with middle cell

        return nextCellValid, destination

    def getmiddleCell(self, frontierCells):
        min_coords = [float('inf'),float('inf')]
        max_coords = [0,0]

        for cell in frontierCells:
            if (cell[0] > max_coords[0]):
                max_coords[0] = cell[0]
            
            if (cell[1] > max_coords[1]):
                max_coords[1] = cell[1]

            if (cell[0] < min_coords[0]):
                min_coords[0] = cell[0]

            if (cell[1] < min_coords[1]):
                min_coords[1] = cell[1]
        
        midpoint = ((min_coords[0]+max_coords[0])/2,(min_coords[1]+max_coords[1])/2)

        min_d2 = float('inf')
        candidate = None
        
        for cell in frontierCells:
            d2 = (cell[0] - midpoint[0])**2 + (cell[1] - midpoint[1])**2
            if d2 < min_d2:
                candidate = cell
                min_d2 = d2

        return candidate

    # The original implemented algorithm to choose next destination
    def chooseNewDestinationOld(self):
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
            print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            self.occupancyGrid.blacklistCell[goal[0]][goal[1]] = True
            self.visualisationUpdateRequired = True
            