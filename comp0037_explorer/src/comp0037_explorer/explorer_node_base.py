import rospy
import threading
import math
import time

from math import pow,atan2,sqrt,pi
from comp0037_mapper.msg import *
from comp0037_mapper.srv import *
from comp0037_reactive_planner_controller.srv import *
from comp0037_reactive_planner_controller.occupancy_grid import OccupancyGrid
from comp0037_reactive_planner_controller.grid_drawer import OccupancyGridDrawer
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from nav_msgs.msg import Odometry
from Queue import PriorityQueue

class ExplorerNodeBase(object):

    def __init__(self):
        rospy.init_node('explorer')

        # Get the drive robot service
        rospy.loginfo('Waiting for service drive_to_goal')
        rospy.wait_for_service('drive_to_goal')
        self.driveToGoalService = rospy.ServiceProxy('drive_to_goal', Goal)
        rospy.loginfo('Got the drive_to_goal service')

        self.waitForGoal =  threading.Condition()
        self.waitForDriveCompleted =  threading.Condition()
        self.goal = None

        # Subscribe to get the map update messages
        self.mapUpdateSubscriber = rospy.Subscriber('updated_map', MapUpdate, self.mapUpdateCallback)
        self.noMapReceived = True

        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        self.pose = Pose2D()

        # Clear the map variables
        self.occupancyGrid = None
        self.deltaOccupancyGrid = None

        self.frontiers = PriorityQueue()

        # Choosing what explorer to use. 
        # 0 - Original inefficient explorer
        # 1 - WFD explorer going to the middle of the largest frontier
        self.explorerAlgorithm = 1

        # Flags used to control the graphical output. Note that we
        # can't create the drawers until we receive the first map
        # message.
        self.showOccupancyGrid = rospy.get_param('show_explorer_occupancy_grid', True)
        self.showDeltaOccupancyGrid = rospy.get_param('show_explorer_delta_occupancy_grid', True)
        self.occupancyGridDrawer = None
        self.deltaOccupancyGridDrawer = None
        self.visualisationUpdateRequired = False

        self.time_scale_factor=None
        self.timeTakenToExplore = float('inf')
        self.coverage = 0

        self.exportDirectory ='comp0037_cw2\\export\\explorer_data.csv'

        # Request an initial map to get the ball rolling
        rospy.loginfo('Waiting for service request_map_update')
        rospy.wait_for_service('request_map_update')
        mapRequestService = rospy.ServiceProxy('request_map_update', RequestMapUpdate)
        mapUpdate = mapRequestService(True)

        while mapUpdate.initialMapUpdate.isPriorMap is True:
            self.kickstartSimulator()
            mapUpdate = mapRequestService(True)
            
        self.mapUpdateCallback(mapUpdate.initialMapUpdate)    
    
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose

    def getCurrentPosition(self):
        currentCoords = (self.pose.x,self.pose.y)
        return currentCoords

    def mapUpdateCallback(self, msg):
        rospy.loginfo("map update received")
        
        # If the occupancy grids do not exist, create them
        if self.occupancyGrid is None:
            self.occupancyGrid = OccupancyGrid.fromMapUpdateMessage(msg)
            self.deltaOccupancyGrid = OccupancyGrid.fromMapUpdateMessage(msg)

        # Update the grids
        self.occupancyGrid.updateGridFromVector(msg.occupancyGrid)
        self.deltaOccupancyGrid.updateGridFromVector(msg.deltaOccupancyGrid)
        
        # Update the frontiers
        if (self.explorerAlgorithm == 1):
            self.updateFrontiers()

        # Flag there's something to show graphically
        self.visualisationUpdateRequired = True

    # Get known neighbouring cells
    def getNeighbouringCells(self, cell, occupancyGrid):
        adj_cells = []
        increments = [(0,-1),(1,-1),(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1)]

        for d in increments:
            new_cell = (cell[0]+d[0], cell[1]+d[1])
            if (occupancyGrid.grid[new_cell[0]][new_cell[1]] != 0.5):
                adj_cells.append(new_cell)

        return adj_cells

    # This method determines if a cell is a frontier cell or not. A
    # frontier cell is open and has at least one neighbour which is
    # unknown.
    def isFrontierCell(self, x, y, occupancyGrid=False):

        if not occupancyGrid:
            occupancyGrid = self.occupancyGrid

        # Check the cell to see if it's open
        if occupancyGrid.getCell(x, y) != 0:
            return False

        # Check the neighbouring cells; if at least one of them is unknown, it's a frontier
        return self.checkIfCellIsUnknown(x, y, -1, -1, occupancyGrid) | self.checkIfCellIsUnknown(x, y, 0, -1, occupancyGrid) \
            | self.checkIfCellIsUnknown(x, y, 1, -1, occupancyGrid) | self.checkIfCellIsUnknown(x, y, 1, 0, occupancyGrid) \
            | self.checkIfCellIsUnknown(x, y, 1, 1, occupancyGrid) | self.checkIfCellIsUnknown(x, y, 0, 1, occupancyGrid) \
            | self.checkIfCellIsUnknown(x, y, -1, 1, occupancyGrid) | self.checkIfCellIsUnknown(x, y, -1, 0, occupancyGrid)
            
    def checkIfCellIsUnknown(self, x, y, offsetX, offsetY, occupancyGrid):
        newX = x + offsetX
        newY = y + offsetY
        return (newX >= 0) & (newX < occupancyGrid.getWidthInCells()) \
            & (newY >= 0) & (newY < occupancyGrid.getHeightInCells()) \
            & (occupancyGrid.getCell(newX, newY) == 0.5)

    # You should provide your own implementation of this method which
    # maintains and updates the frontiers.  The method should return
    # True if any outstanding frontiers exist. If the method returns
    # False, it is assumed that the map is completely explored and the
    # explorer will exit.
    def updateFrontiers(self):
        raise NotImplementedError()

    def chooseNewDestination(self):
        raise NotImplementedError()

    def destinationReached(self, goalReached):
        raise NotImplementedError()

    def updateVisualisation(self):

        # If we don't need to do an update, simply flush the graphics
        # to make sure everything appears properly in VNC
        
        if self.visualisationUpdateRequired is False:

            if self.occupancyGridDrawer is not None:
                self.occupancyGridDrawer.flushAndUpdateWindow()

            if self.deltaOccupancyGridDrawer is not None:
                self.deltaOccupancyGridDrawer.flushAndUpdateWindow()

            return
                

        # Update the visualisation; note that we can only create the
        # drawers here because we don't know the size of the map until
        # we get the first update from the mapper.
        if self.showOccupancyGrid is True:
            if self.occupancyGridDrawer is None:
                windowHeight = rospy.get_param('maximum_window_height_in_pixels', 600)
                self.occupancyGridDrawer = OccupancyGridDrawer('Explorer Node Occupancy Grid',\
                                                               self.occupancyGrid, windowHeight)
	        self.occupancyGridDrawer.open()
	    self.occupancyGridDrawer.update()

        if self.showDeltaOccupancyGrid is True:
            if self.deltaOccupancyGridDrawer is None:
                windowHeight = rospy.get_param('maximum_window_height_in_pixels', 600)
                self.deltaOccupancyGridDrawer = OccupancyGridDrawer('Explorer Node Delta Occupancy Grid',\
                                                                    self.deltaOccupancyGrid, windowHeight)
	        self.deltaOccupancyGridDrawer.open()
	    self.deltaOccupancyGridDrawer.update()

        self.visualisationUpdateRequired = False

    def sendGoalToRobot(self, goal):
        rospy.logwarn("Sending the robot to " + str(goal))
        try:
            response = self.driveToGoalService(goal[0], goal[1], 0)
            return response.reachedGoal
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

    # Special case. If this is the first time everything has
    # started, stdr needs a kicking to generate the first
    # laser message. Telling it to take a very small movement appears to be sufficient
    def kickstartSimulator(self):
        velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=1)
        velocityMessage = Twist()
        velocityPublisher.publish(velocityMessage)
        rospy.sleep(1)
    
    def computeCoverage(self):

        totalCells = float(self.occupancyGrid.getWidthInCells() * self.occupancyGrid.getHeightInCells())
        coveredCells = 0.0
        for x in range(0, self.occupancyGrid.getWidthInCells()):
            for y in range(0, self.occupancyGrid.getHeightInCells()):
                if not (self.checkIfCellIsUnknown(x,y,0,0,self.occupancyGrid)):
                    coveredCells = coveredCells + 1
        
        coverage = coveredCells/totalCells
        return coverage

    def exportData(self):
        # TODO : Implement exporting method
        # Need to write:
        # - self.explorerAlgorithm
        # - self.coverage
        # - self.timeTakenToExplore
        # - self.time_scale_factor
        # - rosparam: use_search_grid_to_validate_start_and_end
        column_headers = ['explorerAlgorithm','coverage','timeTakenToExplore',
                          'time_scale_factor', 'use_search_grid_to_validate_start_and_end']

        data = [self.explorerAlgorithm, self.coverage, self.timeTakenToExplore,
                self.time_scale_factor, 
                rospy.get_param('use_search_grid_to_validate_start_and_end', 'Not Available')]

        # If directory doesn't exist create directory
        if not os.path.exists(os.path.split(self.exportDirectory)[0]):
            os.makedirs(os.path.split(self.exportDirectory)[0])

        # If file doesn't exist create file
        if(os.path.isfile(self.exportDirectory)):
            isFileEmpty = (os.stat(self.exportDirectory).st_size == 0)
        else:
            isFileEmpty = True

        rowList=[]
        rowFound=False

        if isFileEmpty:
            with open(self.exportDirectory, 'w') as write_csvfile:
                # Instanstiate writer
                writer = csv.writer(write_csvfile)
                writer.writerow(column_headers)
                writer.writerow(data)
        else:
            with open(self.exportDirectory, 'r') as read_csvfile:
                # Instantiate reader
                reader = csv.reader(read_csvfile)
                # Find if row already exists for that planner
                    for row in reader:
                        if(str(row[0])==str(self.plannerName) and str(row[1])==str(self.mapName)):
                            rowFound=True
                        else:
                            rowList.append(row)
            # If row was not found then append file.          
            if(not rowFound):
                with open(self.exportDirectory, 'a') as write_csvfile:
                    writer = csv.writer(write_csvfile)
                    writer.writerow(data)
            # If row was found then rewrite the whole file without the old row
            else:
                with open(self.exportDirectory, 'w') as write_csvfile:
                    rowList.append(data)
                    writer = csv.writer(write_csvfile)
                    for row in rowList:
                        writer.writerow(row)

        # print(self.time_scale_factor)
        # print("Time taken to explore map: " + str(self.explorer.timeTakenToExplore) + "s")
        # print("Proportion of map explored: " + str(self.explorer.coverage))


    class ExplorerThread(threading.Thread):
        def __init__(self, explorer):
            threading.Thread.__init__(self)
            self.explorer = explorer
            self.running = False
            self.completed = False

        def isRunning(self):
            return self.running

        def hasCompleted(self):
            return self.completed

        def run(self):

            self.running = True
            # startTime = time.time()
            startTime = rospy.get_time()

            while (rospy.is_shutdown() is False) & (self.completed is False):

                # Special case. If this is the first time everything
                # has started, stdr needs a kicking to generate laser
                # messages. To do this, we get the robot to
                
                # Create a new robot waypoint if required
                if (self.explorer.explorerAlgorithm == 1):
                    newDestinationAvailable, newDestination = self.explorer.chooseNewDestination()
                else:
                    newDestinationAvailable, newDestination = self.explorer.chooseNewDestinationOld()

                # Convert to world coordinates, because this is what the robot understands
                if newDestinationAvailable is True:
                    print 'newDestination = ' + str(newDestination)
                    newDestinationInWorldCoordinates = self.explorer.occupancyGrid.getWorldCoordinatesFromCellCoordinates(newDestination)
                    attempt = self.explorer.sendGoalToRobot(newDestinationInWorldCoordinates)
                    self.explorer.destinationReached(newDestination, attempt)
                else:
                    self.completed = True
                    # endTime = time.time()
                    endTime = rospy.get_time()
                    self.time_scale_factor = rospy.get_param('time_scale_factor',1.5)
                    self.explorer.timeTakenToExplore = (endTime - startTime)
                    self.explorer.coverage = self.explorer.computeCoverage() 

                    print(self.time_scale_factor)
                    print("Time taken to explore map: " + str(self.explorer.timeTakenToExplore) + "s")
                    print("Proportion of map explored: " + str(self.explorer.coverage))
                    self.explorer.exportData()

    def run(self):

        explorerThread = ExplorerNodeBase.ExplorerThread(self)

        keepRunning = True
        
        while (rospy.is_shutdown() is False) & (keepRunning is True):

            rospy.sleep(0.1)
            
            self.updateVisualisation()

            if self.occupancyGrid is None:
                continue

            if explorerThread.isRunning() is False:
                explorerThread.start()

            if explorerThread.hasCompleted() is True:
                explorerThread.join()
                keepRunning = False
