#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
MODULE STRING
"""
import math
from dataclasses import dataclass
from typing import List
import numpy as np
import rospy
from nav_msgs.msg import Path, Odometry
from tf_helper.TFHelper import TFHelper

# parameters
GAINLH = 0.3  # look forward gain rospy.get_param("/gain/lookahead", 0.3)
TARGETSPEED = 2  # rospy.get_param("/target_speed", 2)
LOOKAHEADCONSTANT = 2.0  # rospy.get_param("/lookahead", 2.5) # [m] look-ahead distance
# GAINTS = 0.2  # rospy.get_param("/gain/target_speed", 0.2)
MINSPEED = rospy.get_param("/speed/min", 2)
MAXSPEED = rospy.get_param("/speed/max", 10)
BASELENGTH = rospy.get_param("/base_length", 1.49)  # [m] car length
MAXSTEERING = rospy.get_param("/steering/max", 0.4189)  # [rad] max steering angle


@dataclass
class Position:
    """
    data class to store position of the vehicle
    """

    x: float
    y: float


class Car:
    """
    CLASS DOCSTRING
    """

    def __init__(self) -> None:
        """
        parameters
        ----------
        x : float
            x coordinate of the vehicle rear axle

        y : float
            y coordinate of the vehicle rear axle

        yaw : float
            yaw of the vehicle

        currentSpeed : float
            current speed of the vehicle

        rearX : float
            x coordinate of the rear of the vehicle

        rearY : float
            y coordinate of the rear of the vehicle
        """

        self.position: Position = Position(0.0, 0.0)  # position in rear axle frame
        self.yaw: float = 0.0
        self.currentSpeed: float = 0.0

        self.rearX: float = self.position.x - ((BASELENGTH / 2) * math.cos(self.yaw))
        self.rearY: float = self.position.y - ((BASELENGTH / 2) * math.sin(self.yaw))

        self.lookAhead = rospy.get_param("/lookahead", 4.2)
        self.baseLength = rospy.get_param("/physical/car_base_length", 1.49)  # [m] car length

    def updateState(self, currentState: Odometry) -> None:
        """
        Update state of the car

        """

        self.position.x = currentState.pose.pose.position.x
        self.position.y = currentState.pose.pose.position.y
        self.yaw = currentState.pose.pose.orientation.z
        self.currentSpeed = currentState.twist.twist.linear.x

        self.lookAhead = self.currentSpeed * GAINLH + LOOKAHEADCONSTANT

    def calcDistance(self, pointX: float, pointY: float) -> float:
        """
        calculate the distance between the rear of the vehicle and a point

        Parameters
        ----------
        pointX : float
            x coordinate of the point

        pointY : float
            y coordinate of the point

        Returns
        -------
        distance : float
            distance between the rear of the vehicle and the point

        """
        distanceX: float = self.rearX - pointX
        distanceY: float = self.rearY - pointY
        distance: float = math.hypot(distanceX, distanceY)

        return distance

    def adaptivePurePursuitController(self, trajectory: Position) -> float:
        """
        DOCSTRING
        """
        trajX = trajectory.x
        trajY = trajectory.y

        alpha = math.atan2(trajY - self.rearY, trajX - self.rearX) - self.yaw

        delta = math.atan2(2.0 * BASELENGTH * math.sin(alpha) / self.lookAhead, 1.0)

        delta = min(delta, MAXSTEERING)
        delta = max(delta, -MAXSTEERING)
        return delta

    def proportionalControl(self, delta: float) -> float:
        """
        DOCSTRING
        """

        targetSpeed: float = (20.0 / 3.6) / (abs(delta) * 4)
        targetSpeed = min(targetSpeed, MAXSPEED)
        targetSpeed = max(targetSpeed, MINSPEED)

        return targetSpeed


class WayPoints:
    """
    Class to store new waypoints to a list of waypoints and search for the suitable target point
    to follow with the pure pursuit algorithm
    """

    def __init__(self) -> None:
        """
        Parameters
        ----------
        xList : List[float]
            list of x coordinates of the waypoints

        yList : List[float]
            list of y coordinates of the waypoints

        oldNearestPointIndex : int
            index of the nearest point to the vehicle at the previous time step

        """
        self.waypoints = Path()
        self.xList: List[float] = []
        self.yList: List[float] = []
        self.point = Position(0.0, 0.0)
        self.oldNearestPointIndex: int = 0
        self.helper = TFHelper("control")
        self.firstLoop: bool = False
        self.frameGiven: str = ""

    def updateWaypoints(self, waypointsMsg: Path) -> None:
        """
        callback function for recieving waypoints one time... All the points at once
        """
        # if self.xList == []:
        self.waypoints = waypointsMsg
        self.frameGiven = waypointsMsg.header.frame_id
        # if self.waypoints.header.frame_id == "global":
        #     self.waypoints = self.helper.transformMsg(waypointsMsg, "global")
        #     self.frameGiven = True
        # if self.waypoints.header.frame_id == "rear_link":
        #     self.waypoints = self.helper.transformMsg(waypointsMsg, "rear_link")
        #     self.frameGiven = False

    def targetPoints(self, ind: int) -> Position:
        """
        DOCSTRING
        """

        self.point.x = self.xList[ind]
        self.point.y = self.yList[ind]
        return self.point

    def searchTargetIndex(self, car: Car) -> int:
        """
        DOC STRING
        """

        if self.frameGiven in ("global", "map"):
            if self.firstLoop is False:
                # search nearest point index
                print("Went here")
                for index, _ in enumerate(self.waypoints.poses):
                    # Extracting and storing X and Y coordinates seperately in a list
                    # to get minimum distance in first loop only
                    # print(self.waypoints.poses[index])
                    self.xList.append(self.waypoints.poses[index].pose.position.x)
                    self.yList.append(self.waypoints.poses[index].pose.position.y)
                distanceX = [car.rearX - icx for icx in self.xList]
                distanceY = [car.rearY - icy for icy in self.yList]
                distance = np.hypot(distanceX, distanceY)

                if len(distance) != 0:
                    ind: int = int(np.argmin(distance))
                    self.firstLoop = True
                    self.oldNearestPointIndex = ind
            lastIndex: int = len(self.xList) - 1
            ind = self.oldNearestPointIndex
            # print(self.xList)
            distanceThisIndex = car.calcDistance(self.xList[ind], self.yList[ind])

            while distanceThisIndex < car.lookAhead:

                if ind >= lastIndex - 1:
                    ind = 0
                distanceNextIndex = car.calcDistance(self.xList[ind + 1], self.yList[ind + 1])

                ind = ind + 1
                distanceThisIndex = distanceNextIndex
            self.oldNearestPointIndex = ind
        else:  # if map is not given in first message
            self.xList = []
            self.yList = []
            rospy.loginfo("went in map not given")
            ind = 0
            for index, _ in enumerate(self.waypoints.poses):

                self.xList.append(self.waypoints.poses[index].pose.position.x)
                self.yList.append(self.waypoints.poses[index].pose.position.y)

            while ind <= len(self.xList) - 1:
                distanceThisIndex = car.calcDistance(self.xList[ind], self.yList[ind])
                if distanceThisIndex > car.lookAhead:
                    break
                ind = ind + 1

        return ind
