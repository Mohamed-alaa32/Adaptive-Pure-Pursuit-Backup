#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors

"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

from adaptive_pure_pursuit import Car, WayPoints, Position

# from adaptive_pure_pursuit.src.adaptive_pure_pursuit.adaptive_pure_pursuit import WayPoints, Car
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker


def main() -> None:
    """
    Main function for adaptive pure pursuit vehicle control node, subscribes to
    state and waypoints and publishes control actions
    """
    rospy.init_node("adaptive_pp_controller", anonymous=True)

    controlTopic = rospy.get_param("/control/actions_topic")  # "/control_actions"
    waypointsTopic = rospy.get_param("/control/waypoints_topic")  # "/pathplanning/waypoints"
    markerVizTopic = rospy.get_param("/control/marker_viz_topic")
    stateTopic = rospy.get_param("/control/state_topic")
    controlRate = rospy.get_param("/control/rate", 10)

    waypoints = WayPoints()
    car = Car()

    markerPub = rospy.Publisher(markerVizTopic, Marker, queue_size=10)
    controlActionPub = rospy.Publisher(controlTopic, AckermannDriveStamped, queue_size=10)
    rospy.Subscriber(waypointsTopic, Path, callback=waypoints.updateWaypoints)
    rospy.Subscriber(stateTopic, Odometry, callback=car.updateState)

    marker = Marker()
    rate = rospy.Rate(controlRate)

    # rospy.wait_for_message("/pathplanning/waypoints", Path)
    while not rospy.is_shutdown():

        ind = waypoints.searchTargetIndex(car)
        targetPoints: Position = waypoints.targetPoints(ind)
        delta = car.adaptivePurePursuitController(targetPoints)
        targetSpeed = car.proportionalControl(delta)

        controlAction = AckermannDriveStamped()
        controlAction.drive.steering_angle = delta
        controlAction.drive.speed = targetSpeed
        # controlAction.drive.jerk = car.lookAhead
        # # controlAction.drive.acceleration = targetPoints.x

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = "velodyne"
        marker.pose.position.x = waypoints.xList[ind]
        marker.pose.position.y = waypoints.yList[ind]
        marker.pose.position.z = -0.5
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.id = 0
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target"
        markerPub.publish(marker)
        controlActionPub.publish(controlAction)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
