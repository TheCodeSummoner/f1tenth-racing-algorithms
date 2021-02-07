"""
Visualisation module for spawning markers on the track(s).
"""
from collections import namedtuple
from visualization_msgs.msg import Marker
from rospy import Publisher, Duration

# Provide more verbose namespace for passing marker colours
MarkerColour = namedtuple("PointMarkerColour", ["r", "g", "b"])

# These shouldn't change - they are defined within F1/10 ROS simulator
FRAME_ID = "/map"
MARKER_TOPIC = "/waypoint_vis"

# Define shape of a marker
MARKER_TYPE = Marker.SPHERE
MARKER_ACTION = Marker.ADD

# Define default configuration of a marker
DEFAULT_SCALE = 0.2
DEFAULT_COLOUR = MarkerColour(1.0, 0.0, 0.0)
DEFAULT_LIFETIME = 0.1

# Declare to which topic should the marker messages be published
PUBLISHER = Publisher(MARKER_TOPIC, Marker, queue_size=100)


def mark(position_x: float, position_y: float, colour: MarkerColour = DEFAULT_COLOUR, scale: float = DEFAULT_SCALE,
         duration: float = DEFAULT_LIFETIME):
    """
    Spawn a marker at given location.
    """
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.type = MARKER_TYPE
    marker.action = MARKER_ACTION

    # Marker is a sphere so scale is uniform within each direction
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale

    # Colour should be passed by the caller
    marker.color.r = colour.r
    marker.color.g = colour.g
    marker.color.b = colour.b
    marker.color.a = 1.0

    # Position should be passed by the caller
    marker.pose.position.x = position_x
    marker.pose.position.y = position_y
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0

    # When should the marker disappear
    marker.lifetime = Duration(duration)
    PUBLISHER.publish(marker)
