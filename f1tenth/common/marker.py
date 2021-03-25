"""
Visualisation module for spawning markers on the track(s).
"""
from enum import Enum
from collections import namedtuple
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rospy import Publisher, Duration
from .constants import FRAME_ID

# Provide more verbose namespace for passing marker colours
MarkerColour = namedtuple("PointMarkerColour", ["r", "g", "b"])

# Markers are always added
MARKER_ADD_ACTION = Marker.ADD


class MarkerType(Enum):
    """
    ROS marker types required for visualising different types of data using the same object.
    """

    POINTS = Marker.POINTS
    LINES = Marker.LINE_LIST
    LINE_STRIPS = Marker.LINE_STRIP


class MarkerPublisherChannel(Enum):
    """
    Available publisher channels for marker messages.
    """

    FIRST = Publisher("/visualisation_marker_01", Marker, queue_size=100)
    SECOND = Publisher("/visualisation_marker_02", Marker, queue_size=100)
    THIRD = Publisher("/visualisation_marker_03", Marker, queue_size=100)
    FOURTH = Publisher("/visualisation_marker_04", Marker, queue_size=100)
    FIFTH = Publisher("/visualisation_marker_05", Marker, queue_size=100)


# Define default configuration of a marker
DEFAULT_SCALE = 0.2
DEFAULT_COLOUR = MarkerColour(1.0, 0.0, 0.0)
DEFAULT_LIFETIME = 0.1
DEFAULT_MARKER_CHANNEL = MarkerPublisherChannel.FIRST
DEFAULT_MARKER_TYPE = MarkerType.POINTS


def mark(
    positions: list,
    colour: MarkerColour = DEFAULT_COLOUR,
    scale: float = DEFAULT_SCALE,
    duration: float = DEFAULT_LIFETIME,
    channel: MarkerPublisherChannel = DEFAULT_MARKER_CHANNEL,
    marker_type: MarkerType = DEFAULT_MARKER_TYPE
):
    """
    Spawn a marker at given location and pass data to the "points" part of the marker.
    """
    marker = Marker()
    marker.header.frame_id = FRAME_ID
    marker.type = marker_type.value
    marker.action = MARKER_ADD_ACTION

    # Scale for points is uniform within each direction for points, or handles line width for line-type markers
    marker.scale.x = scale
    if marker_type == MarkerType.POINTS:
        marker.scale.y = scale
        marker.scale.z = scale

    # Colour should be passed by the caller
    marker.color.r = colour.r
    marker.color.g = colour.g
    marker.color.b = colour.b
    marker.color.a = 1.0

    # Position should be passed by the caller
    marker.pose.orientation.w = 1.0

    # When should the marker disappear
    marker.lifetime = Duration(duration)

    for position_x, position_y in positions:
        point = Point()
        point.x, point.y, point.z = position_x, position_y, 0
        marker.points.append(point)

    # Add first point again to close the polygon
    if marker_type == MarkerType.LINE_STRIPS:
        point = Point()
        position_x, position_y = positions[0]
        point.x, point.y, point.z = position_x, position_y, 0
        marker.points.append(point)

    channel.value.publish(marker)
