"""
Visualisation module for spawning markers on the track(s).
"""
from enum import Enum
from typing import Iterable
from collections import namedtuple
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from rospy import Publisher, Duration
from .constants import FRAME_ID

# Provide more verbose namespace for passing marker colours
MarkerColour = namedtuple("PointMarkerColour", ["r", "g", "b"])

# Markers are always added
MARKER_ACTION = Marker.ADD


class MarkerType(Enum):
    """
    ROS marker types required for visualising different types of data using the same object.
    """

    POINT = Marker.SPHERE
    LINE = Marker.LINE_STRIP


class MarkerPublisherChannel(Enum):
    """
    Available publisher channels for marker messages.
    """

    FIRST = Publisher("/visualisation_marker_01", Marker, queue_size=100)
    SECOND = Publisher("/visualisation_marker_02", Marker, queue_size=100)
    THIRD = Publisher("/visualisation_marker_03", Marker, queue_size=100)
    FOURTH = Publisher("/visualisation_marker_04", Marker, queue_size=100)


class MarkerArrayPublisherChannel(Enum):
    """
    Available publisher channels for marker array messages.
    """

    FIRST = Publisher("/visualisation_marker_array_01", MarkerArray, queue_size=100)
    SECOND = Publisher("/visualisation_marker_array_02", MarkerArray, queue_size=100)
    THIRD = Publisher("/visualisation_marker_array_03", MarkerArray, queue_size=100)


# Define default configuration of a marker
DEFAULT_SCALE = 0.2
DEFAULT_COLOUR = MarkerColour(1.0, 0.0, 0.0)
DEFAULT_LIFETIME = 0.1
DEFAULT_MARKER_CHANNEL = MarkerPublisherChannel.FIRST
DEFAULT_MARKER_ARRAY_CHANNEL = MarkerArrayPublisherChannel.FIRST
DEFAULT_MARKER_TYPE = MarkerType.POINT


def _create_marker(
        position_x: float = 0,
        position_y: float = 0,
        marker_id: int = 0,
        colour: MarkerColour = DEFAULT_COLOUR,
        scale: float = DEFAULT_SCALE,
        duration: float = DEFAULT_LIFETIME,
        marker_type: MarkerType = DEFAULT_MARKER_TYPE
) -> Marker:
    """
    Generate a marker using passed parameters.

    Duration of 0 will result in the marker never disappearing.
    """
    marker = Marker()
    marker.id = marker_id
    marker.header.frame_id = FRAME_ID
    marker.type = marker_type.value
    marker.action = MARKER_ACTION

    # Marker is a sphere so scale is uniform within each direction
    marker.scale.x = scale
    if marker_type == MarkerType.POINT:
        marker.scale.y = scale
        marker.scale.z = scale

    # Colour should be passed by the caller
    marker.color.r = colour.r
    marker.color.g = colour.g
    marker.color.b = colour.b
    marker.color.a = 1.0

    # Position should be passed by the caller
    marker.pose.orientation.w = 1.0
    if marker_type == MarkerType.POINT:
        marker.pose.position.x = position_x
        marker.pose.position.y = position_y
        marker.pose.position.z = 0.0

    # When should the marker disappear
    marker.lifetime = Duration(duration)

    return marker


def mark(position_x: float, position_y: float, colour: MarkerColour = DEFAULT_COLOUR, scale: float = DEFAULT_SCALE,
         duration: float = DEFAULT_LIFETIME, channel: MarkerPublisherChannel = DEFAULT_MARKER_CHANNEL):
    """
    Spawn a marker at given location.
    """
    channel.value.publish(_create_marker(
        position_x=position_x,
        position_y=position_y,
        colour=colour,
        scale=scale,
        duration=duration,
    ))


def mark_array(positions: Iterable, colour: MarkerColour = DEFAULT_COLOUR, scale: float = DEFAULT_SCALE,
               duration: float = DEFAULT_LIFETIME, channel: MarkerArrayPublisherChannel = DEFAULT_MARKER_ARRAY_CHANNEL):
    """
    Spawn markers at given locations.
    """
    marker_array = MarkerArray()

    for index, (position_x, position_y) in enumerate(positions):
        marker = _create_marker(
            position_x=position_x,
            position_y=position_y,
            marker_id=index,
            colour=colour,
            scale=scale,
            duration=duration,
        )
        marker_array.markers.append(marker)

    channel.value.publish(marker_array)


def mark_line_strips(positions: Iterable, colour: MarkerColour = DEFAULT_COLOUR, scale: float = DEFAULT_SCALE,
                     duration: float = DEFAULT_LIFETIME, channel: MarkerPublisherChannel = DEFAULT_MARKER_CHANNEL):
    """
    Spawn line strips at the given locations.
    """
    lines = _create_marker(
        colour=colour,
        scale=scale,
        duration=duration,
        marker_type=MarkerType.LINE
    )

    for position_x, position_y in positions:
        point = Point()
        point.x, point.y, point.z = position_x, position_y, 0
        lines.points.append(point)

    channel.value.publish(lines)
