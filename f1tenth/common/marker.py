"""
Visualisation module for spawning markers on the track(s).
"""
from enum import Enum
from typing import Iterable
from collections import namedtuple
from visualization_msgs.msg import Marker, MarkerArray
from rospy import Publisher, Duration

# Provide more verbose namespace for passing marker colours
MarkerColour = namedtuple("PointMarkerColour", ["r", "g", "b"])

# The frame ID shouldn't change - it is defined within the F1/10 ROS simulator
FRAME_ID = "/map"

# Define shape and action of a marker
MARKER_TYPE = Marker.SPHERE
MARKER_ACTION = Marker.ADD


class MarkerPublisherChannel(Enum):
    """
    Available publisher channels for marker messages.
    """
    FIRST = Publisher("/visualisation_marker_01", Marker, queue_size=100)
    SECOND = Publisher("/visualisation_marker_02", Marker, queue_size=100)
    THIRD = Publisher("/visualisation_marker_03", Marker, queue_size=100)


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


def _create_marker(position_x: float, position_y: float, marker_id: int = 0, colour: MarkerColour = DEFAULT_COLOUR,
                   scale: float = DEFAULT_SCALE, duration: float = DEFAULT_LIFETIME) -> Marker:
    """
    Generate a marker using passed parameters.

    Duration of 0 will result in the marker never disappearing.
    """
    marker = Marker()
    marker.id = marker_id
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
