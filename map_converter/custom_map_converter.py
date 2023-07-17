"""
Converts your custom map format to ADP-compatible Map protos.
"""
from __future__ import division

import logging
from pprint import pprint

from simian.public.proto import map_config_pb2

# 1. The following types are all generated from proto files in this repository. For example, to know
#    what fields are in a map_lane_pb2 message, in this repository go to
#    applied/simian/public/proto/map/map_lane.proto.
# 2. You can see more about how to use the generated python APIs here:
#    https://developers.google.com/protocol-buffers/docs/reference/python-generated
# 3. You need to install the Applied python wheel in order to see the following imports.
#    Please see the README for more info.
from simian.public.proto.map import map_lane_pb2
from simian.public.proto.map import map_pb2
from simian.public.proto.map import map_region_pb2

_LOGGER = logging.getLogger("custom_map_converter")
_EXAMPLE_MIN_WIDTH = 1.5


def convert(args, output_path):
    """
    This is the only function that must be defined in this file. It takes in an args struct and the
    output_path, and does all the work of reading in your map, converting to a Simian map, and
    saving that to disk.

    All of the "example" functions below are for reference only and can be deleted / modified as
    needed for your own purposes.
    """
    if not args.input_file:
        raise ValueError("Must specify input_file!")

    # This is the map that will be written to disk.
    output_map = map_pb2.Map()

    # 1. Open your input files.
    # 2. Do any necessary parsing and pre-processing.
    # 3. Populate the fields of the output map.

    # Here are a couple examples of filling out the map proto.
    example_add_lane(output_map)
    example_add_region(output_map)

    # Here we'll print it to the terminal just so you can see the structure. Feel free to delete.
    print("Output map:")
    pprint(output_map)

    # Here we write the completed MapProto to disk and then create and return a config
    # object that will get written to disk later by the converter framework.
    with open(output_path, "wb") as f:  # type: ignore[assignment] # ignore baseline; see #61427
        f.write(output_map.SerializeToString())

    # The numbers here are arbitrary. You should set these as necessary from your own map data.
    return example_create_config(100, 200, 37.3867971, -122.0691547)


def example_create_config(x, y, lat, lng):
    # An example of creating a MapConfig proto. Note that the correct way to fill this out
    # will depend on your map data, such as whether you use UTM zones or other projections.
    # See applied/simian/public/proto/map/map_config.proto for more.
    map_config = map_config_pb2.MapConfig()
    map_config.defaults.start_location.x = x
    map_config.defaults.start_location.y = y
    map_config.transforms.coordinate_system.local_cartesian.latitude_origin = lat
    map_config.transforms.coordinate_system.local_cartesian.longitude_origin = lng
    return map_config


def example_add_region(output_map):
    region = output_map.region.add()
    region.id.id = "42"
    region.type = map_region_pb2.Region.SPEED_BUMP
    region.associated_lane.add().id = "7"
    point_data = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)]
    for point_tuple in point_data:
        new_point = region.polygon.point.add()
        example_fill_point_from_tuple(point_tuple, new_point)


def example_add_lane(output_map):
    # Example of adding a lane.
    #  * Note that you can only assign to primitive types in a proto!
    #  * You cannot assign to a message type, e.g. this does not work: foo.bar = baz
    #    Instead, you must either reach down to assign primitive types within them, or use
    #    "CopyFrom" or "MergeFrom", e.g. foo.bar.CopyFrom(baz).
    out_lane = output_map.lane.add()
    out_lane.id.id = "7"
    out_lane.speed_limit = 20

    # Fill out associations IDs for other lanes.
    # Note that this map will be invalid because lane "8" doesn't exist. In a valid map all
    # references to other lanes should be valid. But for this example we keep it simple.
    successor_id_proto = out_lane.successor_id.add()
    successor_id_proto.id = "8"
    # Should also fill out:
    # * out_lane.predecessor_id
    # * out_lane.left_neighbor_forward_lane_id
    # * out_lane.right_neighbor_forward_lane_id
    # * out_lane.left_neighbor_reverse_lane_id (for drive-on-right maps)
    # * out_lane.right_neighbor_reverse_lane_id (for drive-on-left maps)

    # Just some example curve data:
    input_central_curve = [(0, 0, 0), (10, 0, 0)]
    input_left_boundary = [(0, 1, 0), (10, 1, 0)]
    input_right_boundary = [(0, -1, 0), (10, -1, 0)]

    example_convert_curve(input_central_curve, out_lane.central_curve)
    example_convert_lane_boundary(
        input_left_boundary, out_lane.left_boundary, out_lane.central_curve, out_lane.left_sample
    )
    example_convert_lane_boundary(
        input_right_boundary, out_lane.right_boundary, out_lane.central_curve, out_lane.right_sample
    )

    out_lane.length = out_lane.central_curve.segment[0].length
    out_lane.type = map_lane_pb2.Lane.CITY_DRIVING
    out_lane.direction = map_lane_pb2.Lane.FORWARD
    out_lane.lane_surface = map_lane_pb2.Lane.ASPHALT


def example_euclidean_distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2) ** 0.5


def example_convert_lane_boundary(xyz_tuples, boundary, center_curve, out_samples):
    boundary_type = boundary.boundary_type.add()
    # Boundary type is hard-coded here just for this example.
    boundary_type.types.append(map_lane_pb2.LaneBoundaryType.DOTTED_WHITE)
    example_convert_curve(xyz_tuples, boundary.curve)
    assert len(center_curve.segment) == 1
    assert len(boundary.curve.segment) == 1
    boundary_points = boundary.curve.segment[0].line_segment.point
    # Last point along center curve.
    prev_point = center_curve.segment[0].start_position
    for index, point in enumerate(center_curve.segment[0].line_segment.point):
        lane_sample = out_samples.add()
        if index < len(boundary_points):
            # Match based on index.
            lane_sample.width = example_euclidean_distance(boundary_points[index], point)
        else:
            # Else take last point if no index is available.
            lane_sample.width = example_euclidean_distance(boundary_points[-1], point)
        # Width thresholding.
        lane_sample.width = max(lane_sample.width, _EXAMPLE_MIN_WIDTH)
        lane_sample.s = example_euclidean_distance(point, prev_point)
        prev_point = point


def example_convert_curve(xyz_tuples, curve):
    # Even though the proto structure suggests a Curve can have multiple Segments (each with their
    # own heading and length), in practice you should only add *one* Segment and then populate
    # that Segment with multiple LineSegments within it.
    segment = curve.segment.add(length = 0)
    example_fill_segment(xyz_tuples, segment)


def example_fill_segment(point_tuples, segment):
    example_fill_point_from_tuple(point_tuples[0], segment.start_position)
    prev_point = segment.start_position
    for point_tuple in point_tuples:
        new_point = segment.line_segment.point.add()
        example_fill_point_from_tuple(point_tuple, new_point)
        segment.length += example_euclidean_distance(new_point, prev_point)
        prev_point = new_point


def example_fill_point_from_tuple(src, dest):
    dest.x = src[0]
    dest.y = src[1]
    dest.z = src[2]
