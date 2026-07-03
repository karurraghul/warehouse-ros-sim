#!/usr/bin/env python3
"""Serialize a slam_toolbox map for slam_localization mode.

Run while the sim is mapping with localization_mode:=slam_online:

    ros2 launch robo_ai warehouse_full.launch.py localization_mode:=slam_online \\
        run_waypoint_navigator:=false use_rviz:=true

Then in another terminal:

    python3 src/robo_ai/scripts/bootstrap_slam_map.py \\
        --map-name warehouse \\
        --output-dir src/robo_ai/maps

Produces ``warehouse.posegraph`` and ``warehouse.data`` for slam_localization.
"""
import argparse
import os
import sys
import time

import rclpy
from slam_toolbox.srv import SerializePoseGraph


def main():
    parser = argparse.ArgumentParser(description='Serialize slam_toolbox warehouse map')
    parser.add_argument(
        '--map-name', default='warehouse',
        help='Base filename without extension (default: warehouse)')
    parser.add_argument(
        '--output-dir',
        default=os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'maps'),
        help='Directory for .posegraph and .data files')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    output_base = os.path.join(args.output_dir, args.map_name)

    rclpy.init()
    node = rclpy.create_node('bootstrap_slam_map')
    client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

    if not client.wait_for_service(timeout_sec=args.timeout_sec):
        node.get_logger().error(
            'Service /slam_toolbox/serialize_map unavailable. '
            'Launch sim with localization_mode:=slam_online first.')
        rclpy.shutdown()
        return 1

    request = SerializePoseGraph.Request()
    request.filename = output_base
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout_sec)
    if not future.done() or future.result() is None:
        node.get_logger().error('SerializePoseGraph call failed or timed out.')
        rclpy.shutdown()
        return 1

    result = future.result()
    if result.result != SerializePoseGraph.Response.RESULT_SUCCESS:
        node.get_logger().error(f'SerializePoseGraph failed with code {result.result}')
        rclpy.shutdown()
        return 1

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if (os.path.isfile(output_base + '.posegraph')
                and os.path.isfile(output_base + '.data')):
            break
        time.sleep(0.2)

    node.get_logger().info(
        f'Serialized map written to {output_base}.posegraph / .data')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
