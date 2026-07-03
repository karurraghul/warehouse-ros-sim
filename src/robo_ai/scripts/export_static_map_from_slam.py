#!/usr/bin/env python3
"""Save /map from a running slam_localization session to warehouse_map.pgm.

Use this when AMCL scan/map alignment is off but the SLAM posegraph map looks
correct. AMCL reads warehouse_map.pgm; this refreshes it from live SLAM /map.

Prerequisites — sim already running with slam_localization:

    ros2 launch robo_ai warehouse_full.launch.py \\
        localization_mode:=slam_localization \\
        run_waypoint_navigator:=false use_rviz:=true

Wait ~20 s for slam_toolbox to publish /map, then in another terminal:

    python3 src/robo_ai/scripts/export_static_map_from_slam.py

Writes warehouse_map.yaml + warehouse_map.pgm under src/robo_ai/maps/.
"""
import argparse
import os
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(
        description='Export /map to warehouse_map for AMCL (from slam_localization)')
    parser.add_argument(
        '--output-dir',
        default=os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'maps'),
        help='Directory for warehouse_map.yaml / .pgm')
    parser.add_argument(
        '--map-basename', default='warehouse_map',
        help='Output base name without extension (default: warehouse_map)')
    parser.add_argument(
        '--wait-sec', type=float, default=5.0,
        help='Seconds to wait for /map before saving')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    output_base = os.path.join(args.output_dir, args.map_basename)
    if output_base.endswith('.yaml') or output_base.endswith('.pgm'):
        output_base = os.path.splitext(output_base)[0]

    print(f'Waiting {args.wait_sec}s for /map from slam_toolbox...')
    time.sleep(args.wait_sec)

    cmd = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-f', output_base,
        '--ros-args', '-p', 'save_map_timeout:=10000.0',
    ]
    print('Running:', ' '.join(cmd))
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        print(
            'map_saver_cli failed. Is the sim running with '
            'localization_mode:=slam_localization?',
            file=sys.stderr)
        return result.returncode

    yaml_path = output_base + '.yaml'
    pgm_path = output_base + '.pgm'
    if not os.path.isfile(yaml_path) or not os.path.isfile(pgm_path):
        print(f'Expected {yaml_path} and {pgm_path} — save may have failed.',
              file=sys.stderr)
        return 1

    print(f'Exported static map for AMCL:\n  {yaml_path}\n  {pgm_path}')
    print('Relaunch with localization_mode:=amcl to use the refreshed map.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
