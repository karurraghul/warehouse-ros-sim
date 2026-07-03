#!/usr/bin/env python3
"""Drive a mapping session then serialize warehouse.posegraph for slam_localization.

Usage (from repo root, with sim already built):

    python3 src/robo_ai/scripts/auto_bootstrap_slam_map.py

Runs slam_online without Nav2 waypoint mission, drives the warehouse loop via
cmd_vel, serializes the posegraph while slam_toolbox is still running, then
shuts down the launch.
"""
import argparse
import os
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description='Auto-bootstrap slam_toolbox warehouse map')
    parser.add_argument(
        '--repo-root',
        default=os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))
    parser.add_argument('--startup-wait-sec', type=float, default=45.0)
    parser.add_argument('--drive-timeout-sec', type=float, default=900.0)
    parser.add_argument('--serialize-wait-sec', type=float, default=30.0)
    args = parser.parse_args()

    repo_root = os.path.abspath(args.repo_root)
    maps_dir = os.path.join(repo_root, 'src', 'robo_ai', 'maps')
    bootstrap = os.path.join(repo_root, 'src', 'robo_ai', 'scripts', 'bootstrap_slam_map.py')
    drive = os.path.join(repo_root, 'src', 'robo_ai', 'scripts', 'bootstrap_drive_mission.py')
    waypoints = os.path.join(
        repo_root, 'src', 'robo_ai_nav', 'config', 'waypoints.yaml')
    setup = f'source /opt/ros/humble/setup.bash && source {repo_root}/install/setup.bash'

    launch_cmd = (
        f'{setup} && ros2 launch robo_ai warehouse_full.launch.py '
        'localization_mode:=slam_online use_rviz:=false run_waypoint_navigator:=false'
    )
    print('Starting slam_online mapping session (no Nav2 mission)...')
    launch = subprocess.Popen(
        ['bash', '-lc', launch_cmd],
        cwd=repo_root,
        preexec_fn=os.setsid,
    )

    try:
        print(f'Waiting {args.startup_wait_sec:.0f}s for Gazebo + SLAM startup...')
        time.sleep(args.startup_wait_sec)
        if launch.poll() is not None:
            print(f'Launch exited early with code {launch.returncode}')
            return 1

        drive_cmd = (
            f'{setup} && python3 {drive} --waypoints-file {waypoints}'
        )
        print('Driving warehouse loop for SLAM mapping...')
        drive_result = subprocess.run(
            ['bash', '-lc', drive_cmd],
            cwd=repo_root,
            timeout=args.drive_timeout_sec,
        )
        if drive_result.returncode != 0:
            print('Drive mission reported failures; serializing map anyway.')
    except subprocess.TimeoutExpired:
        print('Drive mission timed out; serializing map anyway.')
    finally:
        pass

    serialize_cmd = (
        f'{setup} && python3 {bootstrap} '
        f'--map-name warehouse --output-dir {maps_dir} '
        f'--timeout-sec {args.serialize_wait_sec}'
    )
    print('Serializing posegraph (sim must still be running)...')
    result = subprocess.run(['bash', '-lc', serialize_cmd], cwd=repo_root)

    if launch.poll() is None:
        os.killpg(os.getpgid(launch.pid), signal.SIGINT)
        try:
            launch.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(launch.pid), signal.SIGKILL)

    posegraph = os.path.join(maps_dir, 'warehouse.posegraph')
    data = os.path.join(maps_dir, 'warehouse.data')
    if result.returncode != 0 or not os.path.isfile(posegraph) or not os.path.isfile(data):
        print('Bootstrap failed. Drive the full warehouse manually, then run:')
        print(f'  python3 {bootstrap} --map-name warehouse --output-dir {maps_dir}')
        return 1

    print(f'Bootstrap complete: {posegraph} and {data}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
