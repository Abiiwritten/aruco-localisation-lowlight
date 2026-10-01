#!/usr/bin/env python3
"""
Compare OptiTrack ground truth vs ArUco vision estimate, reading both
directly as PoseStamped topics from a rosbag2 (no TF replay needed --
both /optitrack/marker_pose and /aruco/pose_optitrack are already in
the 'world' frame).

For each /aruco/pose_optitrack sample, finds the nearest-in-time
/optitrack/marker_pose sample (within --max-time-diff) and computes:
  - translation error (m): euclidean distance between positions
  - rotation error (deg): angle of the relative rotation between
    orientations

Also cross-references /aruco/total_frames and /aruco/detected_frames
(if present in the bag) to report the actual detection rate for the
trial, rather than inferring it from message counts alone.

Usage:
    python3 compare_groundtruth_vs_vision.py lowlight_trial01 \
        --out results_trial01.csv
"""

import argparse
import csv

import numpy as np
from scipy.spatial.transform import Rotation

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def read_topic(bag_dir, topic_name):
    """Returns a list of (t_sec, position_xyz, quaternion_xyzw) tuples
    for every message on topic_name, in time order."""

    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_name not in type_map:
        return []

    msg_type = get_message(type_map[topic_name])
    samples = []

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ])
        quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ])
        samples.append((t_sec, pos, quat))

    return samples


def read_int32_topic_last_value(bag_dir, topic_name):
    """Returns the final Int32 value on a topic (e.g. a running counter),
    or None if the topic isn't in the bag."""

    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_name not in type_map:
        return None

    msg_type = get_message(type_map[topic_name])
    last_value = None

    while reader.has_next():
        topic, data, _t = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        last_value = msg.data

    return last_value


def nearest_match(t_query, samples, max_time_diff):
    """Linear nearest-neighbour search by timestamp. Fine for a few
    thousand samples; swap for bisect on sorted timestamps if this
    becomes a bottleneck on much larger bags."""

    best = None
    best_dt = max_time_diff
    for t_sec, pos, quat in samples:
        dt = abs(t_sec - t_query)
        if dt < best_dt:
            best_dt = dt
            best = (pos, quat)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_dir')
    parser.add_argument('--vision-topic', default='/aruco/pose_optitrack')
    parser.add_argument('--groundtruth-topic', default='/optitrack/marker_pose')
    parser.add_argument('--max-time-diff', type=float, default=0.05,
                         help='Max seconds between matched samples (default 50ms)')
    parser.add_argument('--out', default='results.csv')
    args = parser.parse_args()

    vision_samples = read_topic(args.bag_dir, args.vision_topic)
    gt_samples = read_topic(args.bag_dir, args.groundtruth_topic)

    print(f'{args.vision_topic}: {len(vision_samples)} samples')
    print(f'{args.groundtruth_topic}: {len(gt_samples)} samples')

    if not vision_samples or not gt_samples:
        print('ERROR: one of the topics had zero samples -- check topic '
              'names against what the bag actually contains '
              '(ros2 bag info <bag_dir>).')
        return

    rows = []
    for t_sec, vis_pos, vis_quat in vision_samples:
        match = nearest_match(t_sec, gt_samples, args.max_time_diff)
        if match is None:
            continue
        gt_pos, gt_quat = match

        trans_error = float(np.linalg.norm(vis_pos - gt_pos))

        r_vis = Rotation.from_quat(vis_quat)
        r_gt = Rotation.from_quat(gt_quat)
        rot_error_deg = float(
            np.degrees(np.linalg.norm((r_gt.inv() * r_vis).as_rotvec()))
        )

        rows.append((t_sec, trans_error, rot_error_deg))

    with open(args.out, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp_s', 'translation_error_m', 'rotation_error_deg'])
        writer.writerows(rows)

    print(f'\n{len(rows)} matched samples (of {len(vision_samples)} vision '
          f'samples) written to {args.out}')

    if rows:
        trans = np.array([r[1] for r in rows])
        rot = np.array([r[2] for r in rows])
        print(f'\nTranslation error (m):')
        print(f'  mean = {trans.mean():.4f}')
        print(f'  std  = {trans.std():.4f}')
        print(f'  RMSE = {np.sqrt((trans ** 2).mean()):.4f}')
        print(f'  max  = {trans.max():.4f}')
        print(f'\nRotation error (deg):')
        print(f'  mean = {rot.mean():.3f}')
        print(f'  std  = {rot.std():.3f}')
        print(f'  RMSE = {np.sqrt((rot ** 2).mean()):.3f}')
        print(f'  max  = {rot.max():.3f}')
    else:
        print('No samples matched within --max-time-diff -- check that '
              'both topics actually overlap in time, or widen the window.')

    total = read_int32_topic_last_value(args.bag_dir, '/aruco/total_frames')
    detected = read_int32_topic_last_value(args.bag_dir, '/aruco/detected_frames')
    if total is not None and detected is not None and total > 0:
        print(f'\nDetection rate: {detected}/{total} = '
              f'{100.0 * detected / total:.1f}%')


if __name__ == '__main__':
    main()
