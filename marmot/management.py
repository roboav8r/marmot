#!/usr/bin/env python3

from marmot.datatypes import Detection, Track

def valid_track(trk, trkr):
    trkr.get_logger().info(str(trkr.obj_props[trk.class_str]))

    if trkr.obj_props[trk.class_str] TODO - come back here

    return False

def create_tracks(tracker, det_name):
    tracker.get_logger().info("Create tracks")

    while tracker.dets:
        # TODO - add this once matching module implemented
        # if (len(tracker.dets)-1) in tracker.det_asgn_idx: # If detection at end of list is matched, remove it
        #     tracker.dets.pop()

        tracker.trks.append(Track(tracker))
        tracker.trk_id_count += 1

def delete_tracks(tracker):
    tracker.get_logger().info("Delete tracks")
    tracker.trks = [track for track in tracker.trks if valid_track(track, tracker)]