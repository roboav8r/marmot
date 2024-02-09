#!/usr/bin/env python3

from marmot.datatypes import Track

def valid_track(trk, trkr):

    if (trkr.obj_props[trk.obj_class_str]['delete_method']=='count' and trk.n_cons_misses >= trkr.obj_props[trk.obj_class_str]['n_delete_max']):
        return False
    else:
        return True

def create_tracks(trkr, det_name):
    trkr.get_logger().info("Create tracks")

    while trkr.dets:
        det = trkr.dets.pop()

        # TODO - add this once matching module implemented
        # if (len(tracker.dets)-1) in tracker.det_asgn_idx: # If detection at end of list is matched, remove it
        #     tracker.dets.pop()

        # Ignore tracks as appropriate
        if not trkr.detectors[det.det_name]['detection_params'][det.det_class_str]['ignore']:

            # Create new tracklets from all unmatched detections
            trkr.trks.append(Track(trkr, det))
            trkr.trk_id_count += 1

def delete_tracks(trkr):
    trkr.get_logger().info("Delete tracks")
    trkr.trks = [track for track in trkr.trks if valid_track(track, trkr)]