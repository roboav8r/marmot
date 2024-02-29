#!/usr/bin/env python3

from marmot.datatypes import Track

def valid_track(trk, trkr):

    if (trkr.obj_props[trk.obj_class_str]['delete_method']=='count' and trk.n_cons_misses >= trkr.obj_props[trk.obj_class_str]['n_delete_max']):
        return False
    elif (trkr.obj_props[trk.obj_class_str]['delete_method']=='conf' and trk.track_conf <= trkr.obj_props[trk.obj_class_str]['delete_thresh']):
        return False
    else:
        return True

def create_tracks(trkr, det_name):
    trkr.get_logger().info("Create tracks")

    while trkr.dets:

        det = trkr.dets.pop()
        
        # if (len(trkr.dets)) in trkr.det_asgn_idx: # If detection at end of list is matched, move on
        if (len(trkr.dets)) in trkr.matches[:,0]: # If detection at end of list is matched, move on
            continue

        else:

            # Ignore tracks as appropriate
            if trkr.detectors[det.det_name]['detection_params'][det.det_class_str]['ignore']:
                continue
            
            # If using confidence based method, do not create tracks with confidence below threshold
            elif trkr.obj_props[det.obj_class_str]['create_method']=='conf' and det.class_conf < trkr.obj_props[det.obj_class_str]['detect_thresh']:
                continue
            
            else:
                # Create new tracklets from all unmatched detections
                trkr.trks.append(Track(trkr, det))
                trkr.trk_id_count += 1

def delete_tracks(trkr):
    trkr.get_logger().info("Delete tracks")
    trkr.trks = [track for track in trkr.trks if valid_track(track, trkr)]