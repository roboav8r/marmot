from scipy.optimize import linear_sum_assignment
# from pytorch3d.ops import box3d_overlap
import numpy as np

def dist_3d(det, track):

    # Euclidean distance between positions
    return np.linalg.norm(det.pos[:,0] - track.spatial_state.mean()[0:3])

# def iou3d(det,track):
    # TODO - compute

def compute_cost_matrix(tracker):

    tracker.cost_matrix = np.zeros((len(tracker.dets),len(tracker.trks)))
    
    for ii,det in enumerate(tracker.dets):
        for jj,trk in enumerate(tracker.trks):

            # If classes don't match, add mismatch penalty
            # TODO

            # If classes do match, compute cost/affinity as appropriate and assign to cost matrix
            if tracker.obj_props[trk.obj_class_str]['sim_metric']=='dist_3d':
                tracker.cost_matrix[ii,jj] += dist_3d(det, trk)
            else:
                raise TypeError('Invalid similarity metric: %s' % tracker.obj_props[trk.obj_class_str]['sim_metric'])
       
def solve_cost_matrix(tracker):
    # Compute assignment vectors: each vector contains indices of matched detections and matched tracks
    if tracker.assignment_algo=='hungarian':
        tracker.det_asgn_idx, tracker.trk_asgn_idx = linear_sum_assignment(tracker.cost_matrix)
        tracker.det_asgn_idx, tracker.trk_asgn_idx = list(tracker.det_asgn_idx), list(tracker.trk_asgn_idx)
    else:
        raise TypeError('Invalid assignment algorithm: %s' % tracker.assignment_algo)
    
    # If cost above threshold, remove the match from assignment vector
    assert(len(tracker.det_asgn_idx) == len(tracker.trk_asgn_idx))
    ii = len(tracker.det_asgn_idx)
    while ii:
        idx = ii-1
        if tracker.cost_matrix[tracker.det_asgn_idx[idx],tracker.trk_asgn_idx[idx]] > tracker.obj_props[tracker.trks[tracker.trk_asgn_idx[idx]].obj_class_str]['match_thresh']:
            del tracker.det_asgn_idx[idx], tracker.trk_asgn_idx[idx]       
        ii -=1
    assert(len(tracker.det_asgn_idx) == len(tracker.trk_asgn_idx))

def compute_assignment(tracker):    
    compute_cost_matrix(tracker)
    solve_cost_matrix(tracker)