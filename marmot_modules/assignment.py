from scipy.optimize import linear_sum_assignment
from pytorch3d.ops import box3d_overlap
import numpy as np

def euc_dist(det, track, prob_class_label, det_params, tracker):

    # Euclidean distance between positions
    return np.linalg.norm(det.pos[:,0] - track.spatial_state.mean()[0:3]) + class_mismatch_penalty

def iou3d(det,track):
    # TODO - compute

def compute_cost_matrix(tracker, prob_class_label, det_param):

    tracker.cost_matrix = np.zeros((len(tracker.dets),len(tracker.trks)))
    
    
    for ii,det in enumerate(tracker.dets):
        for jj,trk in enumerate(tracker.trks):

            # If classes don't match, add penalty

            # If classes do match, compute cost/affinity as appropriate and assign to 
            tracker.cost_matrix[ii,jj] = Cost(det, trk, prob_class_label, det_params, tracker)
    
def SolveCostMatrix(tracker):
    tracker.det_asgn_idx, tracker.trk_asgn_idx = linear_sum_assignment(tracker.cost_matrix)
    tracker.det_asgn_idx, tracker.trk_asgn_idx = list(tracker.det_asgn_idx), list(tracker.trk_asgn_idx)
    
    # If cost above threshold, remove the match from assignment vector
    assert(len(tracker.det_asgn_idx) == len(tracker.trk_asgn_idx))
    ii = len(tracker.det_asgn_idx)
    while ii:
        idx = ii-1
        if tracker.cost_matrix[tracker.det_asgn_idx[idx],tracker.trk_asgn_idx[idx]] > tracker.asgn_thresh:
            del tracker.det_asgn_idx[idx], tracker.trk_asgn_idx[idx]       
        ii -=1
    assert(len(tracker.det_asgn_idx) == len(tracker.trk_asgn_idx))

def compute_assignment(tracker):    
    compute_cost_matrix(tracker)
    solve_cost_matrix(tracker)