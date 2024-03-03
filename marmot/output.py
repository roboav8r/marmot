import numpy as np

from diagnostic_msgs.msg import KeyValue
from tracking_msgs.msg import Track3D, Tracks3D
from foxglove_msgs.msg import SceneEntity, SceneUpdate, ArrowPrimitive, CubePrimitive, TextPrimitive, KeyValuePair

def publish_tracks(tracker, pub_name):
    tracker.trks_msg = Tracks3D()
    tracker.trks_msg.header.frame_id = tracker.frame_id
    tracker.trks_msg.header.stamp = tracker.dets_msg.header.stamp
    tracker.trks_msg.metadata = tracker.dets_msg.metadata

    for trk in tracker.trks:

        # Check if tracklet meets track creation criteria
        if tracker.obj_props[trk.obj_class_str]['create_method']=="count":
            if trk.n_cons_matches < tracker.obj_props[trk.obj_class_str]['n_create_min']:
                continue
        elif tracker.obj_props[trk.obj_class_str]['create_method']=="conf":
            if trk.track_conf < tracker.obj_props[trk.obj_class_str]['active_thresh']:        
                continue
        else:
            raise TypeError('Invalid track creation method: %s' % tracker.obj_props[trk.obj_class_str]['create_method'])
        
        # Create track message
        trk_msg = Track3D()

        # Add track information to message
        trk_msg.track_id = trk.trk_id
        trk_msg.track_confidence = trk.track_conf

        # Populate track message for publication
        trk_msg.pose.pose.position.x = trk.spatial_state.mean()[0]
        trk_msg.pose.pose.position.y = trk.spatial_state.mean()[1]
        trk_msg.pose.pose.position.z = trk.spatial_state.mean()[2]

        # compute orientation from rpy
        cr = 1.
        sr = 0.
        cp = 1.
        sp = 0.
        cy = np.cos(trk.spatial_state.mean()[3])
        sy = np.sin(trk.spatial_state.mean()[3])
        trk_msg.pose.pose.orientation.w = cr*cp*cy + sr*sp*sy
        trk_msg.pose.pose.orientation.x = sr*cp*cy - cr*sp*sy
        trk_msg.pose.pose.orientation.y = cr*sp*cy + sr*cp*sy
        trk_msg.pose.pose.orientation.z = cr*cp*sy - sr*sp*cy
    
        trk_msg.bbox.size.x = trk.spatial_state.mean()[4]
        trk_msg.bbox.size.y = trk.spatial_state.mean()[5]
        trk_msg.bbox.size.z =  trk.spatial_state.mean()[6]

        # CP state variables: pos_x, pos_y, pos_z, yaw, length, width, height
        if tracker.obj_props[trk.obj_class_str]['model_type'] in ['cp']:
            # Add spatial information to message
            trk_msg.twist.twist.linear.x = 0.
            trk_msg.twist.twist.linear.y = 0.
            trk_msg.twist.twist.linear.z = 0.

        # pos_x, pos_y, pos_z, yaw, length, width, height, vel_x, vel_y, vel_z
        elif tracker.obj_props[trk.obj_class_str]['model_type'] in ['cvcy']:
            # Add spatial information to message
            trk_msg.twist.twist.linear.x = trk.spatial_state.mean()[7]
            trk_msg.twist.twist.linear.y = trk.spatial_state.mean()[8]
            trk_msg.twist.twist.linear.z = trk.spatial_state.mean()[9]

        elif tracker.obj_props[trk.obj_class_str]['model_type'] in ['cvcy_obj']:
            # Add spatial information to message
            trk_msg.twist.twist.linear.z = trk.spatial_state.mean()[9]
            trk_msg.twist.twist.linear.x = trk.spatial_state.mean()[7]*cy - trk.spatial_state.mean()[8]*sy
            trk_msg.twist.twist.linear.y = trk.spatial_state.mean()[7]*sy + trk.spatial_state.mean()[8]*cy 
        elif tracker.obj_props[trk.obj_class_str]['model_type'] in ['ctra']:
            # Add spatial information to message
            trk_msg.twist.twist.linear.z = 0.
            trk_msg.twist.twist.angular.z = trk.spatial_state.mean()[9]
            trk_msg.twist.twist.linear.x = trk.spatial_state.mean()[7]*cy
            trk_msg.twist.twist.linear.y = trk.spatial_state.mean()[7]*sy
        elif tracker.obj_props[trk.obj_class_str]['model_type'] in ['ack']:
            # Add spatial information to message
            trk_msg.twist.twist.linear.z = 0.
            trk_msg.twist.twist.angular.z = trk.spatial_state.mean()[7]*trk.spatial_state.mean()[8]
            trk_msg.twist.twist.linear.x = trk.spatial_state.mean()[7]*cy
            trk_msg.twist.twist.linear.y = trk.spatial_state.mean()[7]*sy
        else:
            raise AttributeError('Invalid process model type.')
    
        # Add semantic information to message
        trk_msg.track_confidence = float(trk.track_conf)
        trk_msg.class_confidence = float(trk.class_conf)
        trk_msg.class_string = trk.obj_class_str

        tracker.trks_msg.tracks.append(trk_msg)

    tracker.trks_msg.metadata.append(KeyValue(key='time_tracks_published', value=str(tracker.get_clock().now().nanoseconds)))
    tracker.trks_msg.metadata.append(KeyValue(key='num_tracks_published', value=str(len(tracker.trks))))
    
    # Publish populated message
    exec('tracker.%s.publish(tracker.trks_msg)' % pub_name)

def publish_scene(tracker, pub_name):
    # Create scene message
    tracker.scene_msg = SceneUpdate()

    for trk in tracker.trks:

        # Check if tracklet meets track creation criteria
        if tracker.obj_props[trk.obj_class_str]['create_method']=="count":
            if trk.n_cons_matches < tracker.obj_props[trk.obj_class_str]['n_create_min']:
                continue
        else:
            raise TypeError('Invalid track creation method: %s' % tracker.obj_props[trk.obj_class_str]['create_method'])

        # Create track message
        entity_msg = SceneEntity()

        # Add track information to message
        entity_msg.frame_id = tracker.frame_id
        entity_msg.timestamp = tracker.dets_msg.header.stamp
        entity_msg.id = str(trk.trk_id)
        entity_msg.frame_locked = True
        entity_msg.lifetime.nanosec = 500000000

        cube = CubePrimitive()
        cube.pose.position.x = trk.spatial_state.mean()[0]
        cube.pose.position.y = trk.spatial_state.mean()[1]
        cube.pose.position.z = trk.spatial_state.mean()[2]

        cr = 1.
        sr = 0.
        cp = 1.
        sp = 0.
        cy = np.cos(trk.spatial_state.mean()[3])
        sy = np.sin(trk.spatial_state.mean()[3])

        cube.pose.orientation.w = cr*cp*cy + sr*sp*sy
        cube.pose.orientation.x = sr*cp*cy - cr*sp*sy
        cube.pose.orientation.y = cr*sp*cy + sr*cp*sy
        cube.pose.orientation.z = cr*cp*sy - sr*sp*cy
        cube.size.x = trk.spatial_state.mean()[4]
        cube.size.y = trk.spatial_state.mean()[5]
        cube.size.z = trk.spatial_state.mean()[6]
        cube.color.a = 0.1
        entity_msg.cubes.append(cube)

        # Add velocity information to message
        vel_arrow_x = ArrowPrimitive()
        vel_arrow_x.pose = cube.pose
        vel_arrow_x.shaft_length = trk.spatial_state.mean()[3]
        vel_arrow_x.shaft_diameter = .1
        vel_arrow_x.head_length = .2
        vel_arrow_x.shaft_diameter = .1
        vel_arrow_x.color.a = 0.5
        entity_msg.arrows.append(vel_arrow_x)

        # Add semantic information to message
        text = TextPrimitive()
        text.billboard = True
        text.font_size = 12.
        text.scale_invariant = True
        text.color.a = 1.0
        text.pose.position.x = trk.spatial_state.mean()[0]
        text.pose.position.y = trk.spatial_state.mean()[1]
        text.pose.position.z = trk.spatial_state.mean()[2]
        text.text = "%s-%.0f: %.0f %%" % (trk.obj_class_str, trk.trk_id, trk.class_conf*100)
        entity_msg.texts.append(text)

        # Add metadata
        name_md = KeyValuePair()
        name_md.key = 'class_name'
        name_md.value = trk.obj_class_str
        entity_msg.metadata.append(name_md)

        score_md = KeyValuePair()
        score_md.key = 'class_score'
        score_md.value = str(trk.class_conf)
        entity_msg.metadata.append(score_md)

        att_md = KeyValuePair()
        att_md.key = 'attribute'
        att_md.value = '' 
        entity_msg.metadata.append(att_md)

        sample_md = KeyValuePair()
        sample_md.key = trk.metadata[0].key
        sample_md.value = trk.metadata[0].value
        entity_msg.metadata.append(sample_md)

        trk_md = KeyValuePair()
        trk_md.key = 'track_score'
        trk_md.value = str(trk.track_conf)
        entity_msg.metadata.append(trk_md)

        tracker.scene_msg.entities.append(entity_msg)
    
    # Publish scene message
    exec('tracker.%s.publish(tracker.scene_msg)' % pub_name)