#!/usr/bin/env python3

import argparse
import os
import math
import json
from typing import Dict, Tuple
from pathlib import Path

import numpy as np
from PIL import Image

from pyquaternion import Quaternion as PyQuaternion

from nuscenes.nuscenes import NuScenes
from nuscenes.map_expansion.map_api import NuScenesMap
import nuscenes.utils.splits as nuscenes_splits
from nuscenes.eval.common.utils import quaternion_yaw

import rosbag2_py
from rclpy.serialization import serialize_message
from builtin_interfaces.msg import Time
from foxglove_msgs.msg import SceneUpdate, SceneEntity, PoseInFrame, LocationFix, Grid, PackedElementField, CubePrimitive, LinePrimitive, FrameTransform, KeyValuePair, ModelPrimitive
from geometry_msgs.msg import Point, Vector3, Quaternion

EARTH_RADIUS_METERS = 6.378137e6
REFERENCE_COORDINATES = {
    "boston-seaport": [42.336849169438615, -71.05785369873047],
    "singapore-onenorth": [1.2882100868743724, 103.78475189208984],
    "singapore-hollandvillage": [1.2993652317780957, 103.78217697143555],
    "singapore-queenstown": [1.2782562240223188, 103.76741409301758],
}

class NuScenesToMcap():
    def __init__(self,args):

        self.nuscenes_dir = args.nuscenes_dir
        self.mcap_dir = args.mcap_dir
        self.lidar_detector = args.lidar_det
        self.dataset = args.dataset
        self.split_name = args.split

        self.lidar_det_string = "-" + self.lidar_detector if self.lidar_detector else ""

        # Map detection classes to nuScenes tracking classes to get color scheme
        self.lidar_class_map = {'car': 'vehicle.car',
                'truck': 'vehicle.truck',
                'construction_vehicle': 'vehicle.construction',
                'trailer': 'vehicle.trailer',
                'bus': 'vehicle.bus.rigid',
                'barrier': 'movable_object.barrier',
                'bicycle': 'vehicle.bicycle',
                'motorcycle': 'vehicle.motorcycle',
                'pedestrian': 'human.pedestrian.adult',
                'traffic_cone': 'movable_object.trafficcone'}

        # Create nuscenes objects
        self.nusc = NuScenes(version=self.dataset, dataroot=self.nuscenes_dir, verbose=True)
        self.split = eval('nuscenes_splits.' + self.split_name)

        # Load lidar detections for this split
        if self.split_name in ['val', 'mini_val', 'train', 'mini_train']:
            train_det_path = os.path.join(self.nuscenes_dir,'detection-' + self.lidar_detector, self.lidar_detector + '_train.json')
            self.lidar_dets = json.load(open(train_det_path))['results']
            val_det_path = os.path.join(self.nuscenes_dir,'detection-' + self.lidar_detector, self.lidar_detector + '_val.json')
            self.lidar_dets.update(json.load(open(val_det_path))['results'])

        elif self.split_name in ['test']:
            train_det_path = os.path.join(self.nuscenes_dir,'detection-' + self.lidar_detector, self.lidar_detector + '_test.json')
            self.lidar_dets = json.load(open(train_det_path))['results']

        # If output directory doesn't exist, create it
        if not os.path.exists(self.mcap_dir):
            os.mkdir(self.mcap_dir)

    # Member functions
    def get_translation(self,data):
        return Vector3(x=data["translation"][0], y=data["translation"][1], z=data["translation"][2])


    def get_rotation(self, data):
        return Quaternion(x=data["rotation"][1], y=data["rotation"][2], z=data["rotation"][3], w=data["rotation"][0])

    def get_time(self, data):
        t = Time()
        t.sec, msecs = divmod(data["timestamp"], 1_000_000)
        t.nanosec = msecs * 1000

        return t
    
    def get_ego_tf(self, ego_pose, stamp):
        ego_tf = FrameTransform()
        ego_tf.parent_frame_id = "map"
        ego_tf.timestamp = stamp
        ego_tf.child_frame_id = "base_link"
        ego_tf.translation = self.get_translation(ego_pose)
        ego_tf.rotation = self.get_rotation(ego_pose)
        return ego_tf
    
    def load_bitmap(self, dataroot: str, map_name: str, layer_name: str) -> np.ndarray:
        """render bitmap map layers. Currently these are:
        - semantic_prior: The semantic prior (driveable surface and sidewalks) mask from nuScenes 1.0.
        - basemap: The HD lidar basemap used for localization and as general context.

        :param dataroot: Path of the nuScenes dataset.
        :param map_name: Which map out of `singapore-onenorth`, `singepore-hollandvillage`, `singapore-queenstown` and
            'boston-seaport'.
        :param layer_name: The type of bitmap map, `semanitc_prior` or `basemap.
        """
        # Load bitmap.
        if layer_name == "basemap":
            map_path = os.path.join(dataroot, "maps", "basemap", map_name + ".png")
        elif layer_name == "semantic_prior":
            map_hashes = {
                "singapore-onenorth": "53992ee3023e5494b90c316c183be829",
                "singapore-hollandvillage": "37819e65e09e5547b8a3ceaefba56bb2",
                "singapore-queenstown": "93406b464a165eaba6d9de76ca09f5da",
                "boston-seaport": "36092f0b03a857c6a3403e25b4b7aab3",
            }
            map_hash = map_hashes[map_name]
            map_path = os.path.join(dataroot, "maps", map_hash + ".png")
        else:
            raise Exception("Error: Invalid bitmap layer: %s" % layer_name)

        # Convert to numpy.
        if os.path.exists(map_path):
            image = np.array(Image.open(map_path).convert("L"))
        else:
            raise Exception("Error: Cannot find %s %s! Please make sure that the map is correctly installed." % (layer_name, map_path))

        # Invert semantic prior colors.
        if layer_name == "semantic_prior":
            image = image.max() - image

        return image
    
    def get_scene_map(self, scene, image, stamp):
        x, y, w, h = self.scene_bounding_box(scene)
        img_x = int(x * 10)
        img_y = int(y * 10)
        img_w = int(w * 10)
        img_h = int(h * 10)
        img = np.flipud(image)[img_y : img_y + img_h, img_x : img_x + img_w]

        # img values are 0-255
        # convert to a color scale, 0=white and 255=black, in packed RGBA format: 0xFFFFFF00 to 0x00000000
        img = (255 - img) * 0x01010100
        # set alpha to 0xFF for all cells except those that are completely black
        img[img != 0x00000000] |= 0x000000FF

        msg = Grid()
        msg.timestamp = stamp
        msg.frame_id = "map"
        msg.cell_size.x = 0.1
        msg.cell_size.y = 0.1
        msg.column_count = img_w
        msg.row_stride = img_w * 4
        msg.cell_stride = 4
        a_field = PackedElementField()
        a_field.name = 'alpha'
        a_field.offset = 0
        a_field.type = 2
        b_field = PackedElementField()
        b_field.name = 'blue'
        b_field.offset = 1
        b_field.type = 2
        g_field = PackedElementField()
        g_field.name = 'green'
        g_field.offset = 2
        g_field.type = 2
        r_field = PackedElementField()
        r_field.name = 'red'
        r_field.offset = 3
        r_field.type = 2
        msg.fields.append(a_field)
        msg.fields.append(b_field)
        msg.fields.append(g_field)
        msg.fields.append(r_field)
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        msg.data = img.astype("<u4").tobytes()

        return msg

    def scene_bounding_box(self, scene, padding=75.0):
        box = [np.inf, np.inf, -np.inf, -np.inf]
        cur_sample = self.nusc.get("sample", scene["first_sample_token"])
        while cur_sample is not None:
            sample_lidar = self.nusc.get("sample_data", cur_sample["data"]["LIDAR_TOP"])
            ego_pose = self.nusc.get("ego_pose", sample_lidar["ego_pose_token"])
            x, y = ego_pose["translation"][:2]
            box[0] = min(box[0], x)
            box[1] = min(box[1], y)
            box[2] = max(box[2], x)
            box[3] = max(box[3], y)
            cur_sample = self.nusc.get("sample", cur_sample["next"]) if cur_sample.get("next") != "" else None
        box[0] = max(box[0] - padding, 0.0)
        box[1] = max(box[1] - padding, 0.0)
        box[2] = min(box[2] + padding, self.nusc_map.canvas_edge[0]) - box[0]
        box[3] = min(box[3] + padding, self.nusc_map.canvas_edge[1]) - box[1]
        return box
    
    def rectContains(self,rect, point):
        a, b, c, d = rect
        x, y = point[:2]
        return a <= x < a + c and b <= y < b + d
    
    def get_centerline_markers(self, scene, stamp):
        pose_lists = self.nusc_map.discretize_centerlines(1)
        bbox = self.scene_bounding_box(scene)

        contained_pose_lists = []
        for pose_list in pose_lists:
            new_pose_list = []
            for pose in pose_list:
                if self.rectContains(bbox, pose):
                    new_pose_list.append(pose)
            if len(new_pose_list) > 1:
                contained_pose_lists.append(new_pose_list)

        scene_update = SceneUpdate()
        for i, pose_list in enumerate(contained_pose_lists):
            entity = SceneEntity()
            entity.frame_id = "map"
            entity.timestamp = stamp
            entity.id = f"{i}"
            entity.frame_locked = True
            line = LinePrimitive()
            line.type = 0
            line.thickness = 0.1
            line.color.r = 51.0 / 255.0
            line.color.g = 160.0 / 255.0
            line.color.b = 44.0 / 255.0
            line.color.a = 1.0
            line.pose.orientation.w = 1.0
            for pose in pose_list:
                pt = Point()
                pt.x = float(pose[0])
                pt.y = float(pose[1])
                pt.z = 0.
                line.points.append(pt)

            entity.lines.append(line)
            scene_update.entities.append(entity)

        return scene_update
    
    def write_drivable_area(self, writer, nusc_map, ego_pose, stamp):
        translation = ego_pose["translation"]
        rotation = PyQuaternion(ego_pose["rotation"])
        yaw_radians = quaternion_yaw(rotation)
        yaw_degrees = yaw_radians / np.pi * 180
        patch_box = (translation[0], translation[1], 32, 32)
        canvas_size = (patch_box[2] * 10, patch_box[3] * 10)

        drivable_area = nusc_map.get_map_mask(patch_box, yaw_degrees, ["drivable_area"], canvas_size)[0]

        msg = Grid()
        msg.timestamp = stamp
        msg.frame_id = "map"
        msg.cell_size.x = 0.1
        msg.cell_size.y = 0.1
        msg.column_count = drivable_area.shape[1]
        msg.row_stride = drivable_area.shape[1]
        msg.cell_stride = 1
        area_field = PackedElementField()
        area_field.name = 'drivable_area'
        area_field.offset = 0
        area_field.type=2
        msg.fields.append(area_field)
        msg.pose.position.x = translation[0] - (16 * math.cos(yaw_radians)) + (16 * math.sin(yaw_radians))
        msg.pose.position.y = translation[1] - (16 * math.sin(yaw_radians)) - (16 * math.cos(yaw_radians))
        msg.pose.position.z = 0.01  # Drivable area sits 1cm above the map
        q = PyQuaternion(axis=(0, 0, 1), radians=yaw_radians)
        msg.pose.orientation.x = q.x
        msg.pose.orientation.y = q.y
        msg.pose.orientation.z = q.z
        msg.pose.orientation.w = q.w
        msg.data = drivable_area.astype(np.uint8).tobytes()

        writer.write("/driveable_area", serialize_message(msg), stamp.sec*10**9 + stamp.nanosec)

    def get_coordinate(self, ref_lat: float, ref_lon: float, bearing: float, dist: float) -> Tuple[float, float]:
        """
        Using a reference coordinate, extract the coordinates of another point in space given its distance and bearing
        to the reference coordinate. For reference, please see: https://www.movable-type.co.uk/scripts/latlong.html.
        :param ref_lat: Latitude of the reference coordinate in degrees, ie: 42.3368.
        :param ref_lon: Longitude of the reference coordinate in degrees, ie: 71.0578.
        :param bearing: The clockwise angle in radians between target point, reference point and the axis pointing north.
        :param dist: The distance in meters from the reference point to the target point.
        :return: A tuple of lat and lon.
        """
        lat, lon = math.radians(ref_lat), math.radians(ref_lon)
        angular_distance = dist / EARTH_RADIUS_METERS

        target_lat = math.asin(math.sin(lat) * math.cos(angular_distance) + math.cos(lat) * math.sin(angular_distance) * math.cos(bearing))
        target_lon = lon + math.atan2(
            math.sin(bearing) * math.sin(angular_distance) * math.cos(lat),
            math.cos(angular_distance) - math.sin(lat) * math.sin(target_lat),
        )
        return math.degrees(target_lat), math.degrees(target_lon)


    def derive_latlon(self, location: str, pose: Dict[str, float]):
        """
        For each pose value, extract its respective lat/lon coordinate and timestamp.

        This makes the following two assumptions in order to work:
            1. The reference coordinate for each map is in the south-western corner.
            2. The origin of the global poses is also in the south-western corner (and identical to 1).
        :param location: The name of the map the poses correspond to, ie: 'boston-seaport'.
        :param poses: All nuScenes egopose dictionaries of a scene.
        :return: A list of dicts (lat/lon coordinates and timestamps) for each pose.
        """
        assert location in REFERENCE_COORDINATES.keys(), f"Error: The given location: {location}, has no available reference."

        reference_lat, reference_lon = REFERENCE_COORDINATES[location]
        x, y = pose["translation"][:2]
        bearing = math.atan(x / y)
        distance = math.sqrt(x**2 + y**2)
        lat, lon = self.get_coordinate(reference_lat, reference_lon, bearing, distance)
        return lat, lon

    def get_car_scene_update(self, stamp) -> SceneUpdate:
        scene_update = SceneUpdate()
        entity = SceneEntity()
        entity.frame_id = "base_link"
        entity.timestamp = stamp
        entity.id = "car"
        entity.frame_locked = True
        model = ModelPrimitive()
        model.pose.position.x = 1.
        model.pose.orientation.w = 1.
        model.scale.x = 1.
        model.scale.y = 1.
        model.scale.z = 1.
        model.url = "https://assets.foxglove.dev/NuScenes_car_uncompressed.glb"
        model = entity.models.append(model)
        scene_update.entities.append(entity)
        return scene_update

    # Conversion functions
    def convert_split(self):

        for scene in self.split: # for scene in split
            print("Converting %s" % (scene))

            file_root = "%s%s" % (scene, self.lidar_det_string)

            # Check if file exists
            if os.path.exists(os.path.join(self.mcap_dir, file_root, file_root + '_0.mcap')):
                print('%s MCAP already exists. Continuing.' % scene)
                continue

            else:
                self.write_scene_to_mcap(scene,file_root)
            
    def write_scene_to_mcap(self,scene_name,file_root):

        # Create MCAP writer and topics
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri= os.path.join(self.mcap_dir, file_root), storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/map", type="foxglove_msgs/msg/Grid", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/semantic_map", type="foxglove_msgs/msg/SceneUpdate", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/driveable_area", type="foxglove_msgs/msg/Grid", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/annotations", type="foxglove_msgs/msg/SceneUpdate", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/detections", type="foxglove_msgs/msg/SceneUpdate", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/markers/car", type="foxglove_msgs/msg/SceneUpdate", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/sample_token", type="foxglove_msgs/msg/SceneUpdate", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/tf", type="foxglove_msgs/msg/FrameTransform", serialization_format="cdr"
            )
        )

        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/pose", type="foxglove_msgs/msg/PoseInFrame", serialization_format="cdr"
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name="/gps", type="foxglove_msgs/msg/LocationFix", serialization_format="cdr"
            )
        )

        # Lookup scene information
        for idx in range(len(self.nusc.scene)):
            if self.nusc.scene[idx]["name"]==scene_name:
                scene = self.nusc.scene[idx]

        log = self.nusc.get("log", scene["log_token"])
        location = log["location"]
        self.nusc_map = NuScenesMap(dataroot=self.nuscenes_dir, map_name=location)
        image = self.load_bitmap(self.nusc_map.dataroot, self.nusc_map.map_name, "basemap")
        cur_sample = self.nusc.get("sample", scene["first_sample_token"])    
        stamp = self.get_time(
            self.nusc.get("ego_pose",self.nusc.get("sample_data", cur_sample["data"]["LIDAR_TOP"])["ego_pose_token"],
            )
        )

        map_msg = self.get_scene_map(scene, image, stamp)
        centerlines_msg = self.get_centerline_markers(scene,stamp)

        writer.write("/map", serialize_message(map_msg), stamp.sec*10**9 + stamp.nanosec)
        writer.write("/semantic_map", serialize_message(centerlines_msg), stamp.sec*10**9 + stamp.nanosec)

        # Populate MCAP file    
        det_count = 0
        while cur_sample is not None:
            
            sample_lidar = self.nusc.get("sample_data", cur_sample["data"]["LIDAR_TOP"])
            ego_pose = self.nusc.get("ego_pose", sample_lidar["ego_pose_token"])
            stamp = self.get_time(ego_pose)

            # publish /tf
            writer.write("/tf", serialize_message(self.get_ego_tf(ego_pose, stamp)), stamp.sec*10**9 + stamp.nanosec)

            # /driveable_area occupancy grid
            self.write_drivable_area(writer, self.nusc_map, ego_pose, stamp)

            # publish /pose
            pose_in_frame = PoseInFrame()
            pose_in_frame.timestamp = stamp
            pose_in_frame.frame_id = "base_link"
            pose_in_frame.pose.orientation.w = 1.
            writer.write("/pose", serialize_message(pose_in_frame), stamp.sec*10**9 + stamp.nanosec)

            # publish /gps
            lat, lon = self.derive_latlon(location, ego_pose)
            gps = LocationFix()
            gps.latitude = lat
            gps.longitude = lon
            gps.altitude = self.get_translation(ego_pose).z
            writer.write("/gps", serialize_message(gps), stamp.sec*10**9 + stamp.nanosec)

            # publish /markers/annotations
            scene_update = SceneUpdate()
            for annotation_id in cur_sample["anns"]:
                ann = self.nusc.get("sample_annotation", annotation_id)
                marker_id = ann["instance_token"][:4]
                c = np.array(self.nusc.explorer.get_color(ann["category_name"])) / 255.0

                entity = SceneEntity()
                entity.frame_id = "map"
                entity.timestamp = stamp
                entity.id = marker_id
                entity.frame_locked = True

                category_metadata = KeyValuePair()
                category_metadata.key = "category"
                category_metadata.value = ann["category_name"]

                attribute_metadata = KeyValuePair()
                attribute_metadata.key = "attribute_token"
                attribute_metadata.value = next(iter(ann["attribute_tokens"])) if ann["attribute_tokens"] else ''

                visibility_metadata = KeyValuePair()
                visibility_metadata.key = "visibility_token"
                visibility_metadata.value = ann["visibility_token"]

                sample_metadata = KeyValuePair()
                sample_metadata.key = "sample_token"
                sample_metadata.value = ann["sample_token"]

                cube = CubePrimitive()
                cube.pose.position.x = ann["translation"][0]
                cube.pose.position.y = ann["translation"][1]
                cube.pose.position.z = ann["translation"][2]
                cube.pose.orientation.w = ann["rotation"][0]
                cube.pose.orientation.x = ann["rotation"][1]
                cube.pose.orientation.y = ann["rotation"][2]
                cube.pose.orientation.z = ann["rotation"][3]
                cube.size.x = ann["size"][1]
                cube.size.y = ann["size"][0]
                cube.size.z = ann["size"][2]
                cube.color.r = c[0]
                cube.color.g = c[1]
                cube.color.b = c[2]
                cube.color.a = 0.5

                entity.metadata.append(category_metadata)  
                entity.metadata.append(attribute_metadata)
                entity.metadata.append(visibility_metadata)
                entity.metadata.append(sample_metadata)

                entity.cubes.append(cube)
                scene_update.entities.append(entity)
        
            writer.write("/annotations", serialize_message(scene_update), stamp.sec*10**9 + stamp.nanosec)

            # publish detections
            scene_update = SceneUpdate()
            for det in self.lidar_dets[cur_sample['token']]:

                c = np.array(self.nusc.explorer.get_color(self.lidar_class_map[det["detection_name"]])) / 255.0

                entity = SceneEntity()
                entity.frame_id = "map"
                entity.timestamp = stamp
                entity.id = str(det_count)
                entity.frame_locked = True
                entity.lifetime.nanosec = 450000000 # just under half a second so detections clear before 2Hz 
                
                category_metadata = KeyValuePair()         
                category_metadata.key = "detection_name"
                category_metadata.value = det["detection_name"]

                score_metadata = KeyValuePair()                 
                score_metadata.key = "detection_score"
                score_metadata.value = str(det["detection_score"])
                
                attribute_metadata = KeyValuePair()  
                attribute_metadata.key = "attribute_name"
                attribute_metadata.value = det["attribute_name"] if det["attribute_name"] else ''

                sample_metadata = KeyValuePair()
                sample_metadata.key = "sample_token"
                sample_metadata.value = cur_sample['token']

                cube = CubePrimitive()
                cube.pose.position.x = det["translation"][0]
                cube.pose.position.y = det["translation"][1]
                cube.pose.position.z = det["translation"][2]
                cube.pose.orientation.w = det["rotation"][0]
                cube.pose.orientation.x = det["rotation"][1]
                cube.pose.orientation.y = det["rotation"][2]
                cube.pose.orientation.z = det["rotation"][3]
                cube.size.x = det["size"][1]
                cube.size.y = det["size"][0]
                cube.size.z = det["size"][2]
                cube.color.r = c[0]
                cube.color.g = c[1]
                cube.color.b = c[2]
                cube.color.a = det["detection_score"]

                entity.metadata.append(category_metadata)  
                entity.metadata.append(score_metadata)
                entity.metadata.append(attribute_metadata)
                entity.metadata.append(sample_metadata)
                entity.cubes.append(cube)
                scene_update.entities.append(entity)

                det_count +=1
            
            writer.write("/detections", serialize_message(scene_update), stamp.sec*10**9 + stamp.nanosec)

            # publish /markers/car
            writer.write("/markers/car", serialize_message(self.get_car_scene_update(stamp)), stamp.sec*10**9 + stamp.nanosec)

            # move to the next sample
            cur_sample = self.nusc.get("sample", cur_sample["next"]) if cur_sample.get("next") != "" else None

        # Close writer
        del writer
        print("Finished writing %s" % os.path.join(self.mcap_dir, file_root))

def main(args=None):
    parser = argparse.ArgumentParser()
    home_dir = Path.home()
    parser.add_argument(
        "--nuscenes-dir",
        "-n",
        default=home_dir / "nuscenes",
        help="Path to nuscenes data directory (input)",
    )
    parser.add_argument(
        "--mcap-dir",
        "-m",
        default=home_dir / "nuscenes/mcap",
        help="Path to mcap directory (output)",
    )
    parser.add_argument(
        "--lidar-det",
        "-l",
        default="megvii",
        help="3D LiDAR detector name, if used",
    )
    parser.add_argument(
        "--dataset",
        "-d",
        default="v1.0-mini",
        help="NuScenes dataset: v1.0-mini, v1.0-trainval, v1.0-test",
    )
    parser.add_argument(
        "--split",
        "-s",
        default="mini_train",
        help="NuScenes dataset split: mini_train, mini_val, train, val, test",
    )
    args = parser.parse_args()

    # Initialize converter object
    nuscenes_to_mcap = NuScenesToMcap(args)

    # Run conversion routine
    nuscenes_to_mcap.convert_split()

if __name__ == "__main__":
    main()