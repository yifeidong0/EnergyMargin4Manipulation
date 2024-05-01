from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition
import numpy as np
from scipy.spatial.transform import Rotation as R

DIRT_NUM = 5

class SweepEnergyMargin(Task):

    def init_task(self) -> None:
        broom = Shape('small_broom')
        success_sensor = ProximitySensor('success')
        dirts = [Shape('dirt' + str(i)) for i in range(DIRT_NUM)]
        conditions = [DetectedCondition(dirt, success_sensor) for dirt in dirts]
        self.register_graspable_objects([broom])
        self.register_success_conditions(conditions)

    def init_episode(self, index: int) -> List[str]:
        dirts = [Shape('dirt' + str(i)) for i in range(DIRT_NUM)]

        # Reset dirt mass
        mass = 0.01
        masses = [dirt.set_mass(mass) for dirt in dirts]
        masses = [dirt.get_mass() for dirt in dirts]
        
        # Reset dirt positions
        # positions = [dirt.set_position([0,0,1]) for dirt in dirts]
        positions = [dirt.get_position() for dirt in dirts]

        # def euler_to_rotation_matrix(roll, pitch, yaw):
        #     # Create a Rotation object from Euler angles specifying the rotation order as 'xyz'
        #     rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        #     # Convert to rotation matrix
        #     rotation_matrix = rotation.as_matrix()
        #     return rotation_matrix

        # Reset robot end-effector waypoints
        z_table = 0.752
        sweep_distance = 0.2
        # tool_gp_euler = [np.deg2rad(180), np.deg2rad(0), np.deg2rad(-90)]
        # tool_gp_R = euler_to_rotation_matrix(*tool_gp_euler)
        # tool_gp_t = np.array([0.08-0.08, 0.235-0.076, 1.077-0.752]).reshape(3,1)
        # tool_gp_T = np.vstack((np.hstack((tool_gp_R, tool_gp_t)),
        #                                 np.array([0, 0, 0, 1]))) # 4*4, gp frame: grasping point on the broom
        
        pre_world_tool_se2 = [0.3, 0.1, np.pi/6] # tool frame: frontier of the broom. Same orientation as the world.
        # yaw = pre_world_tool_se2[2]
        # pre_world_tool_R = euler_to_rotation_matrix(0, 0, yaw)
        # pre_world_tool_t = np.array(pre_world_tool_se2[:2] + [z_table,]).reshape((3,1))
        # pre_world_tool_T = np.vstack((np.hstack((pre_world_tool_R, pre_world_tool_t)),
        #                                 np.array([0, 0, 0, 1]))) # 4*4, gp frame: grasping point on the broom
        # print('pre_world_tool_T:', pre_world_tool_T)
        # print('tool_gp_T:', tool_gp_T)
        
        # post_in_tool_t = [0.0, -0.1, 0.0] # sweep forward by 0.1m
        # post_in_tool_t_ext = np.array(post_in_tool_t + [1,]).reshape((4,1))
        # post_world_tool_se2 = (pre_world_tool_T @ post_in_tool_t_ext).flatten()[:2].tolist() + [pre_world_tool_se2[-1],]
        # print('post_world_tool_se2:', (pre_world_tool_T @ post_in_tool_t_ext).flatten()) # right
        
        # yaw = post_world_tool_se2[2]
        # post_world_tool_R = euler_to_rotation_matrix(0, 0, yaw)
        # post_world_tool_t = np.array(post_world_tool_se2[:2] + [z_table,]).reshape((3,1))
        # post_world_tool_T = np.vstack((np.hstack((post_world_tool_R, post_world_tool_t)),
        #                                 np.array([0, 0, 0, 1])))

        # pre_world_gp_T = pre_world_tool_T @ tool_gp_T # 4*4, gp frame in world frame
        # post_world_gp_T = post_world_tool_T @ tool_gp_T
        # print('pre_world_gp_T:', pre_world_gp_T)
        # print('post_world_gp_T:', post_world_gp_T)

        # new_pose_se2 = [0.23, 0.23, -1.57] # TODO
        self.reset_waypoints(pre_world_tool_se2, sweep_distance)

        # TODO: Reset dirt number
        # TODO: Reset goal region position

        return ['sweep dirt to dustpan',
                'sweep the dirt up',
                'use the broom to brush the dirt into the dustpan',
                'clean up the dirt',
                'pick up the brush and clean up the table',
                'grasping the broom by its handle, clear way the dirt from the '
                'table',
                'leave the table clean']

    def variation_count(self) -> int:
        return 1

    # TODO: Waypoint planner given the dirt positions
    def plan_waypoint(self) -> None:
        pass

    def reset_waypoints(self, pre_ee_se2: np.ndarray, sweep_distance: float) -> None:
        # def rotation_matrix_to_euler(rotation_matrix):
        #     # Create a Rotation object from a rotation matrix
        #     rotation = R.from_matrix(rotation_matrix)
        #     # Convert to Euler angles, specifying the same rotation order used to create the matrix
        #     euler_angles = rotation.as_euler('xyz', degrees=False)
        #     return euler_angles

        # pre_world_gp_R = pre_world_gp_T[:3, :3]
        # pre_world_gp_euler = rotation_matrix_to_euler(pre_world_gp_R)
        # pre_world_gp_t = pre_world_gp_T[:3, 3].tolist()
        # post_world_gp_R = post_world_gp_T[:3, :3]
        # post_world_gp_euler = rotation_matrix_to_euler(post_world_gp_R)
        # post_world_gp_t = post_world_gp_T[:3, 3].tolist()

        self._waypoints = self._get_waypoints()
        z_float = 1.000
        z_slide = 0.892
        ee_euler = self._waypoints[0].get_waypoint_object().get_orientation()
        ee_euler[2] -= pre_ee_se2[2]
        post_ee_pos = [pre_ee_se2[0] + sweep_distance*np.sin(pre_ee_se2[2]), 
                       pre_ee_se2[1] - sweep_distance*np.cos(pre_ee_se2[2]),]
        # pre_euler = pre_world_gp_euler.tolist()
        # post_euler = post_world_gp_euler.tolist()

        # Floating pre-waypoint
        # print('pre_world_gp_t:', pre_world_gp_t)
        # print('pre_euler:', pre_euler)
        self._waypoints[1].get_waypoint_object().set_position(pre_ee_se2[:2]+[z_float,])
        self._waypoints[1].get_waypoint_object().set_orientation(ee_euler)
        # Sliding pre-waypoint
        self._waypoints[2].get_waypoint_object().set_position(pre_ee_se2[:2]+[z_slide,])
        self._waypoints[2].get_waypoint_object().set_orientation(ee_euler)
        # Sliding post-waypoint
        self._waypoints[3].get_waypoint_object().set_position(post_ee_pos+[z_slide,])
        self._waypoints[3].get_waypoint_object().set_orientation(ee_euler)
        # Floating post-waypoint
        self._waypoints[4].get_waypoint_object().set_position(post_ee_pos+[z_float,])
        self._waypoints[4].get_waypoint_object().set_orientation(ee_euler)