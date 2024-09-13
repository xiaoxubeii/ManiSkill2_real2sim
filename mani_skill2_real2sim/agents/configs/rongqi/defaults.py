from copy import deepcopy
import numpy as np
from mani_skill2_real2sim.agents.controllers import *
from mani_skill2_real2sim.sensors.camera import CameraConfig
from mani_skill2_real2sim.utils.sapien_utils import look_at


class RongqiRobotDefaultConfig:
    def __init__(self):
        self.urdf_path = "{PACKAGE_ASSET_DIR}/descriptions/rm65b_description/urdf/rm_65_b_description.urdf"

        self.urdf_config = dict(
            _materials=dict(
                gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
            ),
            link=dict(
                fixed_plate_link=dict(
                    material="gripper",
                    patch_radius=0.01,
                    min_patch_radius=0.01,
                ),
                Link6=dict(
                    material="gripper", 
                    patch_radius=0.01,
                    min_patch_radius=0.01,
                ),
            ),
        )

        self.arm_joint_names = ["joint1", "joint2",
                                "joint3", "joint4", "joint5", "joint6"]
        self.gripper_joint_names = ["4C2_Joint1"]

        self.arm_stiffness = [1169.7891719504198, 730.0, 808.4601346394447,
                              1229.1299089624076, 1272.2760456418862, 1056.3326605132252]
        self.arm_damping = [330.0, 180.0, 152.12036565582588,
                            309.6215302722146, 201.04998711007383, 269.51458932695414]

        self.arm_force_limit = [200, 200, 100, 100, 100, 100]
        self.arm_friction = 0.0
        self.arm_vel_limit = 1.5
        self.arm_acc_limit = 2.0

        self.gripper_stiffness = 1000
        self.gripper_damping = 200
        self.gripper_force_limit = 60

        self.ee_link_name = "Link6"

    @property
    def controllers(self):
        arm_common_args = [
            self.arm_joint_names,
            -1.0, 1.0, np.pi / 2,
            self.arm_stiffness,
            self.arm_damping,
            self.arm_force_limit,
        ]
        arm_common_kwargs = dict(
            friction=self.arm_friction,
            ee_link=self.ee_link_name,
            normalize_action=False,
        )

        arm_controller = PDEEPoseControllerConfig(
            *arm_common_args, frame="ee", **arm_common_kwargs
        )

        gripper_controller = PDJointPosMimicControllerConfig(
            self.gripper_joint_names,
            0.015, 0.037,
            self.gripper_stiffness,
            self.gripper_damping,
            self.gripper_force_limit,
            normalize_action=True,
            drive_mode="force",
        )

        return {
            "default": {
                "arm": arm_controller,
                "gripper": gripper_controller
            }
        }

    @property
    def cameras(self):
        return [
            CameraConfig(
                uid="3rd_view_camera",
                p=[0.5, -0.5, 0.5],
                q=look_at([0, 0, 0], [1, 1, -1]).q,
                width=640,
                height=480,
                actor_uid="base_link",
            ),
        ]
