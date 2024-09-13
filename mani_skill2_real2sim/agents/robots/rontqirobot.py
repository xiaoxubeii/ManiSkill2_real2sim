import numpy as np
import sapien.core as sapien
from sapien.core import Pose

from mani_skill2_real2sim.agents.base_agent import BaseAgent
from mani_skill2_real2sim.agents.configs.rongqi import defaults
from mani_skill2_real2sim.utils.common import compute_angle_between
from mani_skill2_real2sim.utils.sapien_utils import (
    get_entity_by_name,
    get_pairwise_contact_impulse,
)


class RongqiRobot(BaseAgent):
    _config: defaults.RongqiRobotDefaultConfig

    """
    Rongqi 6DoF robot
    links:
        [Actor(name="base_link"), Actor(name="Link1"), Actor(name="Link2"), Actor(name="Link3"), 
        Actor(name="Link4"), Actor(name="Link5"), Actor(name="Link6")]
    active_joints: 
        ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_limits:
        [[-3.1      3.1    ]
         [-2.268    2.268  ]
         [-2.355    2.355  ]
         [-3.1      3.1    ]
         [-2.233    2.233  ]
         [-6.28     6.28   ]]
    """

    @classmethod
    def get_default_config(cls):
        return defaults.RongqiRobotDefaultConfig()

    def __init__(
        self, scene, control_freq, control_mode=None, fix_root_link=True, config=None
    ):
        if control_mode is None:
            control_mode = "default"
        super().__init__(
            scene,
            control_freq,
            control_mode=control_mode,
            fix_root_link=fix_root_link,
            config=config,
        )

    def _after_init(self):
        super()._after_init()
        self.base_link = [x for x in self.robot.get_links()
                          if x.name == "base_link"][0]
