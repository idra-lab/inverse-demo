import numpy as np

from typing import Optional
from geometry_msgs.msg import PoseStamped, Pose


class PlannerPose:

    def __init__(
        self,
        position: np.ndarray,
        orientation: Optional[np.ndarray] = None,
        frame: str = "base_link"
    ):
        self.pos = position
        if orientation is not None:
            self.ori = orientation / np.linalg.norm(orientation)
        else:
            self.ori = np.array([1.0, 0.0, 0.0, 0.0])

        self.frame = frame

        assert self.pos.shape == (3, )
        assert self.ori.shape == (4, )

    def set_random_rotation(self):
        newori = np.random.rand(4)
        if newori[0] < 0.0:
            newori = -newori
        self.ori = newori / np.linalg.norm(newori)

    def to_pose_stamped_msg(self) -> PoseStamped:
        res = PoseStamped()
        res.header.frame_id = self.frame
        res.pose = self.to_pose_msg()
        return res

    def to_pose_msg(self) -> Pose:
        res = Pose()
        res.position.x = self.pos[0]
        res.position.y = self.pos[1]
        res.position.z = self.pos[2]
        res.orientation.w = self.ori[0]
        res.orientation.x = self.ori[1]
        res.orientation.y = self.ori[2]
        res.orientation.z = self.ori[3]
        return res
