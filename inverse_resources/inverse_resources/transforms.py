import os
import numpy as np
import yaml

from logging import Logger
from geometry_msgs.msg import (Vector3, Point, Pose, TransformStamped, Quaternion)
from scipy.spatial.transform import Rotation
from typing import cast, Optional, TypedDict, Iterable

from . import DEFAULT_DB_FILE


def to_pose_msg(pos: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    return pose


def to_point_msg(pos: np.ndarray) -> Point:
    pt = Point()
    pt.x = pos[0]
    pt.y = pos[1]
    pt.z = pos[2]
    return pt


def to_vec_msg(pos: np.ndarray) -> Vector3:
    return Vector3(x=pos[0], y=pos[1], z=pos[2])


def to_quaternion_msg(rot: Rotation | np.ndarray) -> Quaternion:
    q = None

    if isinstance(rot, Rotation):
        q = rot.as_quat()
    elif isinstance(rot, np.ndarray):
        q = rot
    else:
        raise TypeError(
            f"Don't know how to convert to quaternion message variable of type {type(rot)}"
        )
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class TransformDict(TypedDict):
    from_link: str
    to_link: str
    translation: list[float]
    rotation: list[float]


class TransformData:

    def __init__(
        self,
        from_link: str,
        to_link: str,
        translation: np.ndarray,
        rotation: np.ndarray
    ):
        self.from_link = from_link
        self.to_link = to_link
        self.translation = translation
        self.rotation = rotation

    def __str__(self) -> str:
        return (
            f"Transformation from {self.from_link} to {self.to_link}: "
            f"translation = {self.translation}, rotation = {self.rotation}"
        )

    def to_transform_msg(self, timestamp) -> TransformStamped:
        transf = TransformStamped()
        transf.header.frame_id = self.from_link
        transf.header.stamp = timestamp
        transf.child_frame_id = self.to_link
        transf.transform.translation = to_vec_msg(self.translation)
        transf.transform.rotation = to_quaternion_msg(self.rotation)
        return transf

    def to_dict(self) -> TransformDict:
        return TransformDict(
            from_link=self.from_link,
            to_link=self.to_link,
            translation=self.translation.tolist(),
            rotation=self.rotation.tolist()
        )

    @classmethod
    def from_dict(cls, data: TransformDict):
        return cls(
            data["from_link"],
            data["to_link"],
            np.array(data["translation"]),
            np.array(data["rotation"]),
        )


class TransformDatabase:

    def __init__(self, db_file: str, logger: Logger):
        self.db_file = db_file
        self.logger = logger
        self.transforms: dict[str, list[TransformData]] = dict()
        self.load()

    def load(self, verbose: bool = True):
        self.logger.info(f"Recoving transformation data from file {self.db_file}")
        self.transforms.clear()

        if os.path.exists(self.db_file):
            with open(self.db_file, "r") as f:
                data: dict = yaml.safe_load(f)

                self.transforms.clear()
                for key, value in data.items():
                    self.transforms[key] = [
                        TransformData.from_dict(tf_data) for tf_data in value
                    ]
        else:
            self.logger.warning(f"Unable to find file {self.db_file}")

        if verbose:
            self.logger.info("Rotation expressed as quaternions (qx, qy, qz, qw)")
            for key, value in self.transforms.items():
                self.logger.info(f"Transforms for setup '{key}' ({len(value)}):")
                for tf in value:
                    self.logger.info(f" - from {tf.from_link} to {tf.to_link}")
                    self.logger.info(f"   translation {tf.translation}")
                    self.logger.info(f"   rotation {tf.rotation}")

    def save(self):
        self.logger.debug(f"Saving database into {self.db_file}")
        out_dict = dict()

        # Create dictionary to be stored as yaml
        for key, value in self.transforms.items():
            out_dict[key] = [tf.to_dict() for tf in value]

        with open(self.db_file, "w") as f:
            yaml.dump(out_dict, f)
            self.logger.debug(f"Saving completed")

    def contains_transform(self, setup: str, from_link: str, to_link: str) -> bool:
        return any(
            t.from_link == from_link and t.to_link == to_link
            for t in self.transforms[setup]
        )

    def get_all_transforms(self, setup: str) -> Iterable[TransformData]:
        res = self.transforms.get(setup)
        if res is None:
            return list()
        return res

    def get_transform(self, setup: str, from_link: str,
                      to_link: str) -> Optional[TransformData]:
        # Check setup existance
        if self.transforms.get(setup) is None:
            return None

        # Find desired transform
        for tf in self.transforms[setup]:
            if tf.from_link == from_link and tf.to_link == to_link:
                return tf

        return None  # transform not found

    def add_transform(
        self,
        setup: str,
        from_link: str,
        to_link: str,
        translation: np.ndarray,
        rotation: np.ndarray
    ) -> TransformData:

        # Remove potentially duplicate transform
        if self.transforms.get(setup) is not None:
            self.transforms[setup] = [
                t for t in self.transforms[setup]
                if (t.from_link != from_link and t.to_link != to_link)
            ]
        else:
            self.transforms[setup] = list()

        # Append new transform
        new_transform = TransformData(from_link, to_link, translation, rotation)
        self.transforms[setup].append(new_transform)
        self.save()
        return new_transform


if __name__ == "__main__":
    import logging
    import sys

    logger = logging.getLogger("TransformDatabase")
    logger.setLevel(logging.DEBUG)

    # Avoid adding multiple handlers if the logger is reused
    if not logger.handlers:
        # Create a stream handler that outputs to stdout
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(logging.DEBUG)

        # Create a formatter and attach it to the handler
        formatter = logging.Formatter(
            fmt='[%(asctime)s] %(levelname)s - %(name)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)

        # Add the handler to the logger
        logger.addHandler(handler)

    db = TransformDatabase(DEFAULT_DB_FILE, logger)
