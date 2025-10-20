import yaml
import os.path
from ament_index_python import get_package_share_path
from logging import Logger
from typing import Optional
from visualization_msgs.msg import Marker
from .transforms import TransformData


class Element:

    def __init__(
        self, name: str, transform: TransformData, file_path: Optional[str] = None
    ):

        default_filepath = os.path.join(
            get_package_share_path("inverse_resources"),
            "meshes",
            name,
        )

        self.name = name
        self.transform = transform
        self.file_path = file_path or default_filepath

    def to_marker_msg(self) -> Marker:
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = self.transform.to_link
        msg.type = Marker.MESH_RESOURCE
        msg.mesh_resource = self.file_path
        return msg

    def to_dict(self) -> dict:
        res = dict()
        res["name"] = self.name
        res["transform"] = self.transform.to_dict
        res["file_path"] = self.file_path
        return res

    @classmethod
    def from_dict(cls, data: dict):
        return cls(
            data.get("name"),
            TransformData.from_dict(data.get("transform")),
            data.get("mesh_path"),
        )


class ElementDatabase:

    def __init__(
        self, elements: Optional[list] = None, logger: Optional[Logger] = None
    ):
        self.elements = elements
        self.logger = logger or Logger("dummy_logger")

    def save_to(self, dest: str):
        if self.elements is None:
            self.logger.warning(
                f"ElementDatabase has no elements, avoiding writing to {dest}"
            )
            return

        with open(dest, "w") as f:
            yaml.dump([e.to_dict() for e in self.elements], f)
