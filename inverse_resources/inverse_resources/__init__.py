import os
from ament_index_python import get_package_share_path

THIS_PACKAGE_SHARE = get_package_share_path("inverse_resources")
DEFAULT_DB_FILE = os.path.join(THIS_PACKAGE_SHARE, "setup.yaml")
