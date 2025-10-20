import numpy as np

from logging import Logger
from typing import Optional
from scipy.spatial.transform import Rotation


def solve_icp(X: np.ndarray,
              P: np.ndarray,
              logger: Optional[Logger] = None) -> tuple[np.ndarray, Rotation]:
    """
    Performs one iteration of the ICP (iterative closest point) algorithm.

    Let X = {x1, ..., xN} and P = {p1, ..., pN} be two sets of cartesian positions,
    the algorithm reconstruct a rotation R and a translation t such that
        xi = R.pi + t
    for any i, by minimising the mean squared error after roto-translation.

    Algorithm is described at https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf
    Variable naming is consistent with those slides

    Parameters
    ----------
    X : np.ndarray
        3xN matrix describing the location of the points in the source reference frame
    P : np.ndarray
        3xN matrix describing the location of the points in the target reference frame
    logger : Optional[Logger]

    Returns
    -------
    tuple(np.ndarray, Rotation)
        A tuple containing the translation and the rotation object of the two 
        pointclouds
    """
    # Create dummy logger if no logger is provided
    logger = logger or Logger("dummy_logger")

    # Preliminary checks
    assert (len(X.shape) == 2)
    assert (len(P.shape) == 2)
    if (X.shape[0] != 3):
        logger.warn(f"Provided vector X have shape[0] = {X.shape[0]} != 3")
        X = X.T
    if (P.shape[0] != 3):
        logger.warn(f"Provided vector P have shape[0] = {P.shape[0]} != 3")
        P = P.T
    assert (X.shape[0] == 3)
    assert (P.shape[0] == 3)

    # Compute mean
    mu_X = X.mean(axis=1)
    mu_P = P.mean(axis=1)
    assert (len(mu_X.shape) == 1 and mu_X.shape[0] == 3)
    assert (len(mu_P.shape) == 1 and mu_P.shape[0] == 3)

    # Detrend data
    Xp = X - mu_X.reshape(3, 1)
    Pp = P - mu_P.reshape(3, 1)

    # Compute SVD to find rotation
    W = Pp @ Xp.T
    assert (len(W.shape) == 2 and W.shape[0] == 3 and W.shape[1] == 3)
    U, S, V = np.linalg.svd(W)
    # R = V @ U.T  # rotation matrix
    R = (U @ V).T  # rotation matrix
    assert (len(R.shape) == 2 and R.shape[0] == 3 and R.shape[1] == 3)

    # Compute translation vector
    t = mu_X - R@mu_P

    # Validate outcome
    logger.info("ICP problem solved")
    logger.debug(f"Computed translation: {t}")
    logger.debug(f"Computed rotation matrix:\n{R}")
    # reconstructed pointcloud as outcome of the roto-translation of points P:
    Xrec = t.reshape(3, 1) + R@P
    assert (Xrec.shape == X.shape)
    delta = X - Xrec
    cartesian_error = np.linalg.norm(delta, axis=0)  # pointwise reconstruction error
    mean_error = np.mean(cartesian_error)
    logger.info(f"Mean cartesian error: {mean_error}")

    return t, Rotation.from_matrix(R)


def log_header(logger: Logger, header: str):
    logger.info(f"{60*'='}")
    logger.info(f"{header.center(60)}")
    logger.info(f"{60*'='}")


if __name__ == "__main__":
    import sys
    import logging

    logger = logging.getLogger("ICP test")
    logger.setLevel(logging.DEBUG)
    default_logger_formatter = logging.Formatter("%(message)s")
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(default_logger_formatter)
    logger.addHandler(stdout_handler)

    log_header(logger, "Data creation")
    t = np.random.random(3)
    rpy = np.random.random(3)
    logger.info(f"Random translation of: {t}")
    logger.info(f"Random rotation of: {rpy} rads (roll, pitch, yaw)")

    rot = Rotation.from_euler("xyz", rpy)
    rot_mat = rot.as_matrix()
    logger.info("Rotation as matrix:")
    logger.info(rot_mat)

    P = np.random.random((3, 6))
    X = t.reshape(3, 1) + rot_mat@P

    log_header(logger, "Computing the solution")
    icp_sol = solve_icp(X, P, logger)
    trec: np.ndarray = icp_sol[0]
    Rrec: Rotation = icp_sol[1]

    log_header(logger, "Evaluation")
    rotation_error = (Rrec.inv() * rot).as_quat()
    rot_err_w = rotation_error[-1]
    t_err = t - trec
    logger.info(f"Translation error: {np.linalg.norm(t_err)}")
    logger.info(f"Rotation error w coeff. of quaternion: {abs(rot_err_w)}")
