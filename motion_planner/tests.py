import numpy as np

from motion_plan import within_tol


if __name__ == "__main__":
    # test within_tol
    target = np.array([4, 4, 4])
    test = np.array([3, 4, 5])
    assert within_tol(test, target, 1)
    assert not within_tol(test, target, 0.5)