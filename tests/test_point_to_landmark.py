from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
PYTHON_SCRIPT_DIR = TOP_DIR / 'scripts/python/landmark_tools'
TEST_DIR = Path(__file__).resolve()
sys.path.insert(1, str(PYTHON_SCRIPT_DIR))
import landmark

def test_point_to_landmark():
    """Compare the ply output of the current code to an archival copy
    
    An elevation bias is expected because the point_2_landmark initializes the landmark tangent plane at the planetary radius 
    instead of the elevation of the center point of the landmark  
    """
    # Run executables
    run_cmd([ TOP_DIR / "build/point_2_landmark",
            "-p", "unit_test_data/final_3d_points.txt",
            "-l", "output/pointcloud.lmk",
            "-d", "10", 
            "-lt", "-9.11089",
            "-lg", "15.3501",
            "-s", "100000",
            "-sy", "30000",
            "-planet", "Moon",
            "-filetype", "POINT"], 
        cwd= TEST_DIR)

    L1 = landmark.Landmark("output/pointcloud.lmk")
    gt = landmark.Landmark("gold_standard_data/pointcloud_v3.lmk")

    # Check changes
    difference_ele = np.abs(L1.ele - gt.ele)
    assert np.all(np.less(difference_ele[np.logical_not(np.isnan(difference_ele))], 1))
    difference_srm = np.abs(L1.srm - gt.srm)
    assert np.all(np.less(difference_srm[np.logical_not(np.isnan(difference_srm))], 1))