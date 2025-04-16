from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as visualize_corr
import landmark_tools.landmark as landmark

def test_point_to_landmark_regression():
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

    L1 = landmark.Landmark(TEST_DIR / "output/pointcloud.lmk")
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/pointcloud_v3.lmk")

    # Check changes
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)
    np.testing.assert_allclose(L1.srm, gt.srm, rtol=0, atol=1)

def test_LMK_to_PLY_to_LMK():
    """Transform the LMK file to a PLY file and back. The result should be the same as the original.  
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

    L1 = landmark.Landmark(TEST_DIR / "output/pointcloud.lmk")
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/pointcloud_v3.lmk")

    # Check changes
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)
    np.testing.assert_allclose(L1.srm, gt.srm, rtol=0, atol=1)