from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as landmark

def test_point_to_landmark_regression(tmp_path):
    """Compare the ply output of the current code to an archival copy
    
    An elevation bias is expected because the point_2_landmark initializes the landmark tangent plane at the planetary radius 
    instead of the elevation of the center point of the landmark  
    """
    output_path = tmp_path / "pointcloud.lmk"
    # Run executables
    run_cmd([ TOP_DIR / "build/point_2_landmark",
            "-p", "gold_standard_data/final_3d_points.txt",
            "-l", output_path,
            "-d", "10", 
            "-lt", "-9.11089",
            "-lg", "15.3501",
            "-s", "100000",
            "-sy", "30000",
            "-planet", "Moon",
            "-filetype", "POINT"], 
        cwd= TEST_DIR)

    L1 = landmark.Landmark(output_path)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/pointcloud_v3.lmk")

    # Check changes
    mask = np.logical_not(np.logical_or(np.isnan(L1.ele), np.isnan(gt.ele)))
    np.testing.assert_allclose(L1.ele[mask], gt.ele[mask], rtol=0, atol=1)
    np.testing.assert_allclose(L1.srm[mask], gt.srm[mask], rtol=0, atol=1)

def test_LMK_to_PLY_to_LMK(tmp_path):
    """Transform the LMK file to a PLY file and back. The result should be the same as the original.  
    """
    ply_path = tmp_path / "pointcloud.ply"
    output_path = tmp_path / "pointcloud.lmk"
    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_2_point",
            "-landmark", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
            "-ply", ply_path,
            "-frame", "WORLD"], 
        cwd= TEST_DIR)
    
    assert Path(ply_path).exists()

    run_cmd([ TOP_DIR / "build/point_2_landmark",
            "-p", ply_path,
            "-l", output_path,
            "-d", "10", 
            "-lt", "-86.8",
            "-lg", "15.3501",
            "-s", "10000",
            "-sy", "10000",
            "-planet", "Moon",
            "-filetype", "PLY"], 
        cwd= TEST_DIR)
    
    assert Path(output_path).exists()

    L1 = landmark.Landmark(output_path)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")

    # Check changes
    assert L1 == gt