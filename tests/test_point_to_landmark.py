from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as landmark


# TODO (Cecilia) I discovered that the ASCII format is lossy. This might make this regression test un-regressable.
# def test_point_to_landmark_regression(tmp_path):
#     """Compare the ply output of the current code to an archival copy
    
#     An elevation bias is expected because the point_2_landmark initializes the landmark tangent plane at the planetary radius 
#     instead of the elevation of the center point of the landmark  
#     """
#     output_path = tmp_path / "pointcloud.lmk"
#     # Run executables
#     run_cmd([ TOP_DIR / "build/point_2_landmark",
#             "-p", TEST_DIR / "gold_standard_data/final_3d_points.txt",
#             "-l", output_path,
#             "-d", "10", 
#             "-lt", "-9.11089",
#             "-lg", "15.3501",
#             "-s", "100000",
#             "-sy", "30000",
#             "-planet", "Moon",
#             "-filetype", "POINT"], 
#         cwd= TEST_DIR)

#     L1 = landmark.Landmark(output_path)
#     gt = landmark.Landmark(TEST_DIR / "gold_standard_data/pointcloud_v3.lmk")

#     # Check changes
#     mask = np.logical_or(np.isnan(L1.ele), np.isnan(gt.ele))
#     L1.ele[mask] = 0
#     gt.ele[mask] = 0
#     np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_LMK_to_PLY_to_LMK(tmp_path):
    """Transform the LMK file to a PLY file and back. The result should be the same as the original.  

    The default point_2_landmark method smooths point contributions over several adjacent raster cells.
    For the result to be as similar to the orginal as possible, use the nearest neighbor setting instead (i.e. -smooth false)

    There are still some small differences between the elevation. Test is tolerant to 0.5 m 
    """
    ply_path = tmp_path / "pointcloud.ply"
    output_path = tmp_path / "pointcloud.lmk"
    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_2_point",
            "-landmark", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
            "-ply", ply_path,
            "-frame", "WORLD",
            "-filetype", "PLY_BIG_ENDIAN"], 
        cwd= TEST_DIR)
    
    assert Path(ply_path).exists()

    run_cmd([ TOP_DIR / "build/point_2_landmark",
            "-p", ply_path,
            "-l", output_path,
            "-d", "10", 
            "-lg", "338.0",
            "-lt", "-86.8",
            "-s", "10000",
            "-sy", "10000",
            "-ele", "1791",
            "-planet", "Moon",
            "-filetype", "PLY",
            "-frame", "WORLD",
            "-smooth", "false"], 
        cwd= TEST_DIR)
    
    assert Path(output_path).exists()

    L1 = landmark.Landmark(output_path)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")

    # Check changes
    assert L1.approx_equal(gt, 0.5)
