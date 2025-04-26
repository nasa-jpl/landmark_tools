from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.visualize_corr as visualize_corr

def test_image_comparison_regression(tmp_path):
    """Compare the disparity map output of the current code to an archival copy
    
    TODO Currently testing a tolerance of 5m disparity. Check whether tighter bounds are desired
    """

    # Run executables
    run_cmd([ TOP_DIR / "build/image_comparison",
        "-base_image", TEST_DIR / "gold_standard_data/hillshade_intensity.pgm",
        "-base_nan_mask", TEST_DIR / "gold_standard_data/hillshade_mask.pgm",
        "-base_nan_max_count", "0",
        "-child_image", TEST_DIR / "gold_standard_data/kaguya_intensity.pgm",
        "-child_nan_mask", TEST_DIR / "gold_standard_data/kaguya_mask.pgm",
        "-child_nan_max_count", "0",
        "-warp", "image",
        "-output_dir", tmp_path,
        "-output_filename_prefix", "comparison",
        "-homography_max_dist_between_matching_keypoints", "0"], 
        cwd= TEST_DIR)

    filepath1 = str(tmp_path / "comparison")
    filepath2 = str(TEST_DIR / "gold_standard_data/image_comparison_")
    width = 1000
    height = 1000

    displacement_maps1 = visualize_corr.loadAllDisplacementMatrices(filepath1, width, height, save_image=False)
    displacement_maps2 = visualize_corr.loadAllDisplacementMatrices(filepath2, width, height, save_image=False)

    # Check changes
    for key,I1 in displacement_maps1.items():
        I2 = displacement_maps2[key]
        mask = np.logical_not(np.logical_or(np.isnan(I1), np.isnan(I2)))
        np.testing.assert_allclose(I1[mask], I2[mask], rtol=0, atol=5)


def test_image_comparison_self(tmp_path):
    """An image compared to itself should have zero disparity
    """

    # Run executables
    run_cmd([ TOP_DIR / "build/image_comparison",
        "-base_image", TEST_DIR / "gold_standard_data/hillshade_intensity.pgm",
        "-base_nan_mask", TEST_DIR / "gold_standard_data/hillshade_mask.pgm",
        "-base_nan_max_count", "0",
        "-child_image", TEST_DIR / "gold_standard_data/hillshade_intensity.pgm",
        "-child_nan_mask", TEST_DIR / "gold_standard_data/hillshade_mask.pgm",
        "-child_nan_max_count", "0",
        "-warp", "image",
        "-output_dir", tmp_path,
        "-output_filename_prefix", "comparison",
        "-homography_max_dist_between_matching_keypoints", "0"], 
        cwd= TEST_DIR)

    filepath1 = str(tmp_path / "comparison")
    width = 1000
    height = 1000

    displacement_maps1 = visualize_corr.loadAllDisplacementMatrices(filepath1, width, height, save_image=False)

    # Check changes
    for key,I1 in displacement_maps1.items():
        if(key != "correlation"):
            # Displacement should be low
            np.testing.assert_allclose(I1[np.logical_not(np.isnan(I1))], 0, rtol=0, atol=1)
        else:
            # Correlation should be high
            assert np.all(I1[np.logical_not(np.isnan(I1))] > 0.7), "Not all elements are greater than 0.7" 