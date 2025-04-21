from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.visualize_corr as visualize_corr

def test_landmark_comparison_regression(tmp_path):
    """Compare the disparity map output of the current code to an archival copy
    
    Using the gold standard landmark files to compare with so that we are only comparing differences in the comparison code, 
    not accumulated differences from the landmark creation, rendering, etc.  
    """

    output_prefix = tmp_path / "comparison"
    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_comparison",
        "-l1", TEST_DIR / "gold_standard_data/UTM_WY.lmk_demo.lmk",
        "-l2", TEST_DIR / "gold_standard_data/equal_rectangular_WY.lmk_demo.lmk",
        "-o", output_prefix], 
        cwd= TEST_DIR)

    filepath1 = str(output_prefix)
    filepath2 = str(TEST_DIR / "gold_standard_data/comparison_")
    width = 500
    height = 500

    displacement_maps1 = visualize_corr.loadAllDisplacementMatrices(filepath1, width, height, save_image=False)
    displacement_maps2 = visualize_corr.loadAllDisplacementMatrices(filepath2, width, height, save_image=False)

    # Check changes
    for key,I1 in displacement_maps1.items():
        I2 = displacement_maps2[key]
        np.testing.assert_allclose(I1, I2, rtol=0, atol=1)


def test_landmark_comparison_self(tmp_path):
    """A landmark compared to itself should have zero disparity
    """
    output_prefix = tmp_path / "comparison"

    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_comparison",
        "-l1", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
        "-l2", TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk",
        "-o", output_prefix], 
        cwd= TEST_DIR)

    filepath1 = str(output_prefix)
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
            assert np.all(I1 > 0.7), "Not all elements are greater than 0.7"
