from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
PYTHON_SCRIPT_DIR = TOP_DIR / 'scripts/python/landmark_tools'
TEST_DIR = Path(__file__).resolve()
sys.path.insert(1, str(PYTHON_SCRIPT_DIR))
import visualize_corr

def test_landmark_comparison():
    """Compare the disparity map output of the current code to an archival copy
    
    Using the gold standard landmark files to compare with so that we are only comparing differences in the comparison code, 
    not accumulated differences from the landmark creation, rendering, etc.  
    """
    # Run executables
    run_cmd([ TOP_DIR / "build/landmark_comparison",
        "-l1", "gold_standard_data/UTM_WY.lmk_demo.lmk",
        "-l2", " gold_standard_data/equal_rectangular_WY.lmk_demo.lmk",
        "-o", " output/comparison "], 
        cwd= TEST_DIR)

    filepath1 = TEST_DIR / "output/comparison_"
    filepath2 = TEST_DIR / "gold_standard_data/comparison_"
    width = 500
    height = 500

    displacement_maps1 = visualize_corr.loadAllDisplacementMatrices(filepath1, width, height, save_image=False)
    displacement_maps2 = visualize_corr.loadAllDisplacementMatrices(filepath2, width, height, save_image=False)

    # Check changes
    for key,I1 in displacement_maps1.items():
        I2 = displacement_maps2[key]

        difference = np.abs(I1 - I2)
        assert np.all(np.less(difference[np.logical_not(np.isnan(difference))], 1))