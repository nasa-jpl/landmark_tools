from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as visualize_corr
import landmark_tools.landmark as landmark

def test_crop():
    """Crop the DEM directly to the target size, and compare with a two-step crop sequence that yields the same final size; results should be identical.

    Cropping a landmark will change the elevation values because the anchor point changes, but the structure should remain the same.
    """
    # Run executables
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", "output/Haworth_final_adj_5mpp_surf_tif_rendered_cropped1.lmk", 
             "-operation", "CROP", 
             "-roi", "150", "150", "200", "200"], 
            cwd= TEST_DIR)
    
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", "output/Haworth_final_adj_5mpp_surf_tif_rendered_cropped_rough.lmk", 
             "-operation", "CROP", 
             "-roi", "50", "50", "400", "400"], 
            cwd= TEST_DIR)
    
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", "output/Haworth_final_adj_5mpp_surf_tif_rendered_cropped_rough.lmk", 
             "-output", "output/Haworth_final_adj_5mpp_surf_tif_rendered_cropped2.lmk", 
             "-operation", "CROP", 
             "-roi", "100", "100", "200", "200"], 
            cwd= TEST_DIR)

    L_crop1 = landmark.Landmark( TEST_DIR / "output/Haworth_final_adj_5mpp_surf_tif_rendered_cropped1.lmk")
    L_crop2 = landmark.Landmark( TEST_DIR / "output/Haworth_final_adj_5mpp_surf_tif_rendered_cropped2.lmk")
    
    assert L_crop1 == L_crop2

def test_subset():
    """New DEM is a submatrix of the original raster. Elevation values should not change.
    """
    # Run executables
    width = 200
    height = 300
    x1 = 50
    y1 = 25
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", "output/Haworth_final_adj_5mpp_surf_tif_rendered_subset.lmk", 
             "-operation", "SUBSET", 
             "-roi", str(x1), str(y1), str(width), str(height)], 
            cwd= TEST_DIR)

    L_org = landmark.Landmark( TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")
    L_subset = landmark.Landmark( TEST_DIR / "output/Haworth_final_adj_5mpp_surf_tif_rendered_subset.lmk")
    
    np.testing.assert_allclose(L_org.ele[y1:y1+height, x1:x1+width], L_subset.ele)
    np.testing.assert_allclose(L_org.srm[y1:y1+height, x1:x1+width], L_subset.srm)

    assert L_org.BODY == L_subset.BODY
    assert L_org.lmk_id == L_subset.lmk_id
    assert L_subset.num_cols == width
    assert L_subset.num_rows == height
    assert L_subset.anchor_col == width/2
    assert L_subset.anchor_row== height/2
    assert L_org.resolution == L_subset.resolution
    np.allclose(L_org.mapRworld, L_subset.mapRworld, rtol=0, atol=1e-4)

    ##TODO? Anchor point also changes but by how much would require a calculation

def test_rescale():
    """By upscaling, then downscaling, we should return to the same map.
    """
    # Run executables
    scale = 0.5
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk", 
             "-output", "output/Haworth_final_adj_5mpp_surf_tif_rendered_upsample.lmk", 
             "-operation", "RESCALE", 
             "-scale", str(scale)], 
            cwd= TEST_DIR)
    
    run_cmd([ TOP_DIR / "build/edit_landmark", 
             "-input", "output/Haworth_final_adj_5mpp_surf_tif_rendered_upsample.lmk", 
             "-output", "output/Haworth_final_adj_5mpp_surf_tif_rendered_downsample.lmk", 
             "-operation", "RESCALE", 
             "-scale", str(1/scale)], 
            cwd= TEST_DIR)

    L_org = landmark.Landmark( TEST_DIR / "gold_standard_data/Haworth_final_adj_5mpp_surf_tif_rendered.lmk")
    L_scale = landmark.Landmark( TEST_DIR / "output/Haworth_final_adj_5mpp_surf_tif_rendered_downsample.lmk")
    
    assert L_org == L_scale