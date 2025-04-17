from pathlib import Path
import sys
from test_helpers import run_cmd
import numpy as np

# Path to the top-level repo directory
TOP_DIR = Path(__file__).resolve().parent.parent
TEST_DIR = Path(__file__).resolve().parent
import landmark_tools.landmark as landmark

def write_geographic_config_file(tmp_path):
    config_path = tmp_path / "make_geographic_lmk.txt"
    
    config = \
"""input:
  filename: gold_standard_data/equal_rectangular_WY.raw
  width_px: 1311 
  height_px: 938 
  bit_depth: 32
  upper_left_x_projection_unit: -107.5081027 
  upper_left_y_projection_unit: 44.5936521 
  pixel_resolution_projection_unit : 0.000098783643545
output:
  filename: {}/equal_rectangular_WY.lmk
  width_px: 500
  height_px: 500
  pixel_resolution_meters: 10
  center_latitude: 44.55
  center_longitude: -107.44
""".format(tmp_path)
    
    with open(config_path, 'w') as fp:
        fp.write(config)
    
    return config_path

def test_legacy_configuration_geographic_regression(tmp_path):
    """Compare the landmark file created using a configuration file and the current code to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    output = tmp_path / "equal_rectangular_WY.lmk"
    config_path = write_geographic_config_file(tmp_path)
    
    # Run create_landmark
    run_cmd([ TOP_DIR / "build/create_landmark", 
              "-projection", "GEOGRAPHIC",
              "-planet", "Earth",
              "-nodata_value", "-999999",
              "-config_file", config_path], 
              cwd= TEST_DIR)

    assert Path(output).exists()

    # Check changes
    L1 = landmark.Landmark( output)
    gt = landmark.Landmark( TEST_DIR / "gold_standard_data/equal_rectangular_WY.lmk")

    # Elevation difference less than 1m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_geotiff_configuration_geographic_regression(tmp_path):
    """Compare the landmark file created by automatically reading geotiff metadata to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    output = tmp_path / "equal_rectangular_WY_geotif.lmk"
    run_cmd([TOP_DIR / "build/create_landmark",
              "-geotif_file", "gold_standard_data/equal_rectangular_WY.tif",
                "-lmk_file", output,
                "-lmk_width_meters", "5000",
                "-lmk_height_meters", "5000",
                "-lmk_res", "10",
                "-lmk_center_lat", "44.55",
                "-lmk_center_long", "-107.44",
                "-planet", "Earth"], 
                cwd= TEST_DIR)

    assert Path(output).exists()

    # Check changes
    L1 = landmark.Landmark(output)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/equal_rectangular_WY.lmk")

    # Elevation difference less than 1 m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def write_utm_config_file(tmp_path):
    config_path = tmp_path / "make_utm_lmk.txt"
    
    config = \
"""input:
  filename: gold_standard_data/UTM_WY.raw
  width_px: 1118
  height_px: 1118 
  bit_depth: 32
  upper_left_x_projection_unit: 300920.718 
  upper_left_y_projection_unit: 4940572.392
  pixel_resolution_projection_unit : 8.944543828264759
  latitude_standard_parallel : 0
  longitude_natural_origin : -105
output:
  filename: {}/UTM_WY.lmk
  width_px: 500
  height_px: 500
  pixel_resolution_meters: 10
  center_latitude: 44.55
  center_longitude: -107.44
""".format(tmp_path)
    
    with open(config_path, 'w') as fp:
        fp.write(config)
    
    return config_path

def test_legacy_configuration_utm_regression(tmp_path):
    """Compare the landmark file created using a configuration file and the current code to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    output = tmp_path / "UTM_WY.lmk"
    config_path = write_utm_config_file(tmp_path)
    # Run create_landmark
    run_cmd([TOP_DIR / "build/create_landmark", 
              "-projection", "UTM",
              "-planet", "Earth",
              "-nodata_value", "-999999",
              "-config_file", config_path], 
              cwd= TEST_DIR)

    assert Path(output).exists()

    # Check changes
    L1 = landmark.Landmark(output)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/UTM_WY.lmk")

    # Elevation difference less than 1 m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_geotiff_configuration_utm_regression(tmp_path):
    """Compare the landmark file created by automatically reading geotiff metadata to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1m
    """
    # Run create_landmark
    output = tmp_path / "UTM_WY_geotif.lmk"
    run_cmd([TOP_DIR / "build/create_landmark",
              "-geotif_file", "gold_standard_data/UTM_WY.tif",
                "-lmk_file", output,
                "-lmk_width_meters", "5000",
                "-lmk_height_meters", "5000",
                "-lmk_res", "10",
                "-lmk_center_lat", "44.55",
                "-lmk_center_long", "-107.44",
                "-planet", "Earth"], 
                cwd= TEST_DIR)

    assert Path(output).exists()

    # Check changes
    L1 = landmark.Landmark(output)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/UTM_WY.lmk")

    # Elevation difference less than 1m
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def write_polar_stereo_config_file(tmp_path):
    config_path = tmp_path / "make_polar_stereo_lmk.txt"
    
    config = \
"""input : 
  filename : gold_standard_data/polarstereo_moon.raw 
  width_px : 1000
  height_px : 1000
  bit_depth : 32
  upper_left_x_projection_unit : -19000.000
  upper_left_y_projection_unit : -4000.000
  pixel_resolution_projection_unit : 16
  latitude_standard_parallel : -90.0
  longitude_natural_origin : 0.0
output :
  filename : {}/polarstereo_moon.lmk
  width_px : 500
  height_px : 500
  pixel_resolution_meters : 20
  center_latitude : -89.480003
  center_longitude : 215.199997
""".format(tmp_path)
    
    with open(config_path, 'w') as fp:
        fp.write(config)
    
    return config_path

def test_legacy_configuration_stereo_regression(tmp_path):
    """Compare the landmark file created using a configuration file and the current code to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    output = tmp_path / "polarstereo_moon.lmk"
    config_path = write_polar_stereo_config_file(tmp_path)

    # Run create_landmark
    run_cmd([TOP_DIR / "build/create_landmark", 
              "-projection", "STEREO",
              "-planet", "Moon",
              "-config_file", config_path], 
              cwd="tests/")

    assert Path(output).exists()

    # Check changes
    L1 = landmark.Landmark(output)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/polarstereo_moon.lmk")

    # Elevation difference less than 1 m
    diff = np.abs(L1.ele-gt.ele)
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)

def test_geotiff_configuration_stereo_regression(tmp_path):
    """Compare the landmark file created by automatically reading geotiff metadata to an archival copy
    
    Updates have changed the bilinear interpolation method, so some small differences in value are expected. 
    This test has a tolerance of 1 m
    """
    # Run create_landmark
    output = tmp_path / "polarstereo_moon_geotif.lmk"
    run_cmd([TOP_DIR / "build/create_landmark",
              "-geotif_file", "gold_standard_data/polarstereo_moon.tif",
                "-lmk_file", output,
                "-lmk_width_meters", "10000",
                "-lmk_height_meters", "10000",
                "-lmk_res", "20",
                "-lmk_center_lat", "-89.480003",
                "-lmk_center_long", "215.199997",
                "-planet", "Moon"], 
                cwd= TEST_DIR)

    assert Path(output).exists()

    # Check changes
    L1 = landmark.Landmark(output)
    gt = landmark.Landmark(TEST_DIR / "gold_standard_data/polarstereo_moon.lmk")

    # Elevation difference less than 1 m
    diff = np.abs(L1.ele-gt.ele)
    np.testing.assert_allclose(L1.ele, gt.ele, rtol=0, atol=1)