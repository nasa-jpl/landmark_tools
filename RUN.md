# Run
Instructions for running DEM Comparison tools.

Build these tools by following [Install](INSTALL.md) instructions.

## Table of Contents

### File format conversions

1. [`create_landmark`](#geotiff_landmark) : Convert geotiff format to landmark format
2. [`point_2_landmark`](#pointcloud) : Convert point cloud to landmark format
3. [`landmark_2_point`](#pointcloud) : Convert landmark format to point cloud

### Manipulating landmark files
4. [`render_ply.py`](#hillshade) : Hillshade landmark files
5. [`add_srm`](#hillshade) : Add a surface reflectance map to a landmark file
6. [`distort_landmark`](#distort) : Add noise to a landmark file
7. [`edit_landmark`](#edit) : Crop or scale a landmark file

### Validating landmark files
6. [`landmark_registration`](#map_tie) : Fix map tie error
7. [`landmark_comparison`](#compare) : Compare landmark files

For an example of how to run these tools, see our [demo jupyter notebook](https://github.jpl.nasa.gov/lunamaps/landmark_tools/blob/main/example/MoonDemo.ipynb)

## Tools

<a id="geotiff_landmark"></a>
### create\_landmark

Convert a binary file or geotiff to landmark format. The first two options are for using a configuration file for binary image formats. The options after the OR describe the command line arguments that you can use for GeoTIFF files.

Requires input file to have equal rectangular, UTM, or polar stereographic projection.

```   
Usage for create_landmark:
------------------
  Required arguments:
    -projection <EQ_RECT or UTM or STEREO>
    -config_file <filename> - configuration file
  OR
    -geotif_file   <filename> - input dem tif file name
    -lmk_file   <filename> - output lmk file name
    -lmk_width_meters   <filename> - lmk col size
    -lmk_height_meters   <filename> - lmk row size
    -lmk_res   <filename> - lmk resolution
    -lmk_center_lat   <filename> - lmk center lat
    -lmk_center_long   <filename> - lmk center long
  Optional arguments:
    -planet <Moon or Earth> - (default Moon)
    -nodata_value <int> - (default NaN)
    -srm_file <filename> - png image file containing surface reflectance map
```

#### Make a configuration file

If you do want to use a binary file as input, you need to include a configuration file. Here are examples for different projection types.

  Polar Stereographic Example
  ```
  input.raw                  //input filename
  3200 3200 32               //dem width, dem height, bit depth
  -19000.000 -4000.000 5     //upper left corner x, y in polar stereographic coordinates, pixel size in meters
  -90.0 0.0                  //Latitude of standard parallel, Longitude of origin
  output.lmk                 //output filename
  8000 8000                  //output DEM size col rows in meters
  10                         //output DEM resolution in meters
  -89.480003 215.199997      //lat long degree center of landmark map
  1737400                    // Reference body radius in meters //TODO not used
  ```

  UTM Example
  ```
  input.raw                         //input filename
  9001  9001   16                   //dem size, bit depth
  549994.000 3460006.000 10 -105    //upper left corner x, y in UTM coordinates, pixel size in meters, center of UTM zone 
  output.lmk                        //output filename
  20000 20000                       //size col rows in m
  10                                //resolution  in m
  31.1825 -104.37                   //lat long degree center of landmark map
  63567523                          // Reference body radius in meters //TODO not used
  ```

  Equal Rectangular Example
  ```
  input.raw                         //input filename
  9001  9001   16                   //dem size, bit depth
  -108.0000556 45.0000556 0.00011    //upper left corner longitude, latitude, pixel size in degrees per pixel
  output.lmk                        //output filename
  20000 20000                       //size col rows in m
  10                                //resolution  in m
  31.1825 -104.37                   //lat long degree center of landmark map
  63567523                          // Reference body radius in meters //TODO not used
  ```

  Note: NO COMMAS!!  

  You can get the coordinates and pixel resolution from gdalinfo

  The center of the UTM zone is only required for UTM projection files. For other projections, like  WGS84, leave it out. 

 <a id="pointcloud"></a>
### point\_2\_landmark

```
Convert point cloud to landmark format.
Usage for point_2_landmark:
------------------
Required arguments:
    -p   <filename> - input pointcloud file.
    -l   <filename> - output lmkfile
    -d   <float> - resolution in meters per pixel
    -lt   <float> - latitude of center anchor point
    -lg   <float> - longitude of center anchor point
    -s   <float> - map width in meters
    -sy   <float> - map height in meters
    -planet <Moon|Earth|Mars> - (default Moon)
    -filetype <POINT|PLY> - file format of input file (default POINT)
    -frame <WORLD|LOCAL|RASTER> - reference frame of the input pointcloud (default WORLD)
```

### landmark\_2\_point

```
Write a landmark to a ply mesh or pointcloud
Usage for landmark_2_point:
------------------
  Required arguments:
    -landmark   <filename> - input lmkfile
    -ply  <filename> - output PLY filepath
  Optional arguments:
    -filetype <ASCII|PLY_LITTLE_ENDIAN|PLY_BIG_ENDIAN> - (default arch endian)
    -structure <POINTCLOUD|MESH> - (default MESH)
    -frame <WORLD|LOCAL|RASTER> - (default WORLD)
```


 <a id="hillshade"></a>
### render\_ply.py

Hillshade landmark files

If you are using imagery for the surface reflectance maps of all landmark files, there is no need to hillshade your landmark files. If one or both of the landmark files has no surface reflectance map, you will need to render a hillshaded version of your map. For example, the LOLA DEMs have no surface reflectance map because they are lidar measurements. LOLA DEMs need to be hillshaded.

Calculate sun elevation and azimuth in degrees of any imagery you will be comparing too. Dense correlation matching will work best if the sun angle of the imagery and the hillshading is the same. If no imagery will be used, choose a sun angle that provides good illumination of your map. 

```
usage: render_ply.py [-h] -height HEIGHT -width WIDTH [--save-blend-file]
                     file sun_elevation sun_azimuth output

Render given file with blender

positional arguments:
  file               .ply file defining a DEM mesh in a local reference frame
                     OR a .blend file from a previous run.
  sun_elevation      sun elevation in degrees
  sun_azimuth        sun azimuth in degrees
  output             path to output file of rendered image

options:
  -h, --help         show this help message and exit
  -height HEIGHT     height of render file in pixels.
  -width WIDTH       width of render file in pixels
  --save-blend-file  Save a new blend file for additional renders. This option
                     is recommended to accelerate additional renders.
```

Add the hillshaded image to your landmark file using `add_srm`

```
Usage for add_srm:
Adds a surface image to an existing landmark file. The image must be the same dimensions and resolution as landmark structure. It must also be in an orthographic projection.
------------------
  Required arguments:
    -input   <filename> - input landmark filepath
    -output   <filename> - output landmark filepath
    -srm   <filename> - input surface image
```

 <a id="distort"></a>
### distort\_landmark

```
Simulate map error
Usage for distort_landmark:
------------------
  Required arguments:
    -input   <filename> - input landmark filepath
    -output  <filename> - output landmark filepath
  Optional arguments:
    -translate <x meters> <y meters> <z meters> - simulates map tie error 
    -rotate <in-plane rotation degrees> - simulates map orientation error
    -random_displace <mean> <stddev> - simulates correlation noise with gaussian elevation displacement
```

 <a id="edit"></a>
### edit\_landmark

```
Crop or scale a landmark file
Usage for render_landmark:
------------------
  Required arguments:
    -input   <filename> - input landmark filepath
    -output   <filename> - output landmark filepath
    -operation   <CROP|RESCALE|SUBSET> - what operation to perform
  Optional arguments:
    -scale   <double> - scale for RESCALE operation
    -roi   <left> <top> <width> <height> - roi for crop and subset operations
```

 <a id="compare"></a>
### landmark\_comparison

```
Compare landmark files using a dense patch-based correlation matcher
Usage for landmark_comparison:
------------------
  Required arguments:
    -l1   <lmk_filepath> 
    -l2   <lmk_filepath> 
    -o    <output_prefix> 
    -c    <ftp_config_filepath> 
```

To generate figures and histograms from the output files, use 
`python ../scripts/python/landmark_tools/visualize_corr.py <output_prefix> <width> <height>`
  
  <a id="map_tie"></a> 
### landmark\_registration 

Fix map tie error

If the landmarks are not properly registered, the result of `landmark_comparison` will be sparse. To fix this error, use landmark_registration to detect feature matches and calculate a homography. 

It will create an output landmark file with the name `registered_<target_map>`. Use this registered map to compare to the `<base_map>` in the `landmark_comparison` for improved results. 

```
Reregister landmarks. The child landmark will be reprojected into the base landmark's reference frame.
Usage for landmark_register: 
------------------
  Required arguments:
    -bm   <filename> - base landmark
    -cm   <filename> - child landmark
```
