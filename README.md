# Landmark Tools

This set of tools is provided for validating digital elevation maps (DEMs) when no ground truth is available. They are designed for comparing DEMs to each other and quantifying the agreement between them. If two independent data products produce similar elevation measurements, we can be confident in the DEM accuracy. 

Our tools use a data format for representing the DEMs called a landmark file.

The tools provided here include tools to:
- Create landmark files from point clouds
- Create landmark files from geotiffs with UTM, polar stereo, and equal rectangular projections 
- Render landmark files with hillshading
- Compare landmark files using dense correlation
- Visualize the correlation between landmark files 

**Note: This repository has submodules. Please clone with the following steps**

```
git clone git@github.jpl.nasa.gov:lunamaps/landmark_tools.git
cd landmark_tools
git submodule update --init --recursive
```

To pull updates do 

```
git pull
git submodule sync
git submodule update --init --recursive
```

## Recommended Data Sources

We recommend using the [High-Resolution LOLA Topography for Lunar South Pole Sites](https://pgda.gsfc.nasa.gov/products/78) by Michael Kenneth Barker for validating other Lunar DEMs. These maps are available as geotiffs with a polar stereo projection. 

## Documentation

- [Install](INSTALL.md)
- [Run](RUN.md)
- [Demo Jupyter Notebook](https://github.jpl.nasa.gov/lunamaps/landmark_tools/blob/main/example/MoonDemo.ipynb)

## Support

This code is provided "as-is," without warranties or guarantees of functionality. While we do not commit to ongoing maintenance, we will monitor issues and provide limited support for a period of three months from the initial release date. We encourage users to report bugs or issues, and we will make reasonable efforts to address critical concerns during this period.

## Acknowledgements 

The research was carried out at the Jet Propulsion Laboratory, California Institute of Technology, under a contract with the National Aeronautics and Space Administration (80NM0018D0004). © 2024. California Institute of Technology. Government sponsorship acknowledged.
