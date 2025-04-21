These files are to be used for regression testing comparisons. 

The WY data is cropped from USGS Wyoming DEMs.
- [UTM_WY.tif](https://www.sciencebase.gov/catalog/item/60e686add34e2a7685cfec45)
- [equal_rect_WY.tif](https://www.sciencebase.gov/catalog/item/619c837cd34eb622f69328a7) 

These DEMs are originally provided as a UTM projection and converted to equal rectangular using gdal.

The landmark files were created from these DEMs using commit e52093a7b18132933e15866ccc51dc2653354c43 

The polar stereo DEM is a crop from Site 01 from [High-Resolution LOLA Topography for Lunar South Pole Sites](https://pgda.gsfc.nasa.gov/products/78) 

The landmark files were created from polar stereo DEMs using commit 7ff339f07fbc0e7599d24590a7f81c3a060ad30d due to inconsistancy with how polar stereo projection was handled in  
6828ae2084eeace3a21210e195c91de76c194923
 
