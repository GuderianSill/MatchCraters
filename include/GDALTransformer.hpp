#ifndef GDAL_TRANSFORMER_HPP
#define GDAL_TRANSFORMER_HPP
#include <gdal.h>
#include <gdal_priv.h>

class GDALCoordinateTransformer 
{
private:
    // 地理变换参数
    double adfGeoTransform[6];

public:
    // 初始化地理变换参数
    GDALCoordinateTransformer(GDALDataset* dataset);

    // 将像素坐标转换为地理坐标
    void pixelToGeo(double pixelX, double pixelY, double& geoX, double& geoY);

    // 将地理坐标转换为像素坐标
    void geoToPixel(double geoX, double geoY, double& pixelX, double& pixelY);
};


#endif
