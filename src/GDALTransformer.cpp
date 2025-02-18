#include "GDALTransformer.hpp"
#include <iostream>


GDALCoordinateTransformer::GDALCoordinateTransformer(GDALDataset* dataset) 
{
    if (dataset->GetGeoTransform(adfGeoTransform) != CE_None) 
    {
        std::cerr << "Could not get geotransform." << std::endl;
    }
}

/**
 * @brief 将像素坐标转换为地理坐标
 *
 * @param pixelX 像素坐标X
 * @param pixelY 像素坐标Y
 * @param geoX 地理坐标X
 * @param geoY 地理坐标Y    
 */
void GDALCoordinateTransformer::pixelToGeo(double pixelX, double pixelY, double& geoX, double& geoY) 
{
    geoX = adfGeoTransform[0] + pixelX * adfGeoTransform[1] + pixelY * adfGeoTransform[2];
    geoY = adfGeoTransform[3] + pixelX * adfGeoTransform[4] + pixelY * adfGeoTransform[5];
}

/**
 * @brief 将地理坐标转换为像素坐标
 *
 * @param geoX 地理坐标X
 * @param geoY 地理坐标Y
 * @param pixelX 像素坐标X
 * @param pixelY 像素坐标Y
 */
void GDALCoordinateTransformer::geoToPixel(double geoX, double geoY, double &pixelX, double &pixelY)
 {
        double det = adfGeoTransform[1] * adfGeoTransform[5] - adfGeoTransform[2] * adfGeoTransform[4];
        if (det == 0) 
        {
            std::cerr << "Geotransform matrix is singular, cannot invert." << std::endl;
            return;
        }
    pixelX = (adfGeoTransform[5] * (geoX - adfGeoTransform[0]) - adfGeoTransform[2] * (geoY - adfGeoTransform[3])) / det;
    pixelY = (adfGeoTransform[1] * (geoY - adfGeoTransform[3]) - adfGeoTransform[4] * (geoX - adfGeoTransform[0])) / det;
}
