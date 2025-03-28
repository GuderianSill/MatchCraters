#ifndef CRATER_H
#define CRATER_H
#include <vector>
#include <cmath>
#include <cstdio>
#include <memory>

class Crater
{
private:
    int id;
    std::vector<double> coordinates;//坐标
    double width;
    double height;
    double aspectRatio;             //宽高比
    double area;
    std::vector<std::shared_ptr<Crater>> neighbors;
    double diameter;    
    int image_id;
    static double calculateEuclideanDistance(const std::vector<double>& coord1, const std::vector<double>& coord2);

public:
    Crater(std::vector<double> coordinates, double width, double height, double area, double diameter, int id, int image_id):
        coordinates(coordinates), width(width), height(height), area(area), diameter(diameter), id(id), image_id(image_id)
        {aspectRatio = width / height;}

    Crater(double area, int id): area(area), id(id){}
    Crater(double area): area(area){}

    int get_id() const;
    std::vector<double> get_coordinates() const;
    double get_aspectRatio() const;
    double get_width() const;
    double get_height() const;
    double get_area() const;
    const std::vector<std::shared_ptr<Crater>>& get_neighbors() const;
    double get_diameter() const;
    int get_image_id() const;   

    static double euclideanDistance(const Crater& start, const Crater& end);
    static double getAngle(const Crater& start, const Crater& end);
};

#endif // CRATER_H
