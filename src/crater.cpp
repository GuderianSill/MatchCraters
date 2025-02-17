#include "crater.hpp"


double Crater::calculateEuclideanDistance(const std::vector<double> &coord1, const std::vector<double> &coord2)
{
    double dist = 0.0;
    for (size_t i = 0; i < coord1.size(); i++)
    {
        double diff = coord1[i] - coord2[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

double Crater::get_aspectRatio() const { return aspectRatio; }
std::vector<double> Crater::get_coordinates() const { return coordinates; }
int Crater::get_id() const { return id; }

int Crater::get_image_id() const { return image_id; }

double Crater::get_area() const { return area; }

double Crater::get_diameter() const { return diameter; }

double Crater::euclideanDistance(const Crater& start, const Crater& end)
{
    return calculateEuclideanDistance(start.coordinates, end.coordinates);
}

double Crater::getAngle(const Crater &start, const Crater &end)
{
    double dx = end.coordinates[0] - start.coordinates[0];
    double dy = end.coordinates[1] - start.coordinates[1];

    double angle = std::atan2(dy, dx);

    if (angle < 0)
        angle += 2 * M_PI;

    return angle * 180 / M_PI;
}

