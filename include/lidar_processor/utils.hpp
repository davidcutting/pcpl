// MIT License
//
// Copyright (c) 2022 David Cutting
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cstdint>
#include <variant>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

namespace LidarProcessor
{

namespace Model
{
struct Plane
{
    // Coefficients of plane equation
    double a;
    double b;
    double c;
    double d;
};

struct Line
{
    // Coeffcients of a line equation
    double m;
    double b;
};
}

using RModel = std::variant<Model::Plane, Model::Line>;

struct NormalVector
{
    double x;
    double y;
    double z;
};

void find_plane_coefficients(RModel& plane_model, NormalVector norm, pcl::PointXYZI point)
{
    // See if the passed in Model is truly a plane model
    if (const auto* plane = std::get_if<Model::Plane>(&plane_model); plane != nullptr)
    {
        double d = norm.x * point.x
                 + norm.y * point.y
                 + norm.z * point.z;

        // Return plane coefficients
        plane_model = Model::Plane{norm.x, norm.y, norm.z, d};
    }
    else
    {
        assert(false && "Passed model which is not plane.");
    }
}

double distance_from_plane(RModel plane_model, const pcl::PointXYZI& point)
{
    // Initialize to infinity
    double distance = std::numeric_limits<double>().infinity();

    // See if the passed in Model is truly a plane model
    if (const auto* plane = std::get_if<Model::Plane>(&plane_model); plane != nullptr)
    {
        // Calculate distance of point from plane

        // Find: | ax + by + cz - (-d) |
        double top_half = plane->a * point.x
                        + plane->b * point.y
                        + plane->c * point.z
                        - (-plane->d);
        top_half = std::abs(top_half);

        // Find: sqrt( a^2 + b^2 + c^2 )
        double bottom_half = plane->a * plane->a + plane->b * plane->b + plane->c * plane->c;
        bottom_half = std::sqrt(bottom_half);

        // Divide numerator and denominator
        return top_half / bottom_half;
    }
    // Return: infinity if not a plane, this is an error.
    return distance;
}

void naive_fit(const RModel& model, pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr inliers, const float& threshold)
{   
    // Iterate over PointCloud with i[0] = x, i[1] = y, i[2] = z.
    for (pcl::PointXYZI& point : *in_pcl)
    {  
        // check if point fits model
        if(distance_from_plane(model, point) < threshold)
        {
            // inlier to plane model
            inliers->push_back(point);
        }
    }
}
}