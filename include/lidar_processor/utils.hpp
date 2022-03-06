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
    float a;
    float b;
    float c;
    float d;
};

struct Line
{
    // Coeffcients of a line equation
    float m;
    float b;
};
}

using RModel = std::variant<Model::Plane, Model::Line>;

struct NormalVector
{
    float x;
    float y;
    float z;
};

inline void find_plane_coefficients(RModel& plane_model, const NormalVector& norm, const pcl::PointXYZI& point) noexcept
{
    // See if the passed in Model is truly a plane model
    if (const auto* plane = std::get_if<Model::Plane>(&plane_model); plane != nullptr)
    {
        float d = norm.x * point.x
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

inline float distance_from_plane(const RModel& plane_model, const pcl::PointXYZI& point) noexcept
{
    // Initialize to infinity
    float distance = std::numeric_limits<float>().infinity();

    // See if the passed in Model is truly a plane model
    if (const auto* plane = std::get_if<Model::Plane>(&plane_model); plane != nullptr)
    {
        // Calculate distance of point from plane

        // Find: | ax + by + cz - (-d) |
        float top_half = plane->a * point.x
                       + plane->b * point.y
                       + plane->c * point.z
                       - (-plane->d);
        top_half = std::abs(top_half);

        // Find: sqrt( a^2 + b^2 + c^2 )
        float bottom_half = plane->a * plane->a + plane->b * plane->b + plane->c * plane->c;
        bottom_half = sqrtf(bottom_half);

        // Divide numerator and denominator
        return top_half / bottom_half;
    }
    // Return: infinity if not a plane, this is an error.
    return distance;
}

inline void naive_fit(const RModel& model, pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr inliers, const float& threshold)
{   
    // Iterate over PointCloud with it[0] = x, it[1] = y, it[2] = z.
    pcl::PointCloud<pcl::PointXYZI>::iterator it = in_pcl->begin();
    while (it != in_pcl->end())
    {
        // check if point fits model
        pcl::PointXYZI point;
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;
        point.intensity = it->intensity;
        if(distance_from_plane(model, point) < threshold)
        {
            // NOTE: FILTER OUT POINT HERE
            it = in_pcl->erase(it);
            // inlier to plane model
            inliers->push_back(point);
        }
        else
        {
            it++;
        }
    }
}
}