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

const float& distance_from_plane(RModel plane_model, pcl::PointXYZI point)
{

}

void naive_fit(const RModel& model, const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pcl, pcl::PointCloud<pcl::PointXYZI>::Ptr inliers, const float& threshold)
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

// Below: WIP

class RANSAC
{
private:
    uint16_t k_; // max number of iterations allowed
    RModel model_;

public:
    explicit RANSAC(RModel& model) : model_{model}{}

    void fit_model()
    {
    }
};
}