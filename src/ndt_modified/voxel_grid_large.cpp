﻿/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/*
modified by Shiaoming.
*/

#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <voxel_grid_large.h>

#include <pcl/filters/passthrough.h>

typedef Eigen::Array<size_t, 4, 1> Array4size_t;

struct cloud_point_index_idx
{
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}

    bool operator<(const cloud_point_index_idx &p) const { return (idx < p.idx); }
};

///////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void
pcl::VoxelGridLarge<PointT>::applyFilter(PointCloud &output)
{
    // Has the input dataset been set already?
    if (!input_)
    {
        PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
        output.width = output.height = 0;
        output.points.clear();
        return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1;                    // downsampling breaks the organized structure
    output.is_dense = true;                 // we filter out invalid points

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    if (!filter_field_name_.empty()) // If we don't want to process the entire cloud...
        getMinMax3D<PointT>(input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_), static_cast<float> (filter_limit_max_), min_p, max_p,
                            filter_limit_negative_);
    else
        getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
    {
//        PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. 二分法.\n", getClassName().c_str());

        pcl::PointCloud<PointT> cloud1, cloud2,cloud1f, cloud2f;
        pcl::PassThrough<PointT> pass;

        if (dx > dy && dx > dz)
        {
            pass.setInputCloud(input_);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(min_p[0], min_p[0] + (max_p[0] - min_p[0]) / 2);
            pass.filter(cloud1);

            pass.setFilterLimitsNegative(true);
            pass.filter(cloud2);
        } else if (dy > dx && dy > dz)
        {
            pass.setInputCloud(input_);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_p[1], min_p[1] + (max_p[1] - min_p[1]) / 2);
            pass.filter(cloud1);

            pass.setFilterLimitsNegative(true);
            pass.filter(cloud2);
        } else
        {
            pass.setInputCloud(input_);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_p[2], min_p[2] + (max_p[2] - min_p[2]) / 2);
            pass.filter(cloud1);

            pass.setFilterLimitsNegative(true);
            pass.filter(cloud2);
        }

        VoxelGridLarge voxelGridLarge = *this;
        voxelGridLarge.setInputCloud(cloud1.makeShared());
        voxelGridLarge.filter(cloud1f);

        voxelGridLarge.setInputCloud(cloud2.makeShared());
        voxelGridLarge.filter(cloud2f);

        output = cloud1f + cloud2f;
        return;
    }

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int> (floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (floor(max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // Storage for mapping leaf and pointcloud indexes
    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(indices_->size());

    // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
    if (!filter_field_name_.empty())
    {
        // Get the distance field index
        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex(*input_, filter_field_name_, fields);
        if (distance_idx == -1)
            PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName().c_str(), distance_idx);

        // First pass: go over all points and insert them into the index_vector vector
        // with calculated idx. Points with the same idx value will contribute to the
        // same point of resulting CloudPoint
        for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
        {
            if (!input_->is_dense)
                // Check if the point is invalid
                if (!std::isfinite(input_->points[*it].x) ||
                    !std::isfinite(input_->points[*it].y) ||
                    !std::isfinite(input_->points[*it].z))
                    continue;

            // Get the distance value
            const uint8_t *pt_data = reinterpret_cast<const uint8_t *> (&input_->points[*it]);
            float distance_value = 0;
            memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

            if (filter_limit_negative_)
            {
                // Use a threshold for cutting out points which inside the interval
                if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                    continue;
            } else
            {
                // Use a threshold for cutting out points which are too close/far away
                if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                    continue;
            }

            int ijk0 = static_cast<int> (floor(input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
            int ijk1 = static_cast<int> (floor(input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
            int ijk2 = static_cast<int> (floor(input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

            // Compute the centroid leaf index
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
        }
    }
        // No distance filtering, process all data
    else
    {
        // First pass: go over all points and insert them into the index_vector vector
        // with calculated idx. Points with the same idx value will contribute to the
        // same point of resulting CloudPoint
        for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
        {
            if (!input_->is_dense)
                // Check if the point is invalid
                if (!std::isfinite(input_->points[*it].x) ||
                    !std::isfinite(input_->points[*it].y) ||
                    !std::isfinite(input_->points[*it].z))
                    continue;

            int ijk0 = static_cast<int> (floor(input_->points[*it].x * inverse_leaf_size_[0]) - static_cast<float> (min_b_[0]));
            int ijk1 = static_cast<int> (floor(input_->points[*it].y * inverse_leaf_size_[1]) - static_cast<float> (min_b_[1]));
            int ijk2 = static_cast<int> (floor(input_->points[*it].z * inverse_leaf_size_[2]) - static_cast<float> (min_b_[2]));

            // Compute the centroid leaf index
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
        }
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

    // Third pass: count output cells
    // we need to skip all the same, adjacent idx values
    unsigned int total = 0;
    unsigned int index = 0;
    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
    // Worst case size
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size())
    {
        unsigned int i = index + 1;
        while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
            ++i;
        if (i - index >= min_points_per_voxel_)
        {
            ++total;
            first_and_last_indices_vector.emplace_back(index, i);
        }
        index = i;
    }

    // Fourth pass: compute centroids, insert them into their final position
    output.points.resize(total);
    if (save_leaf_layout_)
    {
        try
        {
            // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
            uint32_t new_layout_size = div_b_[0] * div_b_[1] * div_b_[2];
            //This is the number of elements that need to be re-initialized to -1
            uint32_t reinit_size = std::min(static_cast<unsigned int> (new_layout_size), static_cast<unsigned int> (leaf_layout_.size()));
            for (uint32_t i = 0; i < reinit_size; i++)
            {
                leaf_layout_[i] = -1;
            }
            leaf_layout_.resize(new_layout_size, -1);
        }
        catch (std::bad_alloc &)
        {
            throw PCLException("VoxelGridLarge bin size is too low; impossible to allocate memory for layout",
                               "voxel_grid.hpp", "applyFilter");
        }
        catch (std::length_error &)
        {
            throw PCLException("VoxelGridLarge bin size is too low; impossible to allocate memory for layout",
                               "voxel_grid.hpp", "applyFilter");
        }
    }

    index = 0;
    for (const auto &cp : first_and_last_indices_vector)
    {
        // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
        unsigned int first_index = cp.first;
        unsigned int last_index = cp.second;

        // index is centroid final position in resulting PointCloud
        if (save_leaf_layout_)
            leaf_layout_[index_vector[first_index].idx] = index;

        //Limit downsampling to coords
        if (!downsample_all_data_)
        {
            Eigen::Vector4f centroid(Eigen::Vector4f::Zero ());

            for (unsigned int li = first_index; li < last_index; ++li)
                centroid += input_->points[index_vector[li].cloud_point_index].getVector4fMap();

            centroid /= static_cast<float> (last_index - first_index);
            output.points[index].getVector4fMap() = centroid;
        } else
        {
            CentroidPoint<PointT> centroid;

            // fill in the accumulator with leaf points
            for (unsigned int li = first_index; li < last_index; ++li)
                centroid.add(input_->points[index_vector[li].cloud_point_index]);

            centroid.get(output.points[index]);
        }

        ++index;
    }
    output.width = static_cast<uint32_t> (output.points.size());
}


#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>

#define PCL_INSTANTIATE_VoxelGridLarge(T) template class PCL_EXPORTS pcl::VoxelGridLarge<T>;

PCL_INSTANTIATE(VoxelGridLarge, PCL_XYZ_POINT_TYPES)