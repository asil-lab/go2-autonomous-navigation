/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 01-11-2024.
 */

#ifndef LTM_POINTCLOUD_FILTER__CROP_BOX_FILTER_NODE_HPP_
#define LTM_POINTCLOUD_FILTER__CROP_BOX_FILTER_NODE_HPP_

#include "ltm_pointcloud_filter/pointcloud_filter_node.hpp"

#include <pcl/filters/crop_box.h>

namespace LTM
{
  class CropBoxFilterNode : public PointCloudFilterNode
  {
    public:
      CropBoxFilterNode();
      ~CropBoxFilterNode() = default;

    private:
      void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) override;

      void initializeCropBox();

      pcl::CropBox<pcl::PointXYZ> m_crop_box_filter;

  };  // class CropBoxFilterNode
} // namespace LTM

#endif  // LTM_POINTCLOUD_FILTER__CROP_BOX_FILTER_NODE_HPP_
