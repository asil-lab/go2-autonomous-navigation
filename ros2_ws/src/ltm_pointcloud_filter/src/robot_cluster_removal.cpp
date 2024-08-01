/*
 * Project Lava Tube Mapping, Technical University of Delft.
 * Author: Alexander James Becoy @alexanderjamesbecoy
 * Date: 30-07-2024.
 */

#include <ltm_pointcloud_filter/robot_cluster_removal.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <urdf_parser/urdf_parser.h>

namespace LTM {

  RobotClusterRemoval::RobotClusterRemoval(rclcpp::Clock::SharedPtr clock) {
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  }

  void RobotClusterRemoval::setRobotModel(const std::string &robot_description) {
    urdf::Model model;
    if (!model.initString(robot_description)) {
        return;
    }
    m_robot_model_ = model;
  }

  void RobotClusterRemoval::generateRobotMeshes() {
    for (const auto &link : m_robot_model_.links_) {
      // Skip links without visual geometry
      if (link.second->visual->geometry->type != urdf::Geometry::MESH) {
        continue;
      }

      // Get the mesh file path
      const auto mesh = std::static_pointer_cast<urdf::Mesh>(link.second->visual->geometry);
      const std::string mesh_path = mesh->filename;

      // Load the mesh
      Assimp::Importer importer;
      const aiScene *scene = importer.ReadFile(mesh_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

      // Check if the mesh was loaded
      if (!scene || scene->mNumMeshes == 0) {
        continue;
      }

      // Create a new point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

      // Iterate over the mesh vertices
      for (unsigned int i = 0; i < scene->mMeshes[0]->mNumVertices; i++) {
        pcl::PointXYZ point;
        point.x = scene->mMeshes[0]->mVertices[i].x;
        point.y = scene->mMeshes[0]->mVertices[i].y;
        point.z = scene->mMeshes[0]->mVertices[i].z;
        cloud->push_back(point);
      }

      // Add the point cloud to the map
      m_robot_meshes[link.first] = cloud;
    }
  }

} // namespace LTM