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
#include <pcl/filters/uniform_sampling.h>
#include <urdf_parser/urdf_parser.h>

#include <iostream>

namespace LTM {

  RobotClusterRemoval::RobotClusterRemoval(rclcpp::Clock::SharedPtr clock) {
    // Initialize the transform listener
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  }

  bool RobotClusterRemoval::setRobotModel(const std::string &robot_description) {
    // Load the robot model from the URDF if it exists
    if (!m_robot_model.initFile(robot_description)) {
      std::cout << "Failed to load the robot model from URDF file." << std::endl;
      return false;
    }
    generateRobotMeshes();
    return true;
  }

  RobotClusterRemoval::~RobotClusterRemoval() {
    // Reset the robot model
    m_robot_model.clear();
    m_robot_meshes.clear();

    // Reset the transform listener
    m_tf_buffer.reset();
    m_tf_listener.reset();
  }

  void RobotClusterRemoval::setRobotMeshResolution(const double& resolution) {
    m_robot_mesh_resolution = resolution;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr RobotClusterRemoval::getRobotMesh(
    const std::string &link_name) const {
    // Check if the link exists
    try {
      return m_robot_meshes.at(link_name);
    } catch (const std::out_of_range &e) {
      std::cout << "Link " << link_name << " does not exist in the robot model." << std::endl;
      return nullptr;
    }
  }

  void RobotClusterRemoval::generateRobotMeshes() {
    // Initialize uniform sampling
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setRadiusSearch(m_robot_mesh_resolution);

    for (const auto &link : m_robot_model.links_) {
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
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);

      // Iterate over the mesh vertices
      for (unsigned int i = 0; i < scene->mMeshes[0]->mNumVertices; i++) {
        pcl::PointXYZ point;
        point.x = scene->mMeshes[0]->mVertices[i].x;
        point.y = scene->mMeshes[0]->mVertices[i].y;
        point.z = scene->mMeshes[0]->mVertices[i].z;
        cloud_original->push_back(point);
      }

      // Uniformly sample the point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
      uniform_sampling.setInputCloud(cloud_original);
      uniform_sampling.filter(*cloud_sampled);

      // Add the point cloud to the map
      m_robot_meshes[link.first] = cloud_sampled;
    }
  }

} // namespace LTM