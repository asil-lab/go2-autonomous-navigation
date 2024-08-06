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
#include <pcl/filters/crop_box.h>
#include <pcl/filters/uniform_sampling.h>
#include <urdf_parser/urdf_parser.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>

namespace LTM {

  RobotClusterRemoval::RobotClusterRemoval(rclcpp::Clock::SharedPtr clock)
  {
    setClock(clock);
    initializeTransformListener(clock);
  }

  bool RobotClusterRemoval::setRobotModel(const std::string &robot_description)
  {
    // Load the robot model from the URDF if it exists
    if (!m_robot_model.initFile(robot_description)) {
      std::cout << "Failed to load the robot model from URDF file." << std::endl;
      return false;
    }
    generateRobotMeshes();
    return true;
  }

  RobotClusterRemoval::~RobotClusterRemoval()
  {
    // Reset the robot model
    m_robot_model.clear();
    m_robot_meshes.clear();

    // Reset the transform listener
    m_tf_buffer.reset();
    m_tf_listener.reset();
  }

  void RobotClusterRemoval::removeRobotCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output) const
  {
    // *cloud_output = *cloud_input;

    // @TODO: Finish implement this in the future. For now, a simple 3D box
    // // @TODO: draw robot mesh as pointclouds onto the cloud input
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_drawn(new pcl::PointCloud<pcl::PointXYZ>);
    // // drawRobotMeshes(cloud_input, cloud_drawn);
    // drawRobotMeshes(cloud_input, cloud_output);

    // // @TODO: cluster pointcloud output
    // // clusterRobotMeshes(cloud_output, cloud_output);

    // // @TODO: extract cluster on base origin

    // // @TODO: remove selected cluster of points

    // Transform the point cloud to the base frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    transformPointCloud(cloud_input, cloud_transformed, "base", cloud_input->header.frame_id);

    // // Remove points that are 30cm wide, 80cm long, and 30cm high around the base origin
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector4f min_point(-0.45, -0.5, -1.0, 1.0);
    Eigen::Vector4f max_point(0.45, 0.5, 0.0, 1.0);

    // Remove points that are within the box
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud_transformed);
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);
    crop_box.setNegative(true);
    crop_box.filter(*cloud_filtered);

    // Transform the point cloud back to the world frame
    transformPointCloud(cloud_filtered, cloud_output, cloud_input->header.frame_id, "base");
  }

  void RobotClusterRemoval::setRobotMeshResolution(const double& resolution)
  {
    m_robot_mesh_resolution = resolution;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr RobotClusterRemoval::getRobotMesh(
    const std::string &link_name) const
  {
    // Check if the link exists
    try {
      // Get the point cloud of the link mesh in the link frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      *cloud = *m_robot_meshes.at(link_name);
      cloud->header.frame_id = link_name;

      // Transform the point cloud to the world frame
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      transformPointCloud(cloud, cloud_transformed, "world", link_name); // @TODO: Parameterize the target frame

      return cloud_transformed;
    } catch (const std::out_of_range &e) {
      std::cout << "Link " << link_name << " does not exist in the robot model." << std::endl;
      return nullptr;
    }
  }

  void RobotClusterRemoval::transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, const std::string &target_frame, const std::string &source_frame) const
  {
    // Transform the point cloud from the source frame to the target frame
    pcl_ros::transformPointCloud(target_frame, rclcpp::Time(0), *cloud_input, source_frame, *cloud_output, *m_tf_buffer);

    // Set the frame ID
    cloud_output->header.frame_id = target_frame;
  }

  void RobotClusterRemoval::drawRobotMeshes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output) const
  {
    // Copy the pointcloud input into the pointcloud output
    *cloud_output = *cloud_input;

    // Iterate over robot meshes map, and add the transformed pointcloud
    for (const auto& robot_mesh : m_robot_meshes) {
      *cloud_output += *(getRobotMesh(robot_mesh.first));
    }
  }

  // void RobotClusterRemoval::clusterRobotMeshes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input,
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output) const
  // {
  //   // Cluster pointcloud input and select cluster on base origin
  //   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  //   tree->setInputCloud(cloud_input);

  //   // Create the Euclidean Cluster Extraction object
  //   std::vector<pcl::PointIndices> cluster_indices;
  //   pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_cluster_extraction;
  //   euclidean_cluster_extraction.setClusterTolerance(0.1);  // 10cm, @TODO: Parameterize this
  //   euclidean_cluster_extraction.setMinClusterSize(100);    // @TODO: Parameterize this
  //   euclidean_cluster_extraction.setMaxClusterSize(25000);  // @TODO: Parameterize this
  //   euclidean_cluster_extraction.setSearchMethod(tree);
  //   euclidean_cluster_extraction.setInputCloud(cloud_input);

  //   // Obtain the cluster indices from the input cloud
  //   euclidean_cluster_extraction.extract(cluster_indices);

  //   // Get the translation of base w.r.t. world using tf2 transform

  //   // Get the cluster index surrounding this translation

  //   // Filter the cluster points with this cluster index onto cloud_output
  // }

  void RobotClusterRemoval::generateRobotMeshes()
  {
    // Initialize uniform sampling
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setRadiusSearch(m_robot_mesh_resolution);

    for (const auto &link : m_robot_model.links_) {
      // Check if the link has a collision and visual geometry
      if (!link.second->collision || !link.second->visual) {
        continue;
      }

      // Skip links without visual geometry
      if (link.second->visual->geometry->type != urdf::Geometry::MESH) {
        continue;
      }

      // Get the mesh file path
      const std::string mesh_file_path = resolveMeshPath(
        std::static_pointer_cast<urdf::Mesh>(link.second->visual->geometry)->filename);

      // Load the mesh
      Assimp::Importer importer;
      const aiScene *scene = importer.ReadFile(mesh_file_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

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

      // // Recursively transform the orientation of this mesh according to parent link
      // auto parent_link = link.second->getParent();
      // while (parent_link->name != "base") {
      //   // Get the parent link name
      //   const std::string parent_link_name = parent_link->name;
      //   std::cout << "Parent : " << parent_link_name <<std::endl;

      //   // Get the parent link pose using joint origin and axis
      //   // TODO: Get joint by parent
      //   // urdf::JointSharedPtr joint = m_robot_model.joints_.at(parent_link_name);
      //   const auto joint = m_robot_model.joints_.
      //   tf2::Transform parent_link_pose;
      //   parent_link_pose.setOrigin(tf2::Vector3(joint->parent_to_joint_origin_transform.position.x,
      //     joint->parent_to_joint_origin_transform.position.y, joint->parent_to_joint_origin_transform.position.z));
      //   parent_link_pose.setRotation(tf2::Quaternion(joint->parent_to_joint_origin_transform.rotation.x,
      //     joint->parent_to_joint_origin_transform.rotation.y, joint->parent_to_joint_origin_transform.rotation.z,
      //     joint->parent_to_joint_origin_transform.rotation.w));

      //   // Transform the point cloud
      //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      //   pcl_ros::transformPointCloud(*cloud_sampled, *cloud_transformed, parent_link_pose);

      //   // Update the point cloud
      //   *cloud_sampled = *cloud_transformed;

      //   // Get the parent link of the parent link
      //   parent_link = parent_link->getParent();
      // }

      // Add the mesh point cloud to the map
      m_robot_meshes[link.first] = cloud_sampled;
    }
  }

  std::string RobotClusterRemoval::resolveMeshPath(const std::string &mesh_path_uri) const
  {
    const std::string prefix = "package://";
    if (mesh_path_uri.substr(0, prefix.size()) != prefix) {
      throw std::invalid_argument("Mesh path URI does not start with 'package://'.");
    }

    // Remove the prefix
    const std::string mesh_path = mesh_path_uri.substr(prefix.size());

    // Split the package and the path
    const size_t pos = mesh_path.find("/");
    if (pos == std::string::npos) {
      throw std::invalid_argument("Mesh path URI does not contain a '/'.");
    }

    // Get the package name and the relative path
    const std::string package_name = mesh_path.substr(0, pos);
    const std::string relative_path = mesh_path.substr(pos);

    return ament_index_cpp::get_package_share_directory(package_name) + relative_path;
  }

  void RobotClusterRemoval::initializeTransformListener(const rclcpp::Clock::SharedPtr clock)
  {
    // Initialize the transform listener
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  }

  void RobotClusterRemoval::setClock(const rclcpp::Clock::SharedPtr clock)
  {
    m_clock = clock;
  }

} // namespace LTM