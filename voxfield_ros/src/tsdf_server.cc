#include "voxfield_ros/tsdf_server.h"

#include <chrono>
#include <utility>

#include "voxfield_ros/conversions.h"
#include "voxfield_ros/kindr_conversions.h"
#include "voxfield_ros/param_utils.h"
#include "voxfield_ros/ros_params.h"

namespace voxfield {
namespace {

std::unique_ptr<TsdfIntegratorBase> makeTsdfIntegrator(
    const std::string& method, const TsdfIntegratorBase::Config& config,
    Layer<TsdfVoxel>* layer) {
  if (method.compare("simple") == 0) {
    return std::make_unique<SimpleTsdfIntegrator>(config, layer);
  } else if (method.compare("merged") == 0) {
    return std::make_unique<MergedTsdfIntegrator>(config, layer);
  } else if (method.compare("fast") == 0) {
    return std::make_unique<FastTsdfIntegrator>(config, layer);
  } else {
    return std::make_unique<SimpleTsdfIntegrator>(config, layer);
  }
}

}  // namespace

TsdfServer::TsdfServer(rclcpp::Node::SharedPtr node)
    : TsdfServer(
          node, getTsdfMapConfigFromRosParam(*node),
          getTsdfIntegratorConfigFromRosParam(*node),
          getMeshIntegratorConfigFromRosParam(*node)) {}

TsdfServer::TsdfServer(
    rclcpp::Node::SharedPtr node, const TsdfMap::Config& config,
    const TsdfIntegratorBase::Config& integrator_config,
    const MeshIntegratorConfig& mesh_config)
    : node_(node),
      verbose_(true),
      world_frame_("world"),
      icp_corrected_frame_("icp_corrected"),
      pose_corrected_frame_("pose_corrected"),
      max_block_distance_from_body_(std::numeric_limits<FloatingPoint>::max()),
      slice_level_(0.5),
      use_freespace_pointcloud_(false),
      color_map_(new RainbowColorMap()),
      publish_pointclouds_on_update_(false),
      publish_slices_(false),
      publish_pointclouds_(false),
      publish_tsdf_map_(false),
      cache_mesh_(false),
      enable_icp_(false),
      accumulate_icp_corrections_(true),
      pointcloud_queue_size_(1),
      num_subscribers_tsdf_map_(0),
      transformer_(node) {
  getServerConfigFromRosParam();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  // Advertise topics.
  const rclcpp::QoS kLatchedQos = rclcpp::QoS(1).transient_local().reliable();
  surface_pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "~/surface_pointcloud", kLatchedQos);
  tsdf_pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/tsdf_pointcloud", kLatchedQos);
  occupancy_marker_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "~/occupied_nodes", kLatchedQos);
  tsdf_slice_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/tsdf_slice", kLatchedQos);

  mesh_pub_ =
      node_->create_publisher<voxfield_msgs::msg::Mesh>("~/mesh", kLatchedQos);

  // Publishing/subscribing to a layer from another node (when using this as
  // a library, for example within a planner).
  tsdf_map_pub_ = node_->create_publisher<voxfield_msgs::msg::Layer>(
      "~/tsdf_map_out", rclcpp::QoS(1));
  tsdf_map_sub_ = node_->create_subscription<voxfield_msgs::msg::Layer>(
      "~/tsdf_map_in", rclcpp::QoS(1),
      std::bind(&TsdfServer::tsdfMapCallback, this, std::placeholders::_1));
  robot_model_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "~/Robot_model", rclcpp::QoS(100));
  param(*node_, "publish_tsdf_map", publish_tsdf_map_);

  // MULTI_SENSOR_PLAN.md M2/M4: build the per-sensor configs (legacy: one
  // "default" sensor from the top-level params below; multi-sensor: one per
  // sensor_names entry, with per-sensor overrides on top of these same
  // top-level params). Loading configs doesn't touch tsdf_map_, so this can
  // run before it exists -- and must, so the M11 ICP guard right below can
  // see the sensor count before icp_transform_pub_ would be created.
  std::string method("merged");
  param(*node_, "method", method);

  SensorConfig legacy_input;
  legacy_input.name = "default";
  legacy_input.topic = "pointcloud";
  legacy_input.freespace_topic =
      use_freespace_pointcloud_ ? "freespace_pointcloud" : "";
  legacy_input.frame = sensor_frame_;
  legacy_input.queue_size = pointcloud_queue_size_;
  param(*node_, "input_qos_best_effort", legacy_input.input_qos_best_effort);
  {
    double min_time_between_msgs_sec = 0.0;
    param(*node_, "min_time_between_msgs_sec", min_time_between_msgs_sec);
    legacy_input.min_time_between_msgs_sec = min_time_between_msgs_sec;
  }

  const std::vector<LoadedSensor> loaded_sensors = loadSensors(
      *node_, &integrator_config, nullptr, nullptr, method, legacy_input);

  // M11: ICP is not supported with multiple sensors.
  if (enable_icp_ && loaded_sensors.size() > 1) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "ICP is not supported with multiple sensors; disabling");
    enable_icp_ = false;
  }

  if (enable_icp_) {
    icp_transform_pub_ =
        node_->create_publisher<geometry_msgs::msg::TransformStamped>(
            "~/icp_transform", kLatchedQos);
    param(*node_, "icp_corrected_frame", icp_corrected_frame_);
    param(*node_, "pose_corrected_frame", pose_corrected_frame_);
  }

  // Initialize the TSDF map and its (currently sensor-less) accessories.
  tsdf_map_.reset(new TsdfMap(config));

  mesh_layer_.reset(new MeshLayer(tsdf_map_->block_size()));

  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_map_->getTsdfLayerPtr(), mesh_layer_.get()));

  icp_.reset(new ICP(getICPConfigFromRosParam(*node_)));

  // M3: one integrator instance per sensor, all on the shared layer.
  sensors_.reserve(loaded_sensors.size());
  for (const LoadedSensor& loaded : loaded_sensors) {
    auto sensor = std::make_unique<SensorInput<TsdfIntegratorBase>>();
    sensor->config = loaded.input;
    sensor->integrator = makeTsdfIntegrator(
        loaded.method, loaded.tsdf, tsdf_map_->getTsdfLayerPtr());
    sensors_.push_back(std::move(sensor));
  }

  // Subscriptions last, among these, so no callback can fire before every
  // sensor's integrator exists (M4).
  for (std::unique_ptr<SensorInput<TsdfIntegratorBase>>& owned_sensor :
       sensors_) {
    SensorInput<TsdfIntegratorBase>* sensor = owned_sensor.get();
    const rclcpp::QoS pointcloud_qos =
        sensor->config.input_qos_best_effort
            ? rclcpp::QoS(
                  rclcpp::SensorDataQoS().keep_last(
                      static_cast<size_t>(sensor->config.queue_size)))
            : rclcpp::QoS(sensor->config.queue_size);
    sensor->sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        sensor->config.topic, pointcloud_qos,
        [this, sensor](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          insertPointcloud(msg, sensor);
        });
    if (!sensor->config.freespace_topic.empty()) {
      // points that are not inside an object, but may also not be on a
      // surface. These will only be used to mark freespace beyond the
      // truncation distance.
      sensor->freespace_sub =
          node_->create_subscription<sensor_msgs::msg::PointCloud2>(
              sensor->config.freespace_topic, pointcloud_qos,
              [this, sensor](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                insertFreespacePointcloud(msg, sensor);
              });
    }
  }

  // Advertise services.
  generate_mesh_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/generate_mesh", std::bind(
                             &TsdfServer::generateMeshCallback, this,
                             std::placeholders::_1, std::placeholders::_2));
  clear_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/clear_map", std::bind(
                         &TsdfServer::clearMapCallback, this,
                         std::placeholders::_1, std::placeholders::_2));
  save_map_srv_ = node_->create_service<voxfield_msgs::srv::FilePath>(
      "~/save_map", std::bind(
                        &TsdfServer::saveMapCallback, this,
                        std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = node_->create_service<voxfield_msgs::srv::FilePath>(
      "~/load_map", std::bind(
                        &TsdfServer::loadMapCallback, this,
                        std::placeholders::_1, std::placeholders::_2));
  publish_pointclouds_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/publish_pointclouds",
      std::bind(
          &TsdfServer::publishPointcloudsCallback, this, std::placeholders::_1,
          std::placeholders::_2));
  publish_tsdf_map_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/publish_map", std::bind(
                           &TsdfServer::publishTsdfMapCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

  // If set, use a timer to progressively integrate the mesh.
  double update_mesh_every_n_sec = 1.0;
  param(*node_, "update_mesh_every_n_sec", update_mesh_every_n_sec);

  if (update_mesh_every_n_sec > 0.0) {
    update_mesh_timer_ = rclcpp::create_timer(
        node_, node_->get_clock(),
        rclcpp::Duration::from_seconds(update_mesh_every_n_sec),
        std::bind(&TsdfServer::updateMeshEvent, this));
  } else {
    update_mesh_every_n_ = static_cast<int>(-1.0 * update_mesh_every_n_sec);
  }

  double publish_map_every_n_sec = 1.0;
  param(*node_, "publish_map_every_n_sec", publish_map_every_n_sec);

  if (publish_map_every_n_sec > 0.0) {
    publish_map_timer_ = rclcpp::create_timer(
        node_, node_->get_clock(),
        rclcpp::Duration::from_seconds(publish_map_every_n_sec),
        std::bind(&TsdfServer::publishMapEvent, this));
  }
}

void TsdfServer::getServerConfigFromRosParam() {
  param(*node_, "max_block_distance_from_body", max_block_distance_from_body_);
  param(*node_, "slice_level", slice_level_);
  param(*node_, "world_frame", world_frame_);
  param(*node_, "sensor_frame", sensor_frame_);
  param(*node_, "body_frame", body_frame_);
  param(
      *node_, "publish_pointclouds_on_update", publish_pointclouds_on_update_);
  param(*node_, "publish_slices", publish_slices_);
  param(*node_, "publish_pointclouds", publish_pointclouds_);

  param(*node_, "use_freespace_pointcloud", use_freespace_pointcloud_);
  param(*node_, "pointcloud_queue_size", pointcloud_queue_size_);
  param(*node_, "enable_icp", enable_icp_);
  param(*node_, "accumulate_icp_corrections", accumulate_icp_corrections_);

  param(*node_, "verbose", verbose_);
  param(*node_, "timing", timing_);

  // Robot model related
  param(*node_, "publish_robot_model", publish_robot_model_);
  param(*node_, "robot_model_file", robot_model_file_);
  param(*node_, "robot_model_scale", robot_model_scale_);

  // Mesh settings.
  param(*node_, "mesh_filename", mesh_filename_);
  std::string color_mode("");
  param(*node_, "color_mode", color_mode);
  color_mode_ = getColorModeFromString(color_mode);

  // Color map for intensity pointclouds.
  std::string intensity_colormap("rainbow");
  float intensity_max_value = kDefaultMaxIntensity;
  param(*node_, "intensity_colormap", intensity_colormap);
  param(*node_, "intensity_max_value", intensity_max_value);

  // Default set in constructor.
  if (intensity_colormap == "rainbow") {
    color_map_.reset(new RainbowColorMap());
  } else if (intensity_colormap == "inverse_rainbow") {
    color_map_.reset(new InverseRainbowColorMap());
  } else if (intensity_colormap == "grayscale") {
    color_map_.reset(new GrayscaleColorMap());
  } else if (intensity_colormap == "inverse_grayscale") {
    color_map_.reset(new InverseGrayscaleColorMap());
  } else if (intensity_colormap == "ironbow") {
    color_map_.reset(new IronbowColorMap());
  } else {
    RCLCPP_ERROR_STREAM(
        node_->get_logger(), "Invalid color map: " << intensity_colormap);
  }
  color_map_->setMaxValue(intensity_max_value);
}

void TsdfServer::processPointCloudMessageAndInsert(
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud) {
  if (sensors_.empty()) {
    return;
  }
  processPointCloudMessageAndInsert(
      pointcloud_msg, T_G_C, is_freespace_pointcloud, sensors_[0].get());
}

void TsdfServer::processPointCloudMessageAndInsert(
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg,
    const Transformation& T_G_C, const bool is_freespace_pointcloud,
    SensorInput<TsdfIntegratorBase>* sensor) {
  // Convert the PCL pointcloud into our awesome format.

  // Horrible hack fix to fix color parsing colors in PCL.
  bool color_pointcloud = false;
  bool has_intensity = false;
  bool has_label = false;
  for (size_t d = 0; d < pointcloud_msg->fields.size(); ++d) {
    if (pointcloud_msg->fields[d].name == std::string("rgb")) {
      pointcloud_msg->fields[d].datatype =
          sensor_msgs::msg::PointField::FLOAT32;
      color_pointcloud = true;
    } else if (pointcloud_msg->fields[d].name == std::string("intensity")) {
      has_intensity = true;
    } else if (pointcloud_msg->fields[d].name == std::string("label")) {
      has_label = true;
      RCLCPP_INFO(
          node_->get_logger(),
          "Found semantic/instance label in the point cloud");
    }
  }

  Pointcloud points_C;
  Colors colors;
  Labels labels;
  timing::Timer ptcloud_timer("ptcloud_preprocess");

  // Convert differently depending on RGB or I type.
  if (has_label) {
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointcloud_pcl(
        new pcl::PointCloud<pcl::PointXYZRGBL>());
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, *pointcloud_pcl);
    convertPointcloud(
        *pointcloud_pcl, color_map_, &points_C, &colors, &labels, true);
    pointcloud_pcl.reset(new pcl::PointCloud<pcl::PointXYZRGBL>());
  } else if (color_pointcloud) {
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else if (has_intensity) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  } else {
    pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
    // pointcloud_pcl is modified below:
    pcl::fromROSMsg(*pointcloud_msg, pointcloud_pcl);
    convertPointcloud(pointcloud_pcl, color_map_, &points_C, &colors);
  }
  ptcloud_timer.Stop();

  Transformation T_G_C_refined = T_G_C;
  if (enable_icp_) {
    timing::Timer icp_timer("icp");
    if (!accumulate_icp_corrections_) {
      icp_corrected_transform_.setIdentity();
    }
    static Transformation T_offset;
    const size_t num_icp_updates = icp_->runICP(
        tsdf_map_->getTsdfLayer(), points_C, icp_corrected_transform_ * T_G_C,
        &T_G_C_refined);
    if (verbose_) {
      RCLCPP_INFO(
          node_->get_logger(),
          "ICP refinement performed %zu successful update steps",
          num_icp_updates);
    }
    icp_corrected_transform_ = T_G_C_refined * T_G_C.inverse();

    if (!icp_->refiningRollPitch()) {
      // its already removed internally but small floating point errors can
      // build up if accumulating transforms
      Transformation::Vector6 T_vec = icp_corrected_transform_.log();
      T_vec[3] = 0.0;
      T_vec[4] = 0.0;
      icp_corrected_transform_ = Transformation::exp(T_vec);
    }

    // Publish transforms as both TF and message.
    geometry_msgs::msg::TransformStamped icp_tf_msg, pose_tf_msg;

    transformKindrToMsg(
        icp_corrected_transform_.cast<double>(), &icp_tf_msg.transform);
    icp_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    icp_tf_msg.header.frame_id = world_frame_;
    icp_tf_msg.child_frame_id = icp_corrected_frame_;
    tf_broadcaster_->sendTransform(icp_tf_msg);

    transformKindrToMsg(T_G_C.cast<double>(), &pose_tf_msg.transform);
    pose_tf_msg.header.stamp = pointcloud_msg->header.stamp;
    pose_tf_msg.header.frame_id = icp_corrected_frame_;
    pose_tf_msg.child_frame_id = pose_corrected_frame_;
    tf_broadcaster_->sendTransform(pose_tf_msg);

    geometry_msgs::msg::TransformStamped transform_msg;
    transformKindrToMsg(
        icp_corrected_transform_.cast<double>(), &transform_msg.transform);
    transform_msg.header.stamp = pointcloud_msg->header.stamp;
    transform_msg.header.frame_id = world_frame_;
    transform_msg.child_frame_id = icp_corrected_frame_;
    icp_transform_pub_->publish(transform_msg);

    icp_timer.Stop();
  }

  if (verbose_) {
    RCLCPP_INFO(
        node_->get_logger(), "[%s] Integrating a pointcloud with %lu points.",
        sensor->config.name.c_str(), points_C.size());
  }

  std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  integratePointcloud(
      T_G_C_refined, points_C, colors, is_freespace_pointcloud, sensor);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  if (verbose_) {
    RCLCPP_INFO(
        node_->get_logger(),
        "Finished integrating in %f seconds, have %lu blocks.",
        std::chrono::duration<double>(end - start).count(),
        tsdf_map_->getTsdfLayer().getNumberOfAllocatedBlocks());
  }
  // mesh reconstruction with the counter interval
  if (update_mesh_every_n_ > 0 && frame_count_ != 0 &&
      frame_count_ % update_mesh_every_n_ == 0) {
    updateMesh();
  }

  // MULTI_SENSOR_PLAN.md M9: block removal / mesh clear-sphere /
  // newPoseCallback() use the body's pose instead of the sensor's own when
  // body_frame is set; "" (default) leaves T_pose == T_G_C, unchanged.
  Transformation T_pose = T_G_C;
  if (!body_frame_.empty()) {
    Transformation T_G_B;
    const Transformation identity_extrinsic;
    if (transformer_.lookupSensorTransform(
            body_frame_, &identity_extrinsic,
            rclcpp::Time(pointcloud_msg->header.stamp, RCL_ROS_TIME), &T_G_B)) {
      T_pose = T_G_B;
    } else {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 10000,
          "Failed to look up body_frame '%s'; falling back to sensor '%s's "
          "own pose for block removal / clear sphere / newPoseCallback.",
          body_frame_.c_str(), sensor->config.name.c_str());
    }
  }

  // timing::Timer block_remove_timer("remove_distant_blocks");
  tsdf_map_->getTsdfLayerPtr()->removeDistantBlocks(
      T_pose.getPosition(), max_block_distance_from_body_);
  mesh_layer_->clearDistantMesh(
      T_pose.getPosition(), max_block_distance_from_body_);
  // block_remove_timer.Stop();

  // M10: the robot model marker follows the primary sensor only (in legacy
  // mode that's every cloud, as today).
  if (!sensors_.empty() && sensor == sensors_[0].get()) {
    publishRobotMesh(T_G_C_refined);
  }

  // Callback for inheriting classes.
  newPoseCallback(T_pose);
}

void TsdfServer::publishRobotMesh(const Transformation& T_G_C) {
  // publish the robot model with the pose
  visualization_msgs::msg::Marker robot_model;
  robot_model.header.frame_id = world_frame_;
  robot_model.header.stamp = builtin_interfaces::msg::Time();
  robot_model.mesh_resource = "file://" + robot_model_file_;
  robot_model.mesh_use_embedded_materials = true;
  robot_model.scale.x = robot_model.scale.y = robot_model.scale.z =
      robot_model_scale_;
  robot_model.lifetime = builtin_interfaces::msg::Duration();
  // ROS 2's visualization_msgs::msg::Marker has no MODIFY action; ADD has
  // the same numeric value (0) and is used for both add-and-modify.
  robot_model.action = visualization_msgs::msg::Marker::ADD;
  robot_model.color.a = robot_model.color.r = robot_model.color.g =
      robot_model.color.b = 1.;
  robot_model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

  // Change to horizontal camera frame
  Transformation T_G_CH = T_G_C * transformer_.getModelTransform();
  Eigen::Quaternionf quatrot = T_G_CH.getEigenQuaternion();
  Point quat_vec = quatrot.vec();
  robot_model.pose.orientation.x = quat_vec(0);
  robot_model.pose.orientation.y = quat_vec(1);
  robot_model.pose.orientation.z = quat_vec(2);
  robot_model.pose.orientation.w = quatrot.w();
  Point translation = T_G_CH.getPosition();
  robot_model.pose.position.x = translation(0);
  robot_model.pose.position.y = translation(1);
  robot_model.pose.position.z = translation(2);
  robot_model_pub_->publish(robot_model);
}

// Checks if we can get the next message from queue.
bool TsdfServer::getNextPointcloudFromQueue(
    SensorInput<TsdfIntegratorBase>* sensor,
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr>* queue,
    sensor_msgs::msg::PointCloud2::SharedPtr* pointcloud_msg,
    Transformation* T_G_C) {
  const size_t kMaxQueueSize = 10;
  if (queue->empty()) {
    return false;
  }
  *pointcloud_msg = queue->front();
  // MULTI_SENSOR_PLAN.md F1/M5: an empty per-sensor frame falls back to the
  // message's own header frame (upstream voxblox behavior) instead of
  // failing forever.
  const std::string& from_frame = sensor->config.frame.empty()
                                      ? (*pointcloud_msg)->header.frame_id
                                      : sensor->config.frame;
  const Transformation* T_B_C_or_null =
      sensor->config.has_T_B_C ? &sensor->config.T_B_C : nullptr;
  if (transformer_.lookupSensorTransform(
          from_frame, T_B_C_or_null,
          rclcpp::Time((*pointcloud_msg)->header.stamp, RCL_ROS_TIME), T_G_C)) {
    queue->pop();
    return true;
  } else {
    if (queue->size() >= kMaxQueueSize) {
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 60000,
          "[%s] Input pointcloud queue getting too long! Dropping "
          "some pointclouds. Either unable to look up transform "
          "timestamps or the processing is taking too long.",
          sensor->config.name.c_str());
      while (queue->size() >= kMaxQueueSize) {
        queue->pop();
        ++sensor->num_dropped;
      }
    }
  }
  return false;
}

void TsdfServer::insertPointcloud(
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud) {
  if (sensors_.empty()) {
    return;
  }
  insertPointcloud(pointcloud, sensors_[0].get());
}

void TsdfServer::insertPointcloud(
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_in,
    SensorInput<TsdfIntegratorBase>* sensor) {
  ++sensor->num_received;
  const rclcpp::Time stamp(pointcloud_msg_in->header.stamp, RCL_ROS_TIME);
  const rclcpp::Duration min_time_between_msgs =
      rclcpp::Duration::from_seconds(sensor->config.min_time_between_msgs_sec);
  if (stamp - sensor->last_msg_time > min_time_between_msgs) {
    sensor->last_msg_time = stamp;
    // So we have to process the queue anyway... Push this back.
    sensor->queue.push(pointcloud_msg_in);
  } else {
    ++sensor->num_throttled;
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  bool processed_any = false;
  while (getNextPointcloudFromQueue(
      sensor, &sensor->queue, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = false;
    processPointCloudMessageAndInsert(
        pointcloud_msg, T_G_C, is_freespace_pointcloud, sensor);
    processed_any = true;
    ++sensor->num_integrated;
  }

  // MULTI_SENSOR_PLAN.md M4 / Phase 9 step 4: per-sensor counters, logged
  // (verbose only) once per callback so real-bag runs can check that every
  // sensor keeps up and nothing is dropped after TF warm-up.
  if (verbose_) {
    RCLCPP_INFO(
        node_->get_logger(),
        "[%s] stats: received=%zu throttled=%zu dropped=%zu integrated=%zu",
        sensor->config.name.c_str(), sensor->num_received,
        sensor->num_throttled, sensor->num_dropped, sensor->num_integrated);
  }

  if (!processed_any) {
    return;
  }

  if (publish_pointclouds_on_update_) {
    publishPointclouds();
  }

  if (timing_)
    RCLCPP_INFO_STREAM(
        node_->get_logger(), "Frame [" << frame_count_
                                       << "] timings: " << std::endl
                                       << timing::Timing::Print());

  if (verbose_)
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "Layer memory: " << tsdf_map_->getTsdfLayer().getMemorySize());

  frame_count_++;
}

void TsdfServer::insertFreespacePointcloud(
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud) {
  if (sensors_.empty()) {
    return;
  }
  insertFreespacePointcloud(pointcloud, sensors_[0].get());
}

void TsdfServer::insertFreespacePointcloud(
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_in,
    SensorInput<TsdfIntegratorBase>* sensor) {
  const rclcpp::Time stamp(pointcloud_msg_in->header.stamp, RCL_ROS_TIME);
  const rclcpp::Duration min_time_between_msgs =
      rclcpp::Duration::from_seconds(sensor->config.min_time_between_msgs_sec);
  if (stamp - sensor->last_freespace_msg_time > min_time_between_msgs) {
    sensor->last_freespace_msg_time = stamp;
    // So we have to process the queue anyway... Push this back.
    sensor->freespace_queue.push(pointcloud_msg_in);
  }

  Transformation T_G_C;
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
  while (getNextPointcloudFromQueue(
      sensor, &sensor->freespace_queue, &pointcloud_msg, &T_G_C)) {
    constexpr bool is_freespace_pointcloud = true;
    processPointCloudMessageAndInsert(
        pointcloud_msg, T_G_C, is_freespace_pointcloud, sensor);
  }
}

void TsdfServer::integratePointcloud(
    const Transformation& T_G_C, const Pointcloud& ptcloud_C,
    const Colors& colors, const bool is_freespace_pointcloud) {
  if (sensors_.empty()) {
    return;
  }
  integratePointcloud(
      T_G_C, ptcloud_C, colors, is_freespace_pointcloud, sensors_[0].get());
}

void TsdfServer::integratePointcloud(
    const Transformation& T_G_C, const Pointcloud& ptcloud_C,
    const Colors& colors, const bool is_freespace_pointcloud,
    SensorInput<TsdfIntegratorBase>* sensor) {
  CHECK_EQ(ptcloud_C.size(), colors.size());
  sensor->integrator->integratePointCloud(
      T_G_C, ptcloud_C, colors, is_freespace_pointcloud);
}

void TsdfServer::publishAllUpdatedTsdfVoxels() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayer(tsdf_map_->getTsdfLayer(), &pointcloud);

  publishPclCloud(tsdf_pointcloud_pub_, pointcloud, world_frame_, node_->now());
}

void TsdfServer::publishTsdfSurfacePoints() {
  // Create a pointcloud with distance = intensity.
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
  const float surface_distance_thresh =
      tsdf_map_->getTsdfLayer().voxel_size() * 0.75;
  createSurfacePointcloudFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), surface_distance_thresh, &pointcloud);

  publishPclCloud(
      surface_pointcloud_pub_, pointcloud, world_frame_, node_->now());
}

void TsdfServer::publishTsdfOccupiedNodes() {
  // Create a pointcloud with distance = intensity.
  visualization_msgs::msg::MarkerArray marker_array;
  createOccupancyBlocksFromTsdfLayer(
      tsdf_map_->getTsdfLayer(), world_frame_, &marker_array);
  occupancy_marker_pub_->publish(marker_array);
}

void TsdfServer::publishSlices() {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;

  createDistancePointcloudFromTsdfLayerSlice(
      tsdf_map_->getTsdfLayer(), 2, slice_level_, &pointcloud);

  publishPclCloud(tsdf_slice_pub_, pointcloud, world_frame_, node_->now());
}

void TsdfServer::publishMap(bool reset_remote_map) {
  if (!publish_tsdf_map_) {
    return;
  }
  size_t subscribers = tsdf_map_pub_->get_subscription_count();
  if (subscribers > 0) {
    if (num_subscribers_tsdf_map_ < static_cast<int>(subscribers)) {
      // Always reset the remote map and send all when a new subscriber
      // subscribes. A bit of overhead for other subscribers, but better than
      // inconsistent map states.
      reset_remote_map = true;
    }
    const bool only_updated = !reset_remote_map;
    timing::Timer publish_map_timer("map/publish_tsdf");
    voxfield_msgs::msg::Layer layer_msg;
    serializeLayerAsMsg<TsdfVoxel>(
        tsdf_map_->getTsdfLayer(), only_updated, &layer_msg);
    if (reset_remote_map) {
      layer_msg.action = static_cast<uint8_t>(MapDerializationAction::kReset);
    }
    tsdf_map_pub_->publish(layer_msg);
    publish_map_timer.Stop();
  }
  num_subscribers_tsdf_map_ = static_cast<int>(subscribers);
}

void TsdfServer::publishPointclouds() {
  // Combined function to publish all possible pointcloud messages -- surface
  // pointclouds, updated points, and occupied points.
  publishAllUpdatedTsdfVoxels();
  publishTsdfSurfacePoints();
  publishTsdfOccupiedNodes();
  if (publish_slices_) {
    publishSlices();
  }
}

void TsdfServer::updateMesh() {
  if (verbose_) {
    RCLCPP_INFO(node_->get_logger(), "Updating mesh.");
  }

  timing::Timer generate_mesh_timer("mesh/update");
  constexpr bool only_mesh_updated_blocks = true;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");

  voxfield_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg, node_->now());
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  if (cache_mesh_) {
    cached_mesh_msg_ = mesh_msg;
  }

  publish_mesh_timer.Stop();

  if (publish_pointclouds_ && !publish_pointclouds_on_update_) {
    publishPointclouds();
  }
}

bool TsdfServer::generateMesh() {
  timing::Timer generate_mesh_timer("mesh/generate");
  const bool clear_mesh = true;
  if (clear_mesh) {
    constexpr bool only_mesh_updated_blocks = false;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  } else {
    constexpr bool only_mesh_updated_blocks = true;
    constexpr bool clear_updated_flag = true;
    mesh_integrator_->generateMesh(
        only_mesh_updated_blocks, clear_updated_flag);
  }
  generate_mesh_timer.Stop();

  timing::Timer publish_mesh_timer("mesh/publish");
  voxfield_msgs::msg::Mesh mesh_msg;
  generateVoxbloxMeshMsg(mesh_layer_, color_mode_, &mesh_msg, node_->now());
  mesh_msg.header.frame_id = world_frame_;
  mesh_pub_->publish(mesh_msg);

  publish_mesh_timer.Stop();

  if (!mesh_filename_.empty()) {
    timing::Timer output_mesh_timer("mesh/output");
    const bool success = outputMeshLayerAsPly(mesh_filename_, *mesh_layer_);
    output_mesh_timer.Stop();
    if (success) {
      RCLCPP_INFO(
          node_->get_logger(), "Output file as PLY: %s",
          mesh_filename_.c_str());
    } else {
      RCLCPP_INFO(
          node_->get_logger(), "Failed to output mesh as PLY: %s",
          mesh_filename_.c_str());
    }
  }
  if (timing_)
    RCLCPP_INFO_STREAM(
        node_->get_logger(), "Mesh Timings: " << std::endl
                                              << timing::Timing::Print());
  return true;
}

bool TsdfServer::saveMap(const std::string& file_path) {
  // Inheriting classes should add saving other layers to this function.
  return io::SaveLayer(tsdf_map_->getTsdfLayer(), file_path);
}

bool TsdfServer::loadMap(const std::string& file_path) {
  // Inheriting classes should add other layers to load, as this will only
  // load
  // the TSDF layer.
  constexpr bool kMulitpleLayerSupport = true;
  bool success = io::LoadBlocksFromFile(
      file_path, Layer<TsdfVoxel>::BlockMergingStrategy::kReplace,
      kMulitpleLayerSupport, tsdf_map_->getTsdfLayerPtr());
  if (success) {
    LOG(INFO) << "Successfully loaded TSDF layer.";
  }
  return success;
}

void TsdfServer::clearMapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  clear();
}

void TsdfServer::generateMeshCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  if (!generateMesh()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to generate mesh.");
  }
}

void TsdfServer::saveMapCallback(
    const std::shared_ptr<voxfield_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxfield_msgs::srv::FilePath::Response> /*response*/) {
  if (!saveMap(request->file_path)) {
    RCLCPP_ERROR(
        node_->get_logger(), "Failed to save map to '%s'",
        request->file_path.c_str());
  }
}

void TsdfServer::loadMapCallback(
    const std::shared_ptr<voxfield_msgs::srv::FilePath::Request> request,
    std::shared_ptr<voxfield_msgs::srv::FilePath::Response> /*response*/) {
  if (!loadMap(request->file_path)) {
    RCLCPP_ERROR(
        node_->get_logger(), "Failed to load map from '%s'",
        request->file_path.c_str());
  }
}

void TsdfServer::publishPointcloudsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  publishPointclouds();
}

void TsdfServer::publishTsdfMapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  publishMap();
}

void TsdfServer::updateMeshEvent() {
  updateMesh();
}

void TsdfServer::publishMapEvent() {
  publishMap();
}

void TsdfServer::clear() {
  tsdf_map_->getTsdfLayerPtr()->removeAllBlocks();
  mesh_layer_->clear();

  // Publish a message to reset the map to all subscribers.
  if (publish_tsdf_map_) {
    constexpr bool kResetRemoteMap = true;
    publishMap(kResetRemoteMap);
  }
}

void TsdfServer::tsdfMapCallback(
    const voxfield_msgs::msg::Layer::SharedPtr layer_msg) {
  timing::Timer receive_map_timer("map/receive_tsdf");

  bool success = deserializeMsgToLayer<TsdfVoxel>(
      *layer_msg, tsdf_map_->getTsdfLayerPtr());

  if (!success) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 10000,
        "Got an invalid TSDF map message!");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Got an TSDF map from ROS topic!");
    if (publish_pointclouds_on_update_) {
      publishPointclouds();
    }
  }
}

}  // namespace voxfield
