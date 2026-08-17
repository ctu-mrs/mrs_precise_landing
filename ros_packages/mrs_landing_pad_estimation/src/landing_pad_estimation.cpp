/* include //{ */

#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/node.h>

#include <map>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

#include <mrs_lib/lkf.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_landing_pad_estimation
{

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

/* defines //{ */

#define STATE_X 0
#define STATE_Y 1
#define STATE_Z 2
#define STATE_HEADING 3

//}

/* LKF helpers //{ */

// Define the LKF we will be using
const int _n_states_       = 4;
const int _n_inputs_       = 0;
const int _n_measurements_ = 4;

using lkf_t = mrs_lib::LKF<_n_states_, _n_inputs_, _n_measurements_>;

using A_t        = lkf_t::A_t;
using B_t        = lkf_t::B_t;
using H_t        = lkf_t::H_t;
using Q_t        = lkf_t::Q_t;
using x_t        = lkf_t::x_t;
using P_t        = lkf_t::P_t;
using R_t        = lkf_t::R_t;
using statecov_t = lkf_t::statecov_t;

//}

// --------------------------------------------------------------
// |                          the class                         |
// --------------------------------------------------------------

/* class LandingPadEstimation() //{ */

class LandingPadEstimation : public mrs_lib::Node {

public:
  LandingPadEstimation(rclcpp::NodeOptions options);

  void iterate(const double dt);
  void publish(void);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  bool is_initialized_ = false;

  // params
  double               _prediction_rate_;
  std::string          _uav_name_;
  std::vector<int64_t> _tag_ids_;
  std::string          _estimation_frame_;
  std::string      _full_estimation_frame_;
  std::string      _body_frame_;
  std::string      _full_body_frame_;
  double           _correction_timeout_;
  double           max_relative_distance_;
  bool             _autoprefix_uav_name_;

  double relative_x, relative_y, relative_z;
  double relative_roll, relative_pitch, relative_yaw;

  mrs_lib::Transformer transformer_;

  mrs_lib::SubscriberHandler<apriltag_msgs::msg::AprilTagDetectionArray> sh_tag_detections_;

  mrs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped> ph_pose_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped> ph_measurement_;

  void callbackTagDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msg);

  // lkf matrices
  A_t A_;
  R_t R_;
  Q_t Q_;
  H_t H_;
  B_t B_;

  std::unique_ptr<lkf_t> lkf_;

  std::optional<statecov_t> statecov_;
  rclcpp::Time              time_last_correction_;
  std::mutex                mutex_statecov_;

  std::shared_ptr<TimerType> timer_main_;
  rclcpp::Time last_time_;
  void timerMain();
};

//}

/* LandingPadEstimation() //{ */

LandingPadEstimation::LandingPadEstimation(rclcpp::NodeOptions options) : mrs_lib::Node("LandingPadEstimation", options) {

  node_ = this_node_ptr();
  clock_ = node_->get_clock();

  // | ----------------------- load params ---------------------- |

  mrs_lib::ParamLoader param_loader(node_);
  param_loader.addYamlFileFromParam("config");

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "")
    param_loader.addYamlFile(custom_config_path);

  param_loader.loadParam("prediction_rate", _prediction_rate_);
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("tag_ids", _tag_ids_);
  param_loader.loadParam("estimation_frame", _estimation_frame_);
  param_loader.loadParam("body_frame", _body_frame_);
  param_loader.loadParam("correction_timeout", _correction_timeout_);
  param_loader.loadParam("max_relative_distance", max_relative_distance_);
  param_loader.loadParam("transformer/autoprefix_uav_name", _autoprefix_uav_name_);

  param_loader.loadParam("relative_transform/translation/x", relative_x);
  param_loader.loadParam("relative_transform/translation/y", relative_y);
  param_loader.loadParam("relative_transform/translation/z", relative_z);

  param_loader.loadParam("relative_transform/rotation/roll", relative_roll);
  param_loader.loadParam("relative_transform/rotation/pitch", relative_pitch);
  param_loader.loadParam("relative_transform/rotation/yaw", relative_yaw);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  _full_estimation_frame_ = _uav_name_ + "/" + _estimation_frame_;
  _full_body_frame_       = _uav_name_ + "/" + _body_frame_;

  // state matrix
  param_loader.loadMatrixStatic("lkf/A", A_);

  // input matrix (n_inputs = 0)
  // param_loader.loadMatrixStatic("lkf/B", B_);

  // measurement noise
  param_loader.loadMatrixStatic("lkf/R", R_);

  // process covariance
  param_loader.loadMatrixStatic("lkf/Q", Q_);

  // measurement mapping
  param_loader.loadMatrixStatic("lkf/H", H_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = node_;
  //shopts.node_name          = "LandingPadEstimation";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  //shopts.queue_size         = 10;
  //shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_tag_detections_ =
      mrs_lib::SubscriberHandler<apriltag_msgs::msg::AprilTagDetectionArray>(shopts, "tag_detections_in", &LandingPadEstimation::callbackTagDetections, this);

  // | ----------------------- publishers ----------------------- |

  ph_pose_        = mrs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(node_, "estimated_pose_out");
  ph_measurement_ = mrs_lib::PublisherHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(node_, "measurement_pose_out");

  // | ------------------------- timers ------------------------- |

  {
    mrs_lib::TimerHandlerOptions timer_opts_start;

    timer_opts_start.node      = node_;
    timer_opts_start.autostart = true;

    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_prediction_rate_, clock_), std::bind(&LandingPadEstimation::timerMain, this));
  }

  // | ----------------------- transfomer ----------------------- |

  transformer_ = mrs_lib::Transformer(node_);

  if (_autoprefix_uav_name_) {
    transformer_.setDefaultPrefix(_uav_name_);
  }

  transformer_.retryLookupNewest(true);

  // initialize time sentinels with the correct clock type
  last_time_            = rclcpp::Time(0, 0, clock_->get_clock_type());
  time_last_correction_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | --------------------------- lkf -------------------------- |

  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackTagDetections() //{ */

void LandingPadEstimation::callbackTagDetections(const apriltag_msgs::msg::AprilTagDetectionArray::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "receiving detections");

  // | ----------------- retrive the tag's pose ----------------- |

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> tag_pose;

  std::map<int64_t, apriltag_msgs::msg::AprilTagDetection> detection_map;

  for (const auto& tag : msg->detections) {
    detection_map.insert({tag.id, tag});
  }

  for (const auto desired_id : _tag_ids_) {
    const auto it = detection_map.find(desired_id);
    if (it != detection_map.end()) {
      const std::string tag_frame = it->second.family + ":" + std::to_string(it->second.id);

      auto tf_opt = transformer_.getTransform(tag_frame, msg->header.frame_id, msg->header.stamp);
      if (!tf_opt) {
        tf_opt = transformer_.getTransform(tag_frame, msg->header.frame_id, rclcpp::Time(0));
      }

      if (tf_opt) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header                = tf_opt->header;
        pose_msg.pose.pose.position.x  = tf_opt->transform.translation.x;
        pose_msg.pose.pose.position.y  = tf_opt->transform.translation.y;
        pose_msg.pose.pose.position.z  = tf_opt->transform.translation.z;
        pose_msg.pose.pose.orientation = tf_opt->transform.rotation;
        tag_pose                       = pose_msg;
      }
      break;
    }
  }

  if (!tag_pose) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *clock_, 1000, "tags with the right ids not found");
    return;
  }

  // | ------------------- check for outliers ------------------- |

  {
    auto result = transformer_.transformSingle(tag_pose.value(), _full_body_frame_);

    if (!result) {
      RCLCPP_ERROR(node_->get_logger(), "could not transform the tag detection to '%s'", _full_body_frame_.c_str());
      return;
    }

    if (std::hypot(result->pose.pose.position.x, result->pose.pose.position.y, result->pose.pose.position.z) > max_relative_distance_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "detection too far from the UAV");
      return;
    }
  }

  // | ------------------ publish for debugging ----------------- |

  ph_measurement_.publish(tag_pose.value());

  // | ------------------------- offset ------------------------- |

  {
    // Convert current pose to Eigen
    Eigen::Isometry3d tag_eig;
    tf2::fromMsg(tag_pose.value().pose.pose, tag_eig);

    // Create offset transform directly in Eigen
    const Eigen::Isometry3d offset_eig = Eigen::Translation3d(relative_x, relative_y, relative_z) * Eigen::Quaterniond(mrs_lib::AttitudeConverter(relative_roll, relative_pitch, relative_yaw));

    // Multiply transforms and convert back to geometry_msgs::msg::Pose directly
    tag_pose.value().pose.pose = tf2::toMsg(tag_eig * offset_eig);
  }

  // | ------------------- transform the pose ------------------- |

  auto result = transformer_.transformSingle(tag_pose.value(), _full_estimation_frame_);

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "could not transform the tag detection to '%s'", _full_estimation_frame_.c_str());
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped tag_world_ = result.value();

  RCLCPP_INFO_ONCE(node_->get_logger(), "receiving the right AprilTag");

  // | -------------------------- fuse -------------------------- |

  auto statecov = mrs_lib::get_mutexed(mutex_statecov_, statecov_);

  if (!statecov) {

    statecov = statecov_t();

    statecov->x << tag_world_.pose.pose.position.x, tag_world_.pose.pose.position.y, tag_world_.pose.pose.position.z,
        mrs_lib::AttitudeConverter(tag_world_.pose.pose.orientation).getHeading();

    statecov->P = P_t::Identity();

    RCLCPP_INFO(node_->get_logger(), "statecov initialized");
  }

  // create the measurement vector
  Eigen::VectorXd measurement = Eigen::VectorXd::Zero(_n_measurements_);

  double mes_heading = sradians::unwrap(mrs_lib::AttitudeConverter(tag_world_.pose.pose.orientation).getHeading(), statecov->x[STATE_HEADING]);

  measurement << tag_world_.pose.pose.position.x, tag_world_.pose.pose.position.y, tag_world_.pose.pose.position.z, mes_heading;

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "measurement: " << measurement.transpose());

  try {
    statecov = lkf_->correct(*statecov, measurement, R_);

    statecov->stamp = tag_world_.header.stamp;
  }
  catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "correction step failed");
    return;
  }

  rclcpp::Time time_last_correction = clock_->now();

  RCLCPP_DEBUG(node_->get_logger(), "correct: x=%.2f, y=%.2f, z=%.2f, hdg=%.2f", statecov->x[0], statecov->x[1], statecov->x[2], statecov->x[3]);

  {
    std::scoped_lock lock(mutex_statecov_);

    statecov_             = statecov;
    time_last_correction_ = time_last_correction;
  }
}

//}

// --------------------------------------------------------------
// |                           models                           |
// --------------------------------------------------------------

/* publish() //{ */

void LandingPadEstimation::publish() {

  auto [statecov, time_last_correction] = mrs_lib::get_mutexed(mutex_statecov_, statecov_, time_last_correction_);

  if (!statecov) {
    return;
  }

  if (time_last_correction == rclcpp::Time(0, 0, clock_->get_clock_type()) || (clock_->now() - time_last_correction).seconds() > _correction_timeout_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "landing pad detections timeouted");

    time_last_correction_ = rclcpp::Time(0, 0, clock_->get_clock_type());
    mrs_lib::set_mutexed(mutex_statecov_, {}, statecov_);

    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  pose.header.frame_id = _uav_name_ + "/" + _estimation_frame_;
  pose.header.stamp    = statecov->stamp;

  pose.pose.pose.position.x  = statecov->x[STATE_X];
  pose.pose.pose.position.y  = statecov->x[STATE_Y];
  pose.pose.pose.position.z  = statecov->x[STATE_Z];
  pose.pose.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(statecov->x[STATE_HEADING]);

  ph_pose_.publish(pose);
}

//}

/* iterate() //{ */

void LandingPadEstimation::iterate(const double dt) {

  auto statecov = mrs_lib::get_mutexed(mutex_statecov_, statecov_);

  if (!statecov) {
    return;
  }

  if (dt < 0.001 || dt > 1.0) {
    return;
  }

  try {
    statecov = lkf_->predict(*statecov, Eigen::VectorXd::Zero(_n_inputs_), Q_, dt);

    statecov->stamp = clock_->now();
  }
  catch (...) {
    RCLCPP_ERROR(node_->get_logger(), "prediction step failed");
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "predict: x=%.2f, y=%.2f, z=%.2f, hdg=%.2f", statecov->x[0], statecov->x[1], statecov->x[2], statecov->x[3]);

  mrs_lib::set_mutexed(mutex_statecov_, statecov, statecov_);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void LandingPadEstimation::timerMain() {

  if (!is_initialized_) {
    return;
  }

  if (!statecov_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerMain() spinning");
  const rclcpp::Time current_time = clock_->now();

  if (last_time_.nanoseconds() == 0) {
    last_time_ = current_time;
    return;
  }

  last_time_ = current_time;
  iterate((current_time - last_time_).seconds());

  publish();
}

//}

}  // namespace mrs_landing_pad_estimation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_landing_pad_estimation::LandingPadEstimation)
