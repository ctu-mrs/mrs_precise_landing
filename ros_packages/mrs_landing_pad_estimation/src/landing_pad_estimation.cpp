/* include //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <map>
#include <Eigen/Eigen>

#include <mrs_lib/lkf.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

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

class LandingPadEstimation : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool         is_initialized_ = false;

  void iterate(const double dt);
  void publish(void);

private:
  ros::NodeHandle nh_;

  // params
  double           _prediction_rate_;
  std::string      _uav_name_;
  std::vector<int> _tag_ids_;
  std::string      _estimation_frame_;
  std::string      _full_estimation_frame_;
  std::string      _body_frame_;
  std::string      _full_body_frame_;
  double           _correction_timeout_;
  double           max_relative_distance_;
  bool             _autoprefix_uav_name_;

  mrs_lib::Transformer transformer_;

  mrs_lib::SubscribeHandler<apriltag_ros::AprilTagDetectionArray> sh_tag_detections_;

  mrs_lib::PublisherHandler<geometry_msgs::PoseWithCovarianceStamped> ph_pose_;
  mrs_lib::PublisherHandler<geometry_msgs::PoseWithCovarianceStamped> ph_measurement_;

  void callbackTagDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr msg);

  // lkf matrices
  A_t A_;
  R_t R_;
  Q_t Q_;
  H_t H_;
  B_t B_;

  std::unique_ptr<lkf_t> lkf_;

  std::optional<statecov_t> statecov_;
  ros::Time                 time_last_correction_;
  std::mutex                mutex_statecov_;

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
};

//}

/* onInit() //{ */

void LandingPadEstimation::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  // | ----------------------- load params ---------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "LandingPadEstimation");

  param_loader.loadParam("prediction_rate", _prediction_rate_);
  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("tag_ids", _tag_ids_);
  param_loader.loadParam("estimation_frame", _estimation_frame_);
  param_loader.loadParam("body_frame", _body_frame_);
  param_loader.loadParam("correction_timeout", _correction_timeout_);
  param_loader.loadParam("max_relative_distance", max_relative_distance_);
  param_loader.loadParam("transformer/autoprefix_uav_name", _autoprefix_uav_name_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[LandingPadEstimation]: Could not load all parameters!");
    ros::shutdown();
  }

  _full_estimation_frame_ = _uav_name_ + "/" + _estimation_frame_;
  _full_body_frame_       = _uav_name_ + "/" + _body_frame_;

  // state matrix
  param_loader.loadMatrixStatic("lkf/A", A_);

  // input matrix
  param_loader.loadMatrixStatic("lkf/B", B_);

  // measurement noise
  param_loader.loadMatrixStatic("lkf/R", R_);

  // process covariance
  param_loader.loadMatrixStatic("lkf/Q", Q_);

  // measurement mapping
  param_loader.loadMatrixStatic("lkf/H", H_);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "LandingPadEstimation";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_tag_detections_ =
      mrs_lib::SubscribeHandler<apriltag_ros::AprilTagDetectionArray>(shopts, "tag_detections_in", &LandingPadEstimation::callbackTagDetections, this);

  // | ----------------------- publishers ----------------------- |

  ph_pose_        = mrs_lib::PublisherHandler<geometry_msgs::PoseWithCovarianceStamped>(nh_, "estimated_pose_out", 10);
  ph_measurement_ = mrs_lib::PublisherHandler<geometry_msgs::PoseWithCovarianceStamped>(nh_, "measurement_pose_out", 10);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_prediction_rate_), &LandingPadEstimation::timerMain, this);

  // | ----------------------- transfomer ----------------------- |

  transformer_ = mrs_lib::Transformer("LandingPadEstimation");

  if (_autoprefix_uav_name_) {
    transformer_.setDefaultPrefix(_uav_name_);
  }

  transformer_.retryLookupNewest(true);

  // | --------------------------- lkf -------------------------- |

  lkf_ = std::make_unique<lkf_t>(A_, B_, H_);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[LandingPadEstimation]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackTagDetections() //{ */

void LandingPadEstimation::callbackTagDetections(const apriltag_ros::AprilTagDetectionArray::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[LandingPadEstimation]: receiving detections");

  // | ----------------- retrive the tag's pose ----------------- |

  std::optional<geometry_msgs::PoseWithCovarianceStamped> tag_pose;

  std::map<int, apriltag_ros::AprilTagDetection> detection_map;

  for (auto tag : msg->detections) {
    detection_map.insert(std::pair(tag.id[0], tag));
  }

  for (auto desired_id : _tag_ids_) {
    if (detection_map.find(desired_id) != detection_map.end()) {
      tag_pose = detection_map.at(desired_id).pose;
      break;
    }
  }

  if (!tag_pose) {
    ROS_DEBUG_THROTTLE(1.0, "[LandingPadEstimation]: tags with the right ids not found");
    return;
  }

  // | ------------------- check for outliers ------------------- |

  {
    auto result = transformer_.transformSingle(tag_pose.value(), _full_body_frame_);

    if (!result) {
      ROS_ERROR("[LandingPadEstimation]: could not transform the tag detection to '%s'", _full_body_frame_.c_str());
      return;
    }

    if (std::hypot(tag_pose->pose.pose.position.x, tag_pose->pose.pose.position.y, tag_pose->pose.pose.position.z) > max_relative_distance_) {
      ROS_WARN_THROTTLE(1.0, "[LandingPadEstimation]: detection too far from the UAV");
      return;
    }
  }

  // | ------------------- transform the pose ------------------- |

  auto result = transformer_.transformSingle(tag_pose.value(), _full_estimation_frame_);

  if (!result) {
    ROS_ERROR("[LandingPadEstimation]: could not transform the tag detection to '%s'", _full_estimation_frame_.c_str());
    return;
  }

  geometry_msgs::PoseWithCovarianceStamped tag_world_ = result.value();

  ROS_INFO_ONCE("[LandingPadEstimation]: receiving the right AprilTag");

  // | ------------------ publish for debugging ----------------- |

  ph_measurement_.publish(tag_world_);

  // | -------------------------- fuse -------------------------- |

  auto statecov = mrs_lib::get_mutexed(mutex_statecov_, statecov_);

  if (!statecov) {

    statecov = statecov_t();

    statecov->x << tag_world_.pose.pose.position.x, tag_world_.pose.pose.position.y, tag_world_.pose.pose.position.z,
        mrs_lib::AttitudeConverter(tag_world_.pose.pose.orientation).getHeading();

    statecov->P = P_t::Identity();

    ROS_INFO("[LandingPadEstimation]: statecov initialized");
  }

  // create the measurement vector
  Eigen::VectorXd measurement = Eigen::VectorXd::Zero(_n_measurements_);

  double mes_heading = sradians::unwrap(mrs_lib::AttitudeConverter(tag_world_.pose.pose.orientation).getHeading(), statecov->x[STATE_HEADING]);

  measurement << tag_world_.pose.pose.position.x, tag_world_.pose.pose.position.y, tag_world_.pose.pose.position.z, mes_heading;

  ROS_DEBUG_STREAM("[LandingPadEstimation]: measurement: " << measurement.transpose());

  try {
    statecov = lkf_->correct(*statecov, measurement, R_);

    statecov->stamp = tag_world_.header.stamp;
  }
  catch (...) {
    ROS_ERROR("[LandingPadEstimation]: correction step failed");
    return;
  }

  ros::Time time_last_correction = ros::Time::now();

  ROS_DEBUG("[LandingPadEstimation]: correct: x=%.2f, y=%.2f, z=%.2f, hdg=%.2f", statecov->x[0], statecov->x[1], statecov->x[2], statecov->x[3]);

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

  if (time_last_correction == ros::Time::UNINITIALIZED || (ros::Time::now() - time_last_correction).toSec() > _correction_timeout_) {

    ROS_WARN_THROTTLE(1.0, "[LandingPadEstimation]: landing pad detections timeouted");

    time_last_correction_ = ros::Time::UNINITIALIZED;
    mrs_lib::set_mutexed(mutex_statecov_, {}, statecov_);

    return;
  }

  geometry_msgs::PoseWithCovarianceStamped pose;

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

    statecov->stamp = ros::Time::now();
  }
  catch (...) {
    ROS_ERROR("[LandingPadEstimation]: prediction step failed");
    return;
  }

  ROS_DEBUG("[LandingPadEstimation]: predict: x=%.2f, y=%.2f, z=%.2f, hdg=%.2f", statecov->x[0], statecov->x[1], statecov->x[2], statecov->x[3]);

  mrs_lib::set_mutexed(mutex_statecov_, statecov, statecov_);
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void LandingPadEstimation::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  if (!statecov_) {
  }

  ROS_INFO_ONCE("[LandingPadEstimation]: timerMain() spinning");

  iterate((event.current_real - event.last_real).toSec());

  publish();
}

//}

}  // namespace mrs_landing_pad_estimation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_landing_pad_estimation::LandingPadEstimation, nodelet::Nodelet)
