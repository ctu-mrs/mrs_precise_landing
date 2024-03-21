/* include //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/median_filter.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/String.h>
#include <std_msgs/Float64.h>

//}

using namespace Eigen;

namespace mrs_precise_landing
{

/* using //{ */

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

//}

/* defines //{ */

#define POS_X 0
#define POS_Y 1
#define POS_Z 2
#define ANGLE 3

#define ALIGNMENT_CRITERION_CONTROL_ERROR 0
#define ALIGNMENT_CRITERION_PAD_DETECTION 1

//}

/* STRUCTURES and TYPEDEFS //{ */

typedef enum
{

  MODE_2D = 1,
  MODE_3D = 2,

} Alignment_t;

// state machine
[[maybe_unused]] enum {

  IDLE_STATE,
  ALIGN_STATE,
  DESCEND_STATE,
  ALIGN2_STATE,
  LANDING_STATE,
  REPEAT_STATE,
  ASCEND_STATE,
  ABORT_STATE,

} States_t;

const char *state_names[8] = {
    "IDLING", "ALIGNING", "DESCENDING", "ALIGNING2", "LANDING", "REPEATING", "ASCENDING", "ABORTING",
};

// trajectory types
[[maybe_unused]] enum {

  ALIGN_TRAJECTORY,
  DESCEND_TRAJECTORY,
  ASCEND_TRAJECTORY,
  LANDING_TRAJECTORY,
  REPEAT_TRAJECTORY,
  ABORT_TRAJECTORY,

} Trajectory_t;

//}

// --------------------------------------------------------------
// |                          the class                         |
// --------------------------------------------------------------

/* class PreciseLanding //{ */

class PreciseLanding : public nodelet::Nodelet {

public:
  virtual void onInit();
  bool         is_initialized_ = false;

private:
  ros::NodeHandle nh_;

  std::string _uav_name_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  mrs_lib::PublisherHandler<mrs_msgs::TrajectoryReference> ph_trajectory_reference_;

  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>                 sh_tracker_cmd_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped> sh_landing_tag_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavState>                       sh_uav_state_;
  mrs_lib::SubscribeHandler<std_msgs::Float64>                        sh_mass_nominal_;
  mrs_lib::SubscribeHandler<std_msgs::Float64>                        sh_mass_estimate_;

  void callbackTimeoutTag(const std::string &topic_name, const ros::Time &last_msg);

  void callbackLandingTag(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);

  ros::ServiceServer service_server_land_;
  ros::ServiceServer service_servcer_stop_;

  mrs_lib::ServiceClientHandler<mrs_msgs::String>  sch_switch_controller_;
  mrs_lib::ServiceClientHandler<std_srvs::SetBool> sch_arming_;

  // params loaded from config file
  double _trajectory_dt_;

  std::atomic<bool> see_landing_pad_ = false;

  ros::Time timeouter_;

  // aligning params
  double _aligning_speed_;
  double _aligning_height_;
  double _aligning_radius_;
  double _aligning_timeout_;

  // descending params
  double _descending_speed_;
  double _descending_timeout_;
  double _descending_height_;

  // aligning2 params
  double _aligning2_timeout_;

  double _aligning2_criterion_initial_x_;
  double _aligning2_criterion_initial_y_;
  double _aligning2_criterion_increase_rate_x_;
  double _aligning2_criterion_increase_rate_y_;

  double _aligning2_in_alignment_duration_;
  int    _aligning2_alignment_criterion_;

  ros::Time aligning2_in_radius_time_;
  bool      aligning2_in_radius_ = false;
  double    aligning2_current_x_crit_;
  double    aligning2_current_y_crit_;

  // landing params
  ros::Time landing_since_;

  double _landing_speed_;
  double _landing_height_;
  int    _landing_repeat_threshold_;

  // repeating params
  double _repeating_speed_;
  double _repeating_height_;

  // ascending params
  double     _ascending_speed_;
  double     ascending_height_;
  std::mutex mutex_ascending_height_;

  // aborting params
  double aborting_height_;

  int    _loosing_alignment_threshold_;
  double _object_visibility_timeout_;

  bool callbackLand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void setController(std::string desired_controller);

  void disarm(void);

  bool alignedWithTarget(const double position_thr, const double heading_thr, Alignment_t mode);
  bool lastAlignmentCheck(void);

  bool shouldTimeout(const double &timeout);

  // state machine
  int current_state_, previous_state_;
  int lost_alignment_counter, repeat_landing_counter;

  ros::Timer state_machine_timer_;
  void       stateMachineTimer(const ros::TimerEvent &event);

public:
  double _main_rate_;

  void changeState(int newState);

  std::optional<mrs_msgs::TrajectoryReference> createTrajectory(int trajectoryType);
};

//}

/* onInit() //{ */

void PreciseLanding::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  mrs_lib::ParamLoader param_loader(nh_, "PreciseLanding");

  ros::Time::waitForValid();

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("rate", _main_rate_);

  param_loader.loadParam("trajectory_dt", _trajectory_dt_);

  // aligning params
  param_loader.loadParam("stages/aligning/speed", _aligning_speed_);
  param_loader.loadParam("stages/aligning/height", _aligning_height_);
  param_loader.loadParam("stages/aligning/radius", _aligning_radius_);
  param_loader.loadParam("stages/aligning/timeout", _aligning_timeout_);

  // descending params
  param_loader.loadParam("stages/descending/speed", _descending_speed_);
  param_loader.loadParam("stages/descending/timeout", _descending_timeout_);
  param_loader.loadParam("stages/descending/height", _descending_height_);

  // aligning2 params
  param_loader.loadParam("stages/aligning2/timeout", _aligning2_timeout_);

  param_loader.loadParam("stages/aligning2/criterion/initial_x", _aligning2_criterion_initial_x_);
  param_loader.loadParam("stages/aligning2/criterion/initial_y", _aligning2_criterion_initial_y_);
  param_loader.loadParam("stages/aligning2/criterion/x_increase_rate", _aligning2_criterion_increase_rate_x_);
  param_loader.loadParam("stages/aligning2/criterion/y_increase_rate", _aligning2_criterion_increase_rate_y_);

  param_loader.loadParam("stages/aligning2/alignment_criterion", _aligning2_alignment_criterion_);
  param_loader.loadParam("stages/aligning2/in_alignment_duration", _aligning2_in_alignment_duration_);

  if (!(_aligning2_alignment_criterion_ == ALIGNMENT_CRITERION_CONTROL_ERROR || _aligning2_alignment_criterion_ == ALIGNMENT_CRITERION_PAD_DETECTION)) {

    ROS_ERROR("[PreciseLanding]: the chosen alignment criterion not valid!");
    ros::shutdown();

  } else {

    std::string criterion_name;

    switch (_aligning2_alignment_criterion_) {
      case ALIGNMENT_CRITERION_CONTROL_ERROR: {
        criterion_name = "control error";
        break;
      }
      case ALIGNMENT_CRITERION_PAD_DETECTION: {
        criterion_name = "pad detection";
        break;
      }
    }

    ROS_INFO("[PreciseLanding]: alignment criterion: %s", criterion_name.c_str());
  }

  // landing params
  param_loader.loadParam("stages/landing/speed", _landing_speed_);
  param_loader.loadParam("stages/landing/height", _landing_height_);
  param_loader.loadParam("stages/landing/repeat_threshold", _landing_repeat_threshold_);

  // repeating params
  param_loader.loadParam("stages/repeating/speed", _repeating_speed_);
  param_loader.loadParam("stages/repeating/height", _repeating_height_);

  // ascending params
  param_loader.loadParam("stages/ascending/speed", _ascending_speed_);
  param_loader.loadParam("stages/ascending/height", ascending_height_);

  // aborting params
  param_loader.loadParam("stages/aborting/height", aborting_height_);

  param_loader.loadParam("loosing_alignment_threshold", _loosing_alignment_threshold_);
  param_loader.loadParam("object_visibility_timeout", _object_visibility_timeout_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[PreciseLanding]: [ControlManager]: Could not load all parameters!");
    ros::shutdown();
  }

  transformer_ = std::make_unique<mrs_lib::Transformer>("PreciseLanding");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- publishers ----------------------- |

  ph_trajectory_reference_ = mrs_lib::PublisherHandler<mrs_msgs::TrajectoryReference>(nh_, "trajectory_reference_out", 10);

  // | --------------------- service clients -------------------- |

  sch_switch_controller_ = mrs_lib::ServiceClientHandler<mrs_msgs::String>(nh_, "switch_controller_out");
  sch_arming_            = mrs_lib::ServiceClientHandler<std_srvs::SetBool>(nh_, "arming_out");

  // | --------------------- sercice servers -------------------- |

  service_server_land_  = nh_.advertiseService("land_in", &PreciseLanding::callbackLand, this);
  service_servcer_stop_ = nh_.advertiseService("stop_in", &PreciseLanding::callbackStop, this);

  // | ----------------------- subscribers ---------------------- |

  {
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "PreciseLanding";
    shopts.no_message_timeout = ros::Duration(1.0);
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_landing_tag_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped>(shopts, "landing_pad_in", &PreciseLanding::callbackLandingTag, this,
                                                                                          &PreciseLanding::callbackTimeoutTag, this);
  }

  {
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "PreciseLanding";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_tracker_cmd_   = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in");
    sh_uav_state_     = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in");
    sh_mass_nominal_  = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "mass_nominal_in");
    sh_mass_estimate_ = mrs_lib::SubscribeHandler<std_msgs::Float64>(shopts, "mass_estimate_in");
  }

  // state machine
  current_state_  = IDLE_STATE;
  previous_state_ = IDLE_STATE;

  lost_alignment_counter = 0;
  repeat_landing_counter = 0;

  // | ------------------------- timers ------------------------- |

  // start timers
  state_machine_timer_ = nh_.createTimer(ros::Rate(_main_rate_), &PreciseLanding::stateMachineTimer, this);

  is_initialized_ = true;

  ROS_INFO("[PreciseLanding]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackLandingTag() //{ */

void PreciseLanding::callbackLandingTag(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {

  ROS_INFO_ONCE("[PreciseLanding]: getting landing pad pose");

  see_landing_pad_ = true;
}

//}

/* callbackTimeoutTag() //{ */

void PreciseLanding::callbackTimeoutTag([[maybe_unused]] const std::string &topic_name, [[maybe_unused]] const ros::Time &last_msg) {

  ROS_WARN_THROTTLE(1.0, "[PreciseLanding]: the landing pad timeout");

  see_landing_pad_ = false;
}

//}

// | ------------------------ services ------------------------ |

/* callbackStop() //{ */

bool PreciseLanding::callbackStop([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  res.success = true;
  res.message = "aborting landing";

  changeState(ABORT_STATE);

  return true;
}

//}

/* callbackStart() //{ */

bool PreciseLanding::callbackLand(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  // TODO check preconditions

  changeState(ALIGN_STATE);

  res.success = true;
  res.message = "landing has started";

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void PreciseLanding::changeState(int newState) {

  // just for ROS_INFO
  ROS_INFO("[PreciseLanding]: Switching states: %s -> %s", state_names[current_state_], state_names[newState]);

  previous_state_ = current_state_;
  current_state_  = newState;

  timeouter_ = ros::Time::now();

  // if changing to idle, stop the drone
  switch (newState) {

      /* IDLE_STATE //{ */

    case IDLE_STATE:

      lost_alignment_counter = 0;
      repeat_landing_counter = 0;

      break;

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE:

      setController("Se3Controller");  // TODO

      break;

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE:

      break;

      //}

      /* ALIGN2_STATE //{ */

    case ALIGN2_STATE:

      aligning2_in_radius_time_ = ros::Time(0);
      aligning2_in_radius_      = false;
      aligning2_current_x_crit_ = _aligning2_criterion_initial_x_;
      aligning2_current_y_crit_ = _aligning2_criterion_initial_y_;

      break;

      //}

      /* LANDING_STATE //{ */

    case LANDING_STATE:

      // log when we started the landing
      landing_since_ = ros::Time::now();

      break;

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE:

      if (repeat_landing_counter++ >= _landing_repeat_threshold_) {

        ROS_INFO("[PreciseLanding]: Exceeded the number of landing attempts, going up");

        changeState(ABORT_STATE);
      }

      break;

      //}

      /* ABORT_STATE //{ */

    case ABORT_STATE:

      break;

      //}

      /* ASCEND_STATE //{ */

    case ASCEND_STATE:

      break;

      //}
  }
}

//}

/* createTrajectory() //{ */

std::optional<mrs_msgs::TrajectoryReference> PreciseLanding::createTrajectory(int trajectoryType) {

  auto tracker_cmd = sh_tracker_cmd_.getMsg();
  auto landing_tag = sh_landing_tag_.getMsg();

  geometry_msgs::PoseStamped init_cond;

  {
    init_cond.header           = tracker_cmd->header;
    init_cond.pose.position    = tracker_cmd->position;
    init_cond.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0).setHeading(tracker_cmd->heading);

    auto result = transformer_->transformSingle(init_cond, landing_tag->header.frame_id);

    if (result) {
      init_cond = result.value();
    } else {
      ROS_ERROR("[PreciseLanding]: could not transform initial condition to '%s'", landing_tag->header.frame_id.c_str());
    }
  }

  double init_x   = init_cond.pose.position.x;
  double init_y   = init_cond.pose.position.y;
  double init_z   = init_cond.pose.position.z;
  double init_hdg = mrs_lib::AttitudeConverter(init_cond.pose.orientation).getHeading();

  double landing_pad_x   = landing_tag->pose.pose.position.x;
  double landing_pad_y   = landing_tag->pose.pose.position.y;
  double landing_pad_z   = landing_tag->pose.pose.position.z;
  double landing_pad_hdg = mrs_lib::AttitudeConverter(landing_tag->pose.pose.orientation).getHeading();

  // prepare the trajectorie
  // pose array for debugging
  mrs_msgs::TrajectoryReference trajectory;
  tf::Quaternion                orientation;
  trajectory.fly_now     = true;
  trajectory.use_heading = true;
  trajectory.header      = init_cond.header;

  /* ALIGN_TRAJECTORY //{ */

  if (trajectoryType == ALIGN_TRAJECTORY) {

    double target_heading, target_distance, desired_heading;

    target_heading  = atan2(landing_pad_y - init_y, landing_pad_x - init_x);
    target_distance = mrs_lib::geometry::dist(vec2_t(init_x, init_y), vec2_t(landing_pad_x, landing_pad_y));

    desired_heading = landing_pad_hdg;  // TODO

    /* double desired_height = _aligning_height_ > cmd_odom_stable.pose.position.z ? cmd_odom_stable.pose.position.z : _aligning_height_; */
    double desired_z = landing_pad_z + _aligning_height_;

    if (desired_z > init_z) {
      desired_z = init_z;
    }

    double step_size = _aligning_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = init_x;
      point.position.y = init_y;
      point.position.z = init_z;
      point.heading    = init_hdg;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    for (int i = 0; i < n_steps; i++) {

      mrs_msgs::Reference point;

      point.position.x = trajectory.points.back().position.x + cos(target_heading) * step_size;
      point.position.y = trajectory.points.back().position.y + sin(target_heading) * step_size;
      point.position.z = desired_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // the last point = the landing pad
    {
      mrs_msgs::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = desired_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // return the trajectory
    return trajectory;

    //}

    /* DESCEND_TRAJECTORY //{ */

  } else if (trajectoryType == DESCEND_TRAJECTORY) {

    double target_distance, direction, desired_height, desired_vector, desired_heading;

    desired_height  = _descending_height_;
    desired_vector  = desired_height - init_z;
    target_distance = fabs(desired_vector);
    direction       = (desired_vector <= 0) ? -1 : 1;

    desired_heading = landing_pad_hdg;  // TODO

    double step_size = _descending_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;

      point.heading = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = landing_pad_x;
        point.position.y = landing_pad_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = _descending_height_;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* ASCEND_TRAJECTORY //{ */

  } else if (trajectoryType == ASCEND_TRAJECTORY) {

    double desired_height, desired_vector, target_distance, direction;

    desired_height  = ascending_height_;
    desired_vector  = desired_height - init_z;
    target_distance = fabs(desired_vector);
    direction       = (desired_vector <= 0) ? -1 : 1;

    double step_size = _ascending_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = init_x;
      point.position.y = init_y;
      point.position.z = init_z;
      point.heading    = init_hdg;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = init_x;
        point.position.y = init_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = init_hdg;

        trajectory.points.push_back(point);
      }
    }

    return trajectory;

    //}

    /* LANDING_TRAJECTORY //{ */

  } else if (trajectoryType == LANDING_TRAJECTORY) {

    double desired_heading = landing_pad_hdg;  // TODO

    double target_distance = init_z - landing_pad_z - _landing_height_;
    double direction       = -1;
    double step_size       = _landing_speed_ * _trajectory_dt_;
    int    n_steps         = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = landing_pad_x;
        point.position.y = landing_pad_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z + _landing_height_;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* REPEAT_TRAJECTORY //{ */

  } else if (trajectoryType == REPEAT_TRAJECTORY) {

    double target_distance, desired_height, desired_vector, direction, desired_heading;

    desired_height  = landing_pad_z + _repeating_height_;
    desired_vector  = desired_height - init_z;
    target_distance = fabs(desired_vector);
    direction       = (desired_vector <= 0) ? -1 : 1;

    desired_heading = init_hdg;

    double step_size = _repeating_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::Reference point;

      for (int i = 0; i < n_steps; i++) {

        point.position.x = landing_pad_x;
        point.position.y = landing_pad_y;
        point.position.z = trajectory.points.back().position.z + direction * step_size;
        point.heading    = desired_heading;

        trajectory.points.push_back(point);
      }
    }

    // the last point
    {
      mrs_msgs::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = _repeating_height_;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* ABORT_TRAJECTORY //{ */

  } else if (trajectoryType == ABORT_TRAJECTORY) {

    mrs_msgs::Reference point;

    point.position.x = init_x;
    point.position.y = init_y;
    point.position.z = landing_pad_z + aborting_height_;
    point.heading    = init_hdg;

    trajectory.points.push_back(point);

    return trajectory;

    //}
  }

  return trajectory;
}

//}

// | ------------------------- setters ------------------------ |

/* setController() //{ */

void PreciseLanding::setController(std::string desired_controller) {

  mrs_msgs::String srv;
  srv.request.value = desired_controller;

  ROS_INFO("[PreciseLanding]: switching to controller: \"%s\"", desired_controller.c_str());

  bool res = sch_switch_controller_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[PreciseLanding]: service call for setController() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[PreciseLanding]: service call for setController() failed!");
  }
}

//}

/* disarm() //{ */

void PreciseLanding::disarm(void) {

  std_srvs::SetBool srv;

  srv.request.data = false;

  bool res = sch_arming_.call(srv);

  if (res) {
    if (!srv.response.success) {
      ROS_WARN_THROTTLE(1.0, "[PreciseLanding]: service call for arming() returned false: %s", srv.response.message.c_str());
    }
  } else {
    ROS_ERROR("[PreciseLanding]: service call for arming() failed!");
  }
}

//}

// | -------------------- support routines -------------------- |

/* alignedWithTarget() //{ */

bool PreciseLanding::alignedWithTarget(const double position_thr, const double heading_thr, Alignment_t mode) {

  auto                                     uav_state = sh_uav_state_.getMsg();
  geometry_msgs::PoseWithCovarianceStamped landing_pad;

  {
    auto landing_pad_orig = sh_landing_tag_.getMsg();

    auto result = transformer_->transformSingle(*landing_pad_orig, uav_state->header.frame_id);

    if (!result) {
      ROS_ERROR("[PreciseLanding]: could not transform landing_pad to '%s'", uav_state->header.frame_id.c_str());
      return false;
    }

    landing_pad = result.value();
  }

  double tar_x, tar_y, tar_z, tar_heading;
  double cur_x, cur_y, cur_z, cur_heading;

  tar_x       = landing_pad.pose.pose.position.x;
  tar_y       = landing_pad.pose.pose.position.y;
  tar_z       = landing_pad.pose.pose.position.z + _aligning_height_;
  tar_heading = mrs_lib::AttitudeConverter(landing_pad.pose.pose.orientation).getHeading();

  cur_x       = uav_state->pose.position.x;
  cur_y       = uav_state->pose.position.y;
  cur_z       = uav_state->pose.position.z;
  cur_heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();

  double position_error = 0;

  if (mode == MODE_3D) {
    position_error = sqrt(pow(cur_x - tar_x, 2) + pow(cur_y - tar_y, 2) + pow(cur_z - tar_z, 2));
  } else if (mode == MODE_2D) {
    position_error = sqrt(pow(cur_x - tar_x, 2) + pow(cur_y - tar_y, 2));
  }

  double heading_error = fabs(radians::diff(cur_heading, tar_heading));

  ROS_INFO_THROTTLE(1.0, "[PreciseLanding]: position error during alignment: %.3f m", position_error);

  if (position_error < position_thr && heading_error < heading_thr) {
    return true;
  } else {
    return false;
  }
}

//}

/* lastAlignmentCheck() //{ */

bool PreciseLanding::lastAlignmentCheck(void) {

  auto                                     uav_state = sh_uav_state_.getMsg();
  geometry_msgs::PoseWithCovarianceStamped landing_pad;

  {
    auto landing_pad_orig = sh_landing_tag_.getMsg();

    auto result = transformer_->transformSingle(*landing_pad_orig, uav_state->header.frame_id);

    if (!result) {
      ROS_ERROR("[PreciseLanding]: could not transform landing_pad to '%s'", uav_state->header.frame_id.c_str());
      return false;
    }

    landing_pad = result.value();
  }

  double tar_x, tar_y, tar_heading;
  double cur_x, cur_y, cur_heading;

  tar_x       = landing_pad.pose.pose.position.x;
  tar_y       = landing_pad.pose.pose.position.y;
  tar_heading = mrs_lib::AttitudeConverter(landing_pad.pose.pose.orientation).getHeading();

  cur_x       = uav_state->pose.position.x;
  cur_y       = uav_state->pose.position.y;
  cur_heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();

  double position_error_x = abs(cur_x - tar_x);
  double position_error_y = abs(cur_y - tar_y);
  double heading_error    = fabs(radians::diff(cur_heading, tar_heading));

  ROS_INFO_THROTTLE(1.0, "[PreciseLanding]: alignment error (control mode): x=%.3f m, y=%.3f m, heading=%.3f", position_error_x, position_error_y,
                    heading_error);

  if (position_error_x < aligning2_current_x_crit_ && position_error_y < aligning2_current_y_crit_ && heading_error < 0.1) {
    return true;
  } else {
    return false;
  }
}

//}

/* shouldTimeout() //{ */

bool PreciseLanding::shouldTimeout(const double &timeout) {

  if (timeouter_ == ros::Time::UNINITIALIZED) {
    return false;
  }

  if ((ros::Time::now() - timeouter_).toSec() > timeout) {
    return true;
  } else {
    return false;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* stateMachineTimer() //{ */

void PreciseLanding::stateMachineTimer([[maybe_unused]] const ros::TimerEvent &event) {

  ROS_INFO_ONCE("[PreciseLanding]: got data, working...");

  double cmd_odom_stable_heading = 0;

  switch (current_state_) {

      /* IDLE_STATE //{ */

    case IDLE_STATE: {

      break;
    }

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE: {

      if (shouldTimeout(_aligning_timeout_)) {

        ROS_ERROR("[PreciseLanding]: landing timed out, ABORTING");

        changeState(ABORT_STATE);
      }

      if (!see_landing_pad_) {

        ROS_INFO("[PreciseLanding]: landing pad not visible");

        changeState(ABORT_STATE);
      }

      auto trajectory = createTrajectory(ALIGN_TRAJECTORY);

      if (trajectory) {

        ph_trajectory_reference_.publish(trajectory.value());

      } else {

        changeState(ABORT_STATE);
      }

      if (alignedWithTarget(_aligning_radius_, 0.1, MODE_3D)) {

        ROS_INFO_THROTTLE(1, "[PreciseLanding]: aligned with the landing pad, DESCENDING");

        changeState(DESCEND_STATE);
      }

      break;
    }

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE: {

      if (shouldTimeout(_descending_timeout_)) {

        ROS_ERROR("[PreciseLanding]: timed out, re-ALIGNING");

        changeState(REPEAT_STATE);
      }

      if (!see_landing_pad_) {

        ROS_WARN_THROTTLE(1, "[PreciseLanding]: landing pad not visible");

        changeState(ABORT_STATE);
      }

      auto trajectory = createTrajectory(DESCEND_TRAJECTORY);

      // publish the trajectory
      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);
      }

      if (alignedWithTarget(0.2, 0.1, MODE_3D)) {

        ROS_INFO("[PreciseLanding]: correct height reached, ALIGNING for landing");

        changeState(ALIGN2_STATE);
      }

      break;
    }

      //}

      /* ALIGN2_STATE //{ */

    case ALIGN2_STATE: {

      if (shouldTimeout(_aligning2_timeout_)) {

        ROS_WARN("[PreciseLanding]: Aligning for landing took too long, ABORTING.");

        changeState(ABORT_STATE);
      }

      if (!see_landing_pad_) {

        ROS_WARN_THROTTLE(1, "[PreciseLanding]: landing pad not visible");

        changeState(ABORT_STATE);
      }

      auto trajectory = createTrajectory(DESCEND_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);
      }

      aligning2_current_x_crit_ += (1.0 / _main_rate_) * _aligning2_criterion_increase_rate_x_;
      aligning2_current_y_crit_ += (1.0 / _main_rate_) * _aligning2_criterion_increase_rate_y_;

      ROS_INFO_THROTTLE(1.0, "[PreciseLanding]: alignment x crit: %.1f cm, y crit: %.1f cm", aligning2_current_x_crit_ * 100.0,
                        aligning2_current_y_crit_ * 100.0);

      if (lastAlignmentCheck()) {

        if (!aligning2_in_radius_) {

          aligning2_in_radius_      = true;
          aligning2_in_radius_time_ = ros::Time::now();
        }

      } else {

        if (aligning2_in_radius_) {

          aligning2_in_radius_      = false;
          aligning2_in_radius_time_ = ros::Time(0);
          ROS_WARN("[PreciseLanding]: alignment disturbed");
        }
      }

      // | ---------------- check the alignment time ---------------- |
      if (aligning2_in_radius_) {

        double alignemnt_held_for = (ros::Time::now() - aligning2_in_radius_time_).toSec();

        if (alignemnt_held_for >= _aligning2_in_alignment_duration_) {

          ROS_INFO("[PreciseLanding]: alignment finished, LANDING");

          changeState(LANDING_STATE);

        } else {

          ROS_INFO_THROTTLE(0.1, "[PreciseLanding]: alignment holds for %.2f/%.2f s", alignemnt_held_for, _aligning2_in_alignment_duration_);
        }
      }

      break;
    }

      //}

      /* LANDING_STATE //{ */

    case LANDING_STATE: {

      auto trajectory = createTrajectory(LANDING_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);
      }

      auto nominal_msas   = sh_mass_nominal_.getMsg();
      auto estimated_mass = sh_mass_estimate_.getMsg();

      if (estimated_mass->data < (0.5 * nominal_msas->data)) {

        ROS_INFO("[PreciseLanding]: landing finished");

        disarm();
        changeState(IDLE_STATE);

        return;
      }

      break;
    }

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE: {

      auto trajectory = createTrajectory(REPEAT_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);
      }

      // | -------------------- check the height -------------------- |
      if (alignedWithTarget(0.3, 0.1, MODE_3D)) {

        changeState(ALIGN_STATE);
      }

      break;
    }

      //}

      /* ASCEND_STATE //{ */

    case ASCEND_STATE: {

      auto trajectory = createTrajectory(ASCEND_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);
      }

      if (alignedWithTarget(0.5, 0.1, MODE_3D)) {

        changeState(IDLE_STATE);
      }

      break;
    }

      //}

      /* ABORT_STATE //{ */

    case ABORT_STATE: {

      auto trajectory = createTrajectory(ABORT_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);
      }

      // | --------------- check if we reached to top --------------- |
      if (alignedWithTarget(0.5, 0.1, MODE_3D)) {

        changeState(IDLE_STATE);
      }

      break;
    }

      //}
  }
}

//}

}  // namespace mrs_precise_landing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_precise_landing::PreciseLanding, nodelet::Nodelet)
