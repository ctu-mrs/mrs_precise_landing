/* include //{ */

#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/node.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/median_filter.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <mrs_msgs/msg/trajectory_reference.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/path_srv.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/srv/float64_stamped_srv.hpp>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

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

/* STRUCTURES and TYPEDEFS //{ */

[[maybe_unused]] enum {

  IDLE_STATE,
  ALIGN_STATE,
  DESCEND_STATE,
  ALIGN2_STATE,
  LANDING_STATE,
  REPEAT_STATE,
  ABORT_STATE,

} States_t;

const char *state_names[7] = {
    "IDLING", "ALIGNING", "DESCENDING", "ALIGNING2", "LANDING", "REPEATING", "ABORTING",
};

[[maybe_unused]] enum {

  DESCEND_TRAJECTORY,
  LANDING_TRAJECTORY,
  REPEAT_TRAJECTORY,
  ABORT_TRAJECTORY,

} Trajectory_t;

//}

// --------------------------------------------------------------
// |                          the class                         |
// --------------------------------------------------------------

/* class PreciseLanding //{ */

class PreciseLanding : public mrs_lib::Node {

public:
  PreciseLanding(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;
  bool is_initialized_ = false;

  std::string _uav_name_;

  std::string _frame_id_;

  std::unique_ptr<mrs_lib::Transformer> transformer_;

  mrs_lib::PublisherHandler<mrs_msgs::msg::TrajectoryReference> ph_trajectory_reference_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>                 sh_tracker_cmd_;
  mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseWithCovarianceStamped> sh_landing_pad_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>                       sh_uav_state_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>                        sh_mass_estimate_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>      sh_ctrl_diag_;

  void callbackTimeoutPad(const std::string& topic_name, const rclcpp::Time& last_msg);

  void callbackLandingPad(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server_land_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_servcer_stop_;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>            sch_switch_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>            sch_switch_tracker_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>           sch_arming_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Float64StampedSrv> sch_set_min_z;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>           sch_enable_min_height_check_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>           sch_path_;

  // params loaded from config file
  double _trajectory_dt_;

  double inital_mass_estimate_;

  std::atomic<bool> see_landing_pad_ = false;

  rclcpp::Time timeouter_;

  std::string controller_;
  std::string tracker_;

  bool   _heading_relative_to_pad_enabled_;
  double _heading_relative_to_pad_;

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

  double _aligning2_criterion_initial_radius_;
  double _aligning2_criterion_radius_increase_rate_;
  double _aligning2_criterion_radius_limit_;

  double _aligning2_in_alignment_duration_;

  rclcpp::Time aligning2_in_radius_time_;
  bool         aligning2_in_radius_ = false;
  double       aligning2_current_radius_;

  // landing params
  rclcpp::Time landing_since_;

  double _landing_speed_;
  double _landing_height_;
  double _landing_mass_factor_;
  int    _landing_repeat_threshold_;
  bool   _landing_disarming_vision_enabled_;
  double _landing_disarming_vision_distance_;

  // repeating params
  double _repeating_speed_;
  double _repeating_height_;
  double _repeating_timeout_;

  // aborting params
  double aborting_height_;

  void callbackLand([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void callbackAbort([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool setController(const std::string &desired_controller);

  bool setTracker(const std::string &desired_tracker);

  void disarm(void);

  bool setMinZ(const double state);

  bool enableMinHeightCheck(const bool state);

  void gotoPath(const double x, const double y, const double z, const double hdg, const std::string &frame);

  bool alignmentCheck(const double &desired_height, const double &position_hor_thr, const double &position_vert_thr, const double &heading_thr);

  bool shouldTimeout(const double &timeout);

  std::optional<mrs_msgs::msg::ReferenceStamped> getTransformedTrackerCmd(const std::string &frame_id);

  std::optional<mrs_msgs::msg::ReferenceStamped> getTransformedLandingPad(const std::string &frame_id);

  std::optional<mrs_msgs::msg::ReferenceStamped> getTransformedUavState(const std::string &frame_id);

  // state machine
  int        current_state_, previous_state_;
  std::mutex mutex_state_;

  int repeat_landing_counter;

  std::shared_ptr<TimerType> state_machine_timer_;
  void       stateMachineTimer();

public:
  double _main_rate_;

  void changeState(int newState);

  std::optional<mrs_msgs::msg::TrajectoryReference> createTrajectory(int trajectoryType);
};

//}

/* PreciseLanding() //{ */

PreciseLanding::PreciseLanding(rclcpp::NodeOptions options) : mrs_lib::Node("PreciseLanding", options) {

  node_ = this_node_ptr();
  clock_ = node_->get_clock();

  mrs_lib::ParamLoader param_loader(node_);
  param_loader.addYamlFileFromParam("config");

  std::string custom_config_path;
  if (param_loader.loadParam("custom_config", custom_config_path) && !custom_config_path.empty())
    param_loader.addYamlFileFromParam("custom_config");

  // rclcpp::Time::waitForValid(); TODO: replacement?

  param_loader.loadParam("uav_name", _uav_name_);

  param_loader.loadParam("frame_id", _frame_id_);

  param_loader.loadParam("rate", _main_rate_);

  param_loader.loadParam("trajectory_dt", _trajectory_dt_);

  param_loader.loadParam("controller", controller_);
  param_loader.loadParam("tracker", tracker_);

  param_loader.loadParam("desired_heading/relative_to_pad/enabled", _heading_relative_to_pad_enabled_);
  param_loader.loadParam("desired_heading/relative_to_pad/heading", _heading_relative_to_pad_);

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

  param_loader.loadParam("stages/aligning2/criterion/initial_radius", _aligning2_criterion_initial_radius_);
  param_loader.loadParam("stages/aligning2/criterion/radius_increase_rate", _aligning2_criterion_radius_increase_rate_);
  param_loader.loadParam("stages/aligning2/criterion/limit_radius", _aligning2_criterion_radius_limit_);

  param_loader.loadParam("stages/aligning2/in_alignment_duration", _aligning2_in_alignment_duration_);

  // landing params
  param_loader.loadParam("stages/landing/speed", _landing_speed_);
  param_loader.loadParam("stages/landing/height", _landing_height_);
  param_loader.loadParam("stages/landing/disarming/mass_factor", _landing_mass_factor_);
  param_loader.loadParam("stages/landing/repeat_threshold", _landing_repeat_threshold_);
  param_loader.loadParam("stages/landing/disarming/vision/enabled", _landing_disarming_vision_enabled_);
  param_loader.loadParam("stages/landing/disarming/vision/distance", _landing_disarming_vision_distance_);

  // repeating params
  param_loader.loadParam("stages/repeating/speed", _repeating_speed_);
  param_loader.loadParam("stages/repeating/height", _repeating_height_);
  param_loader.loadParam("stages/repeating/timeout", _repeating_timeout_);

  // aborting params
  param_loader.loadParam("stages/aborting/height", aborting_height_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- publishers ----------------------- |

  ph_trajectory_reference_ = mrs_lib::PublisherHandler<mrs_msgs::msg::TrajectoryReference>(node_, "trajectory_reference_out");

  // | --------------------- service clients -------------------- |

  sch_switch_controller_       = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "switch_controller_out");
  sch_switch_tracker_          = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "switch_tracker_out");
  sch_arming_                  = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "arming_out");
  sch_set_min_z                = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Float64StampedSrv>(node_, "set_min_z_out");
  sch_enable_min_height_check_ = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "enable_min_height_check_out");
  sch_path_                    = mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>(node_, "path_out");

  // | --------------------- sercice servers -------------------- |

  service_server_land_ = node_->create_service<std_srvs::srv::Trigger>("land_in", std::bind(&PreciseLanding::callbackLand, this, std::placeholders::_1, std::placeholders::_2));
  service_servcer_stop_ = node_->create_service<std_srvs::srv::Trigger>("abort_in", std::bind(&PreciseLanding::callbackAbort, this, std::placeholders::_1, std::placeholders::_2));

  // | ----------------------- subscribers ---------------------- |

  {
    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node               = node_;
    //shopts.node_name          = "PreciseLanding";
    shopts.no_message_timeout = rclcpp::Duration::from_seconds(1.0);
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    //shopts.queue_size         = 10;
    //shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_landing_pad_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(shopts, "landing_pad_in", &PreciseLanding::callbackLandingPad, this,
                                                                                          &PreciseLanding::callbackTimeoutPad, this);
  }

  {
    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node               = node_;
    //shopts.node_name          = "PreciseLanding";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    //shopts.queue_size         = 10;
    //shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_tracker_cmd_   = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "tracker_cmd_in");
    sh_uav_state_     = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>(shopts, "uav_state_in");
    sh_mass_estimate_ = mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(shopts, "mass_estimate_in");
    sh_ctrl_diag_     = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "ctrl_diag_in");
  }

  // state machine
  current_state_  = IDLE_STATE;
  previous_state_ = IDLE_STATE;

  // | ------------------------- timers ------------------------- |

  // start timers
  {
    mrs_lib::TimerHandlerOptions timer_opts;
    timer_opts.node      = node_;
    timer_opts.autostart = true;

    state_machine_timer_ = std::make_shared<TimerType>(timer_opts, rclcpp::Rate(_main_rate_, clock_), std::bind(&PreciseLanding::stateMachineTimer, this));
  }

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackLandingPad() //{ */

void PreciseLanding::callbackLandingPad([[maybe_unused]] const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting landing pad pose");

  see_landing_pad_ = true;
}

//}

/* callbackTimeoutPad() //{ */

void PreciseLanding::callbackTimeoutPad([[maybe_unused]] const std::string& topic_name, [[maybe_unused]] const rclcpp::Time& last_msg) {

  see_landing_pad_ = false;
}

//}

// | ------------------------ services ------------------------ |

/* callbackStop() //{ */

void PreciseLanding::callbackAbort([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  res->success = true;
  res->message = "aborting landing";

  {
    std::scoped_lock lock(mutex_state_);

    changeState(ABORT_STATE);
  }
}

//}

/* callbackLand() //{ */

void PreciseLanding::callbackLand([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  {
    std::stringstream ss;

    if (!sh_uav_state_.hasMsg()) {
      ss << "missing UAV state";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
      res->message = ss.str();
      res->success = false;
      return;
    }

    if (!sh_landing_pad_.hasMsg()) {
      ss << "missing landing pad detections";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
      res->message = ss.str();
      res->success = false;
      return;
    }

    if (!sh_tracker_cmd_.hasMsg()) {
      ss << "missing tracker cmd";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
      res->message = ss.str();
      res->success = false;
      return;
    }

    if (!sh_mass_estimate_.hasMsg()) {
      ss << "missing estimated mass";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
      res->message = ss.str();
      res->success = false;
      return;
    }

    if (!sh_ctrl_diag_.hasMsg()) {
      ss << "missing control manager diagnostics";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
      res->message = ss.str();
      res->success = false;
      return;
    }

    if (!sh_ctrl_diag_.getMsg()->flying_normally) {
      ss << "not flying normally";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
      res->message = ss.str();
      res->success = false;
      return;
    }

    if (!see_landing_pad_) {
      ss << "landing pad not detected";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000, ss.str());
      res->message = ss.str();
      res->success = false;
      return;
    }
  }

  {
    std::scoped_lock lock(mutex_state_);

    if (current_state_ == IDLE_STATE) {

      changeState(ALIGN_STATE);

      res->success = true;
      res->message = "landing has started";

    } else {

      res->success = false;
      res->message = "landing already in process";
    }
  }
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void PreciseLanding::changeState(int newState) {

  // just for ROS_INFO
  RCLCPP_INFO(node_->get_logger(), "Switching states: %s -> %s", state_names[current_state_], state_names[newState]);

  previous_state_ = current_state_;
  current_state_  = newState;

  timeouter_ = clock_->now();

  // if changing to idle, stop the drone
  switch (newState) {

      /* IDLE_STATE //{ */

    case IDLE_STATE: {

      repeat_landing_counter = 0;

      break;
    }

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE: {

      if (!setController(controller_)) {
        RCLCPP_ERROR(node_->get_logger(), "failed to switch controller");
        changeState(IDLE_STATE);
        return;
      }

      if (!setTracker(tracker_)) {
        RCLCPP_ERROR(node_->get_logger(), "failed to switch tracker");
        changeState(IDLE_STATE);
        return;
      }

      if (!enableMinHeightCheck(false)) {
        RCLCPP_ERROR(node_->get_logger(), "failed to disable the min height check");
        changeState(IDLE_STATE);
        return;
      }

      break;
    }

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE: {

      auto mass_estimate = sh_mass_estimate_.getMsg();

      inital_mass_estimate_ = mass_estimate->data;

      break;
    }

      //}

      /* ALIGN2_STATE //{ */

    case ALIGN2_STATE: {

      aligning2_in_radius_time_ = rclcpp::Time(0);
      aligning2_in_radius_      = false;
      aligning2_current_radius_ = _aligning2_criterion_initial_radius_;

      break;
    }

      //}

      /* LANDING_STATE //{ */

    case LANDING_STATE: {

      // log when we started the landing
      landing_since_ = clock_->now();

      break;
    }

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE: {

      if (repeat_landing_counter++ >= _landing_repeat_threshold_) {

        RCLCPP_INFO(node_->get_logger(), "Exceeded the number of landing attempts, aborting");

        changeState(ABORT_STATE);
      }

      break;
    }

      //}

      /* ABORT_STATE //{ */

    case ABORT_STATE: {

      RCLCPP_DEBUG(node_->get_logger(), "aborting");

      break;
    }

      //}
  }
}

//}

/* createTrajectory() //{ */

std::optional<mrs_msgs::msg::TrajectoryReference> PreciseLanding::createTrajectory(int trajectoryType) {

  auto tracker_cmd = getTransformedTrackerCmd(_frame_id_);
  auto landing_pad = getTransformedLandingPad(_frame_id_);

  if (!tracker_cmd || !landing_pad) {
    return {};
  }

  double init_x   = tracker_cmd->reference.position.x;
  double init_y   = tracker_cmd->reference.position.y;
  double init_z   = tracker_cmd->reference.position.z;
  double init_hdg = tracker_cmd->reference.heading;

  double landing_pad_x   = landing_pad->reference.position.x;
  double landing_pad_y   = landing_pad->reference.position.y;
  double landing_pad_z   = landing_pad->reference.position.z;
  double landing_pad_hdg = landing_pad->reference.heading;

  // prepare the trajectorie
  // pose array for debugging
  mrs_msgs::msg::TrajectoryReference trajectory;
  tf2::Quaternion                    orientation;
  trajectory.fly_now     = true;
  trajectory.use_heading = true;
  trajectory.header      = tracker_cmd->header;

  /* DESCEND_TRAJECTORY //{ */

  if (trajectoryType == DESCEND_TRAJECTORY) {

    double target_distance, direction, desired_height, desired_vector, desired_heading;

    desired_height  = landing_pad_z + _descending_height_;
    desired_vector  = desired_height - init_z;
    target_distance = std::abs(desired_vector);
    direction       = (desired_vector <= 0) ? -1 : 1;

    if (_heading_relative_to_pad_enabled_) {
      desired_heading = landing_pad_hdg + _heading_relative_to_pad_;
    } else {
      desired_heading = init_hdg;
    }

    double step_size = _descending_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;

      point.heading = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::msg::Reference point;

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
      mrs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* LANDING_TRAJECTORY //{ */

  } else if (trajectoryType == LANDING_TRAJECTORY) {

    double desired_heading;

    if (_heading_relative_to_pad_enabled_) {
      desired_heading = landing_pad_hdg + _heading_relative_to_pad_;
    } else {
      desired_heading = init_hdg;
    }

    double target_distance = init_z - landing_pad_z - _landing_height_;
    double direction       = -1;
    double step_size       = _landing_speed_ * _trajectory_dt_;
    int    n_steps         = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::msg::Reference point;

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
      mrs_msgs::msg::Reference point;

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
    target_distance = std::abs(desired_vector);
    direction       = (desired_vector <= 0) ? -1 : 1;

    if (_heading_relative_to_pad_enabled_) {
      desired_heading = landing_pad_hdg + _heading_relative_to_pad_;
    } else {
      desired_heading = init_hdg;
    }

    double step_size = _repeating_speed_ * _trajectory_dt_;
    int    n_steps   = int(floor(target_distance / step_size));

    // the first point
    {
      mrs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = init_z;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    // sample the trajectory
    {
      mrs_msgs::msg::Reference point;

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
      mrs_msgs::msg::Reference point;

      point.position.x = landing_pad_x;
      point.position.y = landing_pad_y;
      point.position.z = desired_height;
      point.heading    = desired_heading;

      trajectory.points.push_back(point);
    }

    return trajectory;

    //}

    /* ABORT_TRAJECTORY //{ */

  } else if (trajectoryType == ABORT_TRAJECTORY) {

    mrs_msgs::msg::Reference point;

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

bool PreciseLanding::setController(const std::string &desired_controller) {

  auto srv = std::make_shared<mrs_msgs::srv::String::Request>();
  srv->value = desired_controller;

  RCLCPP_INFO(node_->get_logger(), "switching to controller: \"%s\"", desired_controller.c_str());

  auto res = sch_switch_controller_.callSync(srv);

  if (res) {
    if (!res.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for setController() returned false: %s", res.value()->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for setController() failed!");
    return false;
  }

  return true;
}

//}

/* setTracker() //{ */

bool PreciseLanding::setTracker(const std::string &desired_tracker) {

  auto srv = std::make_shared<mrs_msgs::srv::String::Request>();
  srv->value = desired_tracker;

  RCLCPP_INFO(node_->get_logger(), "switching to tracker: \"%s\"", desired_tracker.c_str());

  auto res = sch_switch_tracker_.callSync(srv);

  if (res) {
    if (!res.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for setTracker() returned false: %s", res.value()->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for setTracker() failed!");
    return false;
  }

  return true;
}

//}

/* disarm() //{ */

void PreciseLanding::disarm(void) {

  auto srv = std::make_shared<std_srvs::srv::SetBool::Request>();

  srv->data = false;

  auto res = sch_arming_.callSync(srv);

  if (res) {
    if (!res.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for disarm() returned false: %s", res.value()->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for disarm() failed!");
  }
}

//}

/* setMinZ() //{ */

bool PreciseLanding::setMinZ(const double z) {

  auto srv = std::make_shared<mrs_msgs::srv::Float64StampedSrv::Request>();
  srv->header.frame_id = _frame_id_;
  srv->value           = z;

  auto res = sch_set_min_z.callSync(srv);

  if (res) {
    if (!res.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for setMinZ() returned false: %s", res.value()->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for setMinZ() failed!");
    return false;
  }

  return true;
}

//}

/* enableMinHeightCheck() //{ */

bool PreciseLanding::enableMinHeightCheck(const bool state) {

  auto srv = std::make_shared<std_srvs::srv::SetBool::Request>();
  srv->data = state;

  RCLCPP_INFO(node_->get_logger(), "%s min height check", state ? "enabling" : "disabling");

  auto res = sch_enable_min_height_check_.callSync(srv);

  if (res) {
    if (!res.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for enableMinHeightCheck() returned false: %s", res.value()->message.c_str());
      return false;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for enableMinHeightCheck() failed!");
    return false;
  }

  return true;
}

//}

/* gotoPath() //{ */

void PreciseLanding::gotoPath(const double x, const double y, const double z, const double hdg, const std::string &frame) {

  auto srv = std::make_shared<mrs_msgs::srv::PathSrv::Request>();
  srv->path.fly_now                 = true;
  srv->path.header.frame_id         = frame;
  srv->path.max_execution_time      = 0.9;
  srv->path.max_deviation_from_path = 0.5;
  srv->path.use_heading             = true;

  srv->path.override_constraints                 = true;
  srv->path.override_max_acceleration_vertical   = 2.0;
  srv->path.override_max_acceleration_horizontal = 2.0;
  srv->path.override_max_velocity_vertical       = _aligning_speed_;
  srv->path.override_max_velocity_horizontal     = _aligning_speed_;
  srv->path.override_max_jerk_vertical           = 20.0;
  srv->path.override_max_jerk_horizontal         = 20.0;

  mrs_msgs::msg::Reference point;
  point.position.x = x;
  point.position.y = y;
  point.position.z = z;
  point.heading    = hdg;

  srv->path.points.push_back(point);

  auto res = sch_path_.callSync(srv);

  if (res) {
    if (!res.value()->success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for gotoPath() returned false: %s", res.value()->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "service call for gotoPath() failed!");
  }
}

//}

// | -------------------- support routines -------------------- |

/* shouldTimeout() //{ */

bool PreciseLanding::shouldTimeout(const double &timeout) {

  if (timeouter_ == rclcpp::Time(0)) {
    return false;
  }

  if ((clock_->now() - timeouter_).seconds() > timeout) {
    return true;
  } else {
    return false;
  }
}

//}

/* getTransformedTrackerCmd() //{ */

std::optional<mrs_msgs::msg::ReferenceStamped> PreciseLanding::getTransformedTrackerCmd(const std::string &frame_id) {

  if (!sh_tracker_cmd_.hasMsg()) {
    return {};
  }

  auto tracker_cmd = sh_tracker_cmd_.getMsg();

  mrs_msgs::msg::ReferenceStamped out;

  {
    out.header             = tracker_cmd->header;
    out.reference.position = tracker_cmd->position;
    out.reference.heading  = tracker_cmd->heading;

    auto result = transformer_->transformSingle(out, _frame_id_);

    if (result) {
      out = result.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "could not transform tracker cmd to '%s'", _frame_id_.c_str());
      return {};
    }
  }

  return {out};
}

//}

/* getTransformedLandingPad() //{ */

std::optional<mrs_msgs::msg::ReferenceStamped> PreciseLanding::getTransformedLandingPad(const std::string &frame_id) {

  if (!sh_landing_pad_.hasMsg()) {
    return {};
  }

  auto landing_pad = sh_landing_pad_.getMsg();

  mrs_msgs::msg::ReferenceStamped out;

  {
    out.header             = landing_pad->header;
    out.reference.position = landing_pad->pose.pose.position;
    out.reference.heading  = mrs_lib::AttitudeConverter(landing_pad->pose.pose.orientation).getHeading();

    auto result = transformer_->transformSingle(out, _frame_id_);

    if (result) {
      out = result.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "could not transform landing pad to '%s'", _frame_id_.c_str());
      return {};
    }
  }

  return {out};
}

//}

/* getTransformedUavState() //{ */

std::optional<mrs_msgs::msg::ReferenceStamped> PreciseLanding::getTransformedUavState(const std::string &frame_id) {

  if (!sh_uav_state_.hasMsg()) {
    return {};
  }

  auto uav_state = sh_uav_state_.getMsg();

  mrs_msgs::msg::ReferenceStamped out;

  {
    out.header             = uav_state->header;
    out.reference.position = uav_state->pose.position;
    out.reference.heading  = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();

    auto result = transformer_->transformSingle(out, _frame_id_);

    if (result) {
      result = result.value();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "could not transform UAV state to '%s'", _frame_id_.c_str());
      return {};
    }
  }

  return {out};
}

//}

/* alignmentCheck() //{ */

bool PreciseLanding::alignmentCheck(const double &desired_height, const double &position_hor_thr, const double &position_vert_thr, const double &heading_thr) {

  auto uav_state   = getTransformedUavState(_frame_id_);
  auto landing_pad = getTransformedLandingPad(_frame_id_);

  if (!landing_pad || !uav_state) {
    return false;
  }

  double tar_x, tar_y, tar_z, tar_heading;
  double cur_x, cur_y, cur_z, cur_heading;

  tar_x = landing_pad->reference.position.x;
  tar_y = landing_pad->reference.position.y;
  tar_z = landing_pad->reference.position.z + desired_height;

  cur_x       = uav_state->reference.position.x;
  cur_y       = uav_state->reference.position.y;
  cur_z       = uav_state->reference.position.z;
  cur_heading = uav_state->reference.heading;

  if (_heading_relative_to_pad_enabled_) {
    tar_heading = landing_pad->reference.heading + _heading_relative_to_pad_;
  } else {
    tar_heading = cur_heading;
  }

  double horizontal_error = std::hypot(cur_x - tar_x, cur_y - tar_y);
  double vertical_error   = std::abs(cur_z - tar_z);
  double heading_error    = std::abs(radians::diff(cur_heading, tar_heading));

  if (horizontal_error < position_hor_thr && vertical_error < position_vert_thr && heading_error < heading_thr) {
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

void PreciseLanding::stateMachineTimer() {

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_state_);

  RCLCPP_INFO_ONCE(node_->get_logger(), "stateMachineTimer() running");

  // | ------- check for nor normal flying for all states ------- |

  if (current_state_ != IDLE_STATE) {

    auto ctrl_diag = sh_ctrl_diag_.getMsg();

    if (!ctrl_diag->flying_normally) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "not flying normally, aborting");

      changeState(IDLE_STATE);

      return;
    }
  }

  // | ------------------- state machine logic ------------------ |

  switch (current_state_) {

      /* IDLE_STATE //{ */

    case IDLE_STATE: {

      break;
    }

      //}

      /* ALIGN_STATE //{ */

    case ALIGN_STATE: {

      if (shouldTimeout(_aligning_timeout_)) {

        RCLCPP_ERROR(node_->get_logger(), "landing timed out, ABORTING");

        changeState(ABORT_STATE);

        return;
      }

      if (!see_landing_pad_) {

        RCLCPP_INFO(node_->get_logger(), "landing pad not visible");

        changeState(ABORT_STATE);

        return;
      }

      auto landing_pad = getTransformedLandingPad(_frame_id_);
      auto tracker_cmd = getTransformedTrackerCmd(_frame_id_);

      double des_x = landing_pad->reference.position.x;
      double des_y = landing_pad->reference.position.y;
      double des_z = landing_pad->reference.position.z + _aligning_height_;
      double des_heading;

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      if (_heading_relative_to_pad_enabled_) {
        des_heading = landing_pad->reference.heading + _heading_relative_to_pad_;
      } else {
        des_heading = tracker_cmd->reference.heading;
      }

      gotoPath(des_x, des_y, des_z, des_heading, _frame_id_);

      if (alignmentCheck(_aligning_height_, _aligning_radius_, 0.2, 0.1)) {

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "aligned with the landing pad, DESCENDING");

        changeState(DESCEND_STATE);

        return;
      }

      break;
    }

      //}

      /* DESCEND_STATE //{ */

    case DESCEND_STATE: {

      if (shouldTimeout(_descending_timeout_)) {

        RCLCPP_ERROR(node_->get_logger(), "timed out, re-ALIGNING");

        changeState(REPEAT_STATE);

        return;
      }

      if (!see_landing_pad_) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "landing pad not visible");

        changeState(ABORT_STATE);

        return;
      }

      auto landing_pad = getTransformedLandingPad(_frame_id_);

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      auto trajectory = createTrajectory(DESCEND_TRAJECTORY);

      // publish the trajectory
      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      if (alignmentCheck(_descending_height_, 0.1, 0.2, 0.1)) {

        RCLCPP_INFO(node_->get_logger(), "correct height reached, ALIGNING for landing");

        changeState(ALIGN2_STATE);

        return;
      }

      break;
    }

      //}

      /* ALIGN2_STATE //{ */

    case ALIGN2_STATE: {

      if (shouldTimeout(_aligning2_timeout_)) {

        RCLCPP_WARN(node_->get_logger(), "Aligning for landing took too long, ABORTING.");

        changeState(ABORT_STATE);

        return;
      }

      if (!see_landing_pad_) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "landing pad not visible");

        changeState(ABORT_STATE);

        return;
      }

      auto landing_pad = getTransformedLandingPad(_frame_id_);

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      auto trajectory = createTrajectory(DESCEND_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "alignment radius criterion: %.1f cm", aligning2_current_radius_ * 100.0);

      if (alignmentCheck(_descending_height_, aligning2_current_radius_, 0.1, 0.1)) {

        if (!aligning2_in_radius_) {

          aligning2_in_radius_      = true;
          aligning2_in_radius_time_ = clock_->now();
        }

      } else {

        aligning2_current_radius_ += (1.0 / _main_rate_) * _aligning2_criterion_radius_increase_rate_;

        if (aligning2_current_radius_ > _aligning2_criterion_radius_limit_) {
          aligning2_current_radius_ = _aligning2_criterion_radius_limit_;
        }

        if (aligning2_in_radius_) {

          aligning2_in_radius_      = false;
          aligning2_in_radius_time_ = rclcpp::Time(0);
          RCLCPP_WARN(node_->get_logger(), "alignment disturbed");
        }
      }

      // | ---------------- check the alignment time ---------------- |

      if (aligning2_in_radius_) {

        double alignemnt_held_for = (clock_->now() - aligning2_in_radius_time_).seconds();

        if (alignemnt_held_for > _aligning2_in_alignment_duration_) {

          RCLCPP_INFO(node_->get_logger(), "alignment finished, LANDING");

          changeState(LANDING_STATE);

          return;

        } else {

          RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 100, "alignment holds for %.2f/%.2f s", alignemnt_held_for, _aligning2_in_alignment_duration_);
        }
      }

      break;
    }

      //}

      /* LANDING_STATE //{ */

    case LANDING_STATE: {

      auto landing_pad = getTransformedLandingPad(_frame_id_);

      if (!setMinZ(landing_pad->reference.position.z + 1.5 * _landing_height_)) {
        RCLCPP_ERROR(node_->get_logger(), "failed to set safety area's min Z");
        changeState(ABORT_STATE);
        return;
      }

      auto trajectory = createTrajectory(LANDING_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      auto estimated_mass = sh_mass_estimate_.getMsg();

      if (estimated_mass->data < (_landing_mass_factor_ * inital_mass_estimate_)) {

        RCLCPP_INFO(node_->get_logger(), "disarming using mass estimator threshold");

        RCLCPP_INFO(node_->get_logger(), "landing finished");

        disarm();

        changeState(IDLE_STATE);

        return;
      }

      if (_landing_disarming_vision_enabled_ && alignmentCheck(0.0, 0.1, _landing_disarming_vision_distance_, 1.0)) {

        RCLCPP_INFO(node_->get_logger(), "disarming using vision-based threshold");

        RCLCPP_INFO(node_->get_logger(), "landing finished");

        disarm();

        changeState(IDLE_STATE);

        return;
      }

      break;
    }

      //}

      /* REPEAT_STATE //{ */

    case REPEAT_STATE: {

      if (shouldTimeout(_repeating_timeout_)) {

        RCLCPP_ERROR(node_->get_logger(), "repeating timed out, ABORTING");

        changeState(ABORT_STATE);

        return;
      }

      auto trajectory = createTrajectory(REPEAT_TRAJECTORY);

      if (trajectory) {
        ph_trajectory_reference_.publish(trajectory.value());
      } else {
        changeState(ABORT_STATE);

        return;
      }

      // | -------------------- check the height -------------------- |
      if (alignmentCheck(_repeating_height_, 0.2, 0.2, 0.1)) {

        changeState(ALIGN_STATE);

        return;
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

        return;
      }

      changeState(IDLE_STATE);

      break;
    }

      //}
  }
}

//}

}  // namespace mrs_precise_landing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_precise_landing::PreciseLanding)
