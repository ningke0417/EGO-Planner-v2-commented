#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/TrajectoryExecution.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace Eigen;

ros::Publisher pos_cmd_pub;
ros::Publisher execution_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
bool heartbeat_paused_ = false;
ros::Time hold_stamp_(0);
Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
Eigen::Vector3d last_vel_ = Eigen::Vector3d::Zero();
Eigen::Vector3d last_acc_ = Eigen::Vector3d::Zero();
int drone_id_ = -1;
uint64_t execution_session_ = 0, execution_sequence_ = 0;
uint8_t execution_mode_ = traj_utils::TrajectoryExecution::WAITING;
bool require_replan_ = false;
double execution_elapsed_ = 0.0;
double heartbeat_timeout_ = 0.5, max_message_age_ = 0.3;
double max_start_jump_ = 0.3;
ros::Time last_execution_publish_(0);
ros::Time last_published_start_(0);
uint8_t last_published_mode_ = 0;
bool last_published_replan_ = false;
bool last_published_heartbeat_fresh_ = false;

// yaw control
double last_yaw_, last_yawdot_;
double time_forward_;

bool heartbeatFresh(const ros::Time &now)
{
  return !heartbeat_time_.isZero() && now >= heartbeat_time_ &&
         (now - heartbeat_time_).toSec() <= heartbeat_timeout_;
}

void latchHold(const ros::Time &now, const char *reason = "lost planner heartbeat")
{
  if (!receive_traj_ || heartbeat_paused_)
    return;
  heartbeat_paused_ = true;
  require_replan_ = true;
  hold_stamp_ = now;
  execution_mode_ = traj_utils::TrajectoryExecution::HOLDING;
  last_vel_.setZero();
  last_acc_.setZero();
  ROS_ERROR("[traj_server] Execution hold (%s); keeping commanded position until a fresh replacement trajectory arrives.", reason);
}

void holdCallback(const std_msgs::Empty::ConstPtr &)
{
  // The safety/FSM path may explicitly fence an otherwise live execution.
  // Repeated requests while already held do not move the creation-time fence.
  latchHold(ros::Time::now(), "planner requested safety hold");
}

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  const ros::Time now = ros::Time::now();
  // Detect a gap even if this callback runs before the delayed command timer.
  if (receive_traj_ && !heartbeat_time_.isZero() && !heartbeatFresh(now))
    latchHold(now);
  if (heartbeat_paused_ && !heartbeatFresh(now))
  {
    ROS_WARN("[traj_server] Planner heartbeat restored; old trajectory remains held and requires replanning.");
  }
  heartbeat_time_ = now;
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  const ros::Time now = ros::Time::now();
  if (receive_traj_ && !heartbeatFresh(now))
    latchHold(now);
  if (!heartbeatFresh(now))
  {
    ROS_WARN_THROTTLE(1.0, "[traj_server] Rejecting trajectory without fresh planner heartbeat.");
    return;
  }
  if (msg->order != 5 || msg->duration.empty() || msg->traj_id < 0 ||
      msg->drone_id < 0 || (drone_id_ >= 0 && msg->drone_id != drone_id_))
  {
    ROS_ERROR("[traj_server] Invalid trajectory order, identity, or empty duration list.");
    return;
  }
  const size_t coefficients = msg->duration.size() * 6;
  if (coefficients != msg->coef_x.size() || coefficients != msg->coef_y.size() ||
      coefficients != msg->coef_z.size())
  {
    ROS_ERROR("[traj_server] Invalid trajectory coefficient dimensions.");
    return;
  }
  const double age = (now - msg->start_time).toSec();
  if (msg->start_time.isZero() || age < -0.05 || age > max_message_age_ ||
      (receive_traj_ && msg->start_time <= start_time_) ||
      (heartbeat_paused_ && msg->start_time <= hold_stamp_))
  {
    ROS_WARN_THROTTLE(1.0, "[traj_server] Rejecting stale, replayed, future, or pre-hold trajectory.");
    return;
  }
  double total_duration = 0.0;
  for (float duration : msg->duration)
  {
    if (!std::isfinite(duration) || duration <= 0.0)
    {
      ROS_ERROR("[traj_server] Nonfinite/nonpositive trajectory duration.");
      return;
    }
    total_duration += duration;
  }
  for (const auto *axis : {&msg->coef_x, &msg->coef_y, &msg->coef_z})
    for (float value : *axis)
      if (!std::isfinite(value))
      {
        ROS_ERROR("[traj_server] Nonfinite trajectory coefficient.");
        return;
      }
  if (!std::isfinite(total_duration) || age >= total_duration)
  {
    ROS_WARN("[traj_server] Rejecting already expired trajectory.");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  boost::shared_ptr<poly_traj::Trajectory> replacement(new poly_traj::Trajectory(dura, cMats));
  const double entry_time = std::max(0.0, age);
  const Eigen::Vector3d entry_position = replacement->getPos(entry_time);
  if (!entry_position.allFinite() ||
      !replacement->getVel(entry_time).allFinite() ||
      !replacement->getAcc(entry_time).allFinite() ||
      !replacement->getPos(total_duration).allFinite() ||
      (receive_traj_ && (entry_position - last_pos_).norm() > max_start_jump_))
  {
    ROS_ERROR("[traj_server] Rejecting nonfinite or discontinuous replacement trajectory.");
    return;
  }
  traj_ = replacement;

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;
  if (!receive_traj_)
    last_pos_ = traj_->getPos(0.0);

  receive_traj_ = true;
  if (drone_id_ < 0)
    drone_id_ = msg->drone_id;
  heartbeat_paused_ = false;
  require_replan_ = false;
  execution_elapsed_ = entry_time;
  // start_time_ is the planner's absolute timestamp and is never shifted.
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void publish_execution(const ros::Time &now)
{
  const bool changed = execution_mode_ != last_published_mode_ ||
                       require_replan_ != last_published_replan_ ||
                       start_time_ != last_published_start_ ||
                       heartbeatFresh(now) != last_published_heartbeat_fresh_;
  if (!changed && !last_execution_publish_.isZero() &&
      (now - last_execution_publish_).toSec() < 0.05)
    return;
  traj_utils::TrajectoryExecution value;
  value.header.stamp = now;
  value.header.frame_id = "world";
  value.session = execution_session_;
  value.sequence = ++execution_sequence_;
  value.drone_id = drone_id_;
  value.trajectory_id = traj_id_;
  value.trajectory_start_time = start_time_;
  value.trajectory_duration = traj_duration_;
  value.elapsed = execution_elapsed_;
  for (int axis = 0; axis < 3; ++axis)
  {
    value.position[axis] = last_pos_[axis];
    value.velocity[axis] = last_vel_[axis];
    value.acceleration[axis] = last_acc_[axis];
  }
  value.mode = execution_mode_;
  value.require_replan = require_replan_;
  value.heartbeat_fresh = heartbeatFresh(now);
  execution_pub.publish(value);
  last_execution_publish_ = now;
  last_published_mode_ = execution_mode_;
  last_published_replan_ = require_replan_;
  last_published_start_ = start_time_;
  last_published_heartbeat_fresh_ = value.heartbeat_fresh;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
  last_vel_ = v;
  last_acc_ = a;
  publish_execution(cmd.header.stamp);
}

void cmdCallback(const ros::TimerEvent &e)
{
  ros::Time time_now = ros::Time::now();
  if (!receive_traj_)
  {
    // Readiness has no spatial meaning. A cold/restarted server must advertise
    // its session and live heartbeat without inventing a vehicle at the origin
    // or commanding a position before a planner supplies the first trajectory.
    if (drone_id_ >= 0)
    {
      execution_mode_ = traj_utils::TrajectoryExecution::WAITING;
      execution_elapsed_ = 0.0;
      require_replan_ = false;
      publish_execution(time_now);
    }
    return;
  }

  if (!heartbeatFresh(time_now))
    latchHold(time_now);
  if (heartbeat_paused_)
  {
    execution_mode_ = traj_utils::TrajectoryExecution::HOLDING;
    // Continue the command stream, but never resume the previous moving path
    // merely because heartbeat communication becomes available again.
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
    return;
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    execution_mode_ = traj_utils::TrajectoryExecution::RUNNING;
    execution_elapsed_ = t_cur;
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos,
                              std::max(1e-3, std::min(0.1, (time_now - time_last).toSec())));
    /*** calculate yaw ***/

    double tf = std::min(traj_duration_, t_cur + 2.0);
    pos_f = traj_->getPos(tf);

    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
  else if (t_cur >= traj_duration_)
  {
    execution_mode_ = traj_utils::TrajectoryExecution::FINISHED;
    execution_elapsed_ = traj_duration_;
    // Keep publishing a zero-velocity endpoint. Otherwise downstream odometry
    // simulators retain the final moving command after a trajectory expires.
    pos = traj_->getPos(traj_duration_);
    publish_cmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                last_yaw_, 0.0);
  }
  else
  {
    // A trajectory may start at most 50 ms in the future. Preserve the current
    // commanded position while waiting; report HOLDING without requesting a
    // new plan, since this is not a lost-heartbeat hold.
    execution_mode_ = traj_utils::TrajectoryExecution::HOLDING;
    execution_elapsed_ = 0.0;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0.0);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber hold_sub = nh.subscribe("hold", 1, holdCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  execution_pub = nh.advertise<traj_utils::TrajectoryExecution>("execution_state", 10);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("drone_id", drone_id_, -1);
  nh.param("heartbeat_timeout", heartbeat_timeout_, 0.5);
  nh.param("max_trajectory_age", max_message_age_, 0.3);
  nh.param("max_trajectory_start_jump", max_start_jump_, 0.3);
  if (!std::isfinite(heartbeat_timeout_) || heartbeat_timeout_ <= 0.0 ||
      !std::isfinite(max_message_age_) || max_message_age_ <= 0.0 ||
      !std::isfinite(max_start_jump_) || max_start_jump_ <= 0.0)
  {
    ROS_FATAL("[traj_server] Invalid execution freshness or continuity parameter.");
    return 1;
  }
  execution_session_ = ros::WallTime::now().toNSec();
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}
