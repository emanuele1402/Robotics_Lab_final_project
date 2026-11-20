#include "kdl_planner.h"

/**
 * @brief [Default constructor].
 * 
  */
KDLPlanner::KDLPlanner(){}

/**
 * @brief [Planner Constructor if trapezoidal profile].
 * 
 * @param [_maxVel] [max vel of trapezoidal velocity profile].
 * @param [_maxAcc] [max acc of trapezoidal velocity profile].
 */
KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

/**
 * @brief [Planner Constructor if generic traj].
 * 
 * @param [] [].
 * @param
 * @param
 * @param
 */
KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

/**
 * @brief [Planner Constructor for a circular trajectory]
 * 
 * @param
 */
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius, double _accDuration)
{
  trajDuration_ = _trajDuration;
  accDuration_ = _accDuration;
  trajInit_ = _trajInit;
  trajRadius_ = _trajRadius;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

/**
 * @brief [Trapezoidal velocity profile].
 * 
 * [Trapezoidal velocity profile with given accDuration_ acceleration
 *  time period and trajDuration_ total duration that output the pos vel and acc
 *  at the time t].
 * 
 * @param [time] [current time].
 */
trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}

////////////////////////////
// TODO: HW functions     //
////////////////////////////

/**
 * @brief [trapezoidal velocity profile function].
 *
 * @param [t] [current time at which the curvilinear abscissa is computed].
 * @param [t_c] [cruise time].
 * @param [s] [curvilinear abscissa]
 * @param [s_dot] [Velocity at the current time]
 * @param [s_ddot] [acceleration at the current time]
 */
void KDLPlanner::trapezoidal_vel(double t, double t_c, double &s, double &s_dot, double &s_ddot)
{
  // Compute s_ddot_c
  double s_ddot_c = -1.0 / (std::pow(t_c,2) - trajDuration_*t_c);

  // Compute s, s_dot, s_ddot at current time t
  if (t <= t_c)
  {
    s = 0.5*s_ddot_c*std::pow(t,2);
    s_dot = s_ddot_c*t;
    s_ddot = s_ddot_c;
  }
  else if (t <= (trajDuration_ - t_c))
  {
    s = s_ddot_c*t_c*(t - t_c/2);
    s_dot = s_ddot_c*t_c;
    s_ddot = 0;
  }
  else
  {
    s = 1 - 0.5*s_ddot_c*std::pow(trajDuration_ - t, 2);
    s_dot = s_ddot_c*(trajDuration_ - t);
    s_ddot = -s_ddot_c;
  }
}

/**
 * @brief [cubic polynomial function]
 *
 * @param [t] [current time at which the curvilinear abscissa is computed].
 * @param [s] [curvilinear abscissa]
 * @param [s_dot] [velocity at the current time]
 * @param [s_ddot] [acceleration at the current time]
 */
void KDLPlanner::cubic_polynomial(double t, double &s, double &s_dot, double &s_ddot)
{
  // Polynomial coefficient
  double a_0 = 0;
  double a_1 = 0;
  double a_2 = 3 / std::pow(trajDuration_, 2);
  double a_3 = -2 / std::pow(trajDuration_,3);

  // Cubic profile
  s = a_3*std::pow(t,3) + a_2*std::pow(t,2) + a_1*t+ a_0;
  s_dot = 3*a_3*std::pow(t,2) + 2*a_2*t + a_1;
  s_ddot = 6*a_3*t + 2*a_2;
}

/**
 * @brief [circular trajectory function]
 * 
 * @param [s] [curvilinear abscissa]
 * @param [s_dot] [velocity at the current time]
 * @param [s_ddot] [acceleration at the current time]
 */
trajectory_point KDLPlanner::compute_circular_trajectory(double s, double s_dot, double s_ddot)
{
  trajectory_point traj;

  // position
  traj.pos(0) = trajInit_(0);
  traj.pos(1) = trajInit_(1) - trajRadius_*cos(2*M_PI*s) + trajRadius_;
  traj.pos(2) = trajInit_(2) - trajRadius_*sin(2*M_PI*s);

  // velocity
  traj.vel(0) = 0;
  traj.vel(1) = 2*M_PI*s_dot*trajRadius_*sin(2*M_PI*s);
  traj.vel(2) = -2*M_PI*s_dot*trajRadius_*cos(2*M_PI*s);

  // acceleration
  traj.acc(0) = 0;
  traj.acc(1) = 2*M_PI*trajRadius_*(s_ddot*sin(2*M_PI*s) + 2*M_PI*std::pow(s_dot,2)*cos(2*M_PI*s));
  traj.acc(2) = 2*M_PI*trajRadius_*(-s_ddot*cos(2*M_PI*s) + 2*M_PI*std::pow(s_dot,2)*sin(2*M_PI*s));

  return traj;
}

/**
 * @brief [linear trajectory function]
 * 
 * @param [s] [curvilinear abscissa]
 * @param [s_dot] [velocity at the current time]
 * @param [s_ddot] [acceleration at the current time]
 */
trajectory_point KDLPlanner::compute_linear_trajectory(double s, double s_dot, double s_ddot)
{
  trajectory_point traj;

  Eigen::Vector3d delta_p = trajEnd_ - trajInit_;

  // Point to poin interpolation
  traj.pos = trajInit_ + s*delta_p;
  traj.vel = s_dot*delta_p;
  traj.acc = s_ddot*delta_p;

  return traj;
}

/**
 * @brief [circular trajectory function with cubic profile]
 */
trajectory_point KDLPlanner::circular_traj_cubic(const double t)
{
  double s, s_dot, s_ddot;
  cubic_polynomial(t, s, s_dot, s_ddot);
  return compute_circular_trajectory(s, s_dot, s_ddot);
}

/**
 * @brief [circular trajectory function with trapezoidal proflie]
 */
trajectory_point KDLPlanner::circular_traj_trapezoidal(const double t)
{
  double s, s_dot, s_ddot;
  trapezoidal_vel(t, accDuration_, s, s_dot, s_ddot);
  return compute_circular_trajectory(s, s_dot, s_ddot);
}

/**
 * @brief [linear trajectory function with cubic profile]
 */
trajectory_point KDLPlanner::linear_traj_cubic(const double t)
{
  double s, s_dot, s_ddot;
  cubic_polynomial(t, s, s_dot, s_ddot);
  return compute_linear_trajectory(s, s_dot, s_ddot);
}

/**
 * @brief [linear trajectory function with trapezoidal profile]
 */
trajectory_point KDLPlanner::linear_traj_trapezoidal(const double t)
{
  double s, s_dot, s_ddot;
  trapezoidal_vel(t, accDuration_, s, s_dot, s_ddot);
  return compute_linear_trajectory(s, s_dot, s_ddot);
}

void KDLPlanner::setQuinticTrajectory(double _trajDuration, Eigen::Vector3d _trajInit, 
                                      Eigen::Vector3d _trajEnd, double _vInit, double _vEnd)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;

    double T = _trajDuration;
    double dist = (trajEnd_ - trajInit_).norm();
    if (dist < 1e-6) dist = 1e-6; 

    // Scaliamo le velocità lineari (m/s) in velocità dell'ascissa curvilinea (1/s)
    // s_dot = v / dist
    double s_dot_0 = _vInit / dist;
    double s_dot_f = _vEnd / dist;

    // Coefficienti per s(t) che va da 0 a 1
    // Vincoli: s(0)=0, s_dot(0)=s_dot_0, s_ddot(0)=0, s(T)=1, s_dot(T)=s_dot_f, s_ddot(T)=0
    Eigen::Matrix<double, 6, 6> A;
    A << 0, 0, 0, 0, 0, 1,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 2, 0, 0,
         pow(T,5), pow(T,4), pow(T,3), pow(T,2), T, 1,
         5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0,
         20*pow(T,3), 12*pow(T,2), 6*T, 2, 0, 0;

    Eigen::VectorXd b(6);
    b << 0.0, s_dot_0, 0.0, 1.0, s_dot_f, 0.0;

    quintic_coeffs_ = A.colPivHouseholderQr().solve(b);
}

trajectory_point KDLPlanner::getQuinticPoint(double t)
{
    if (t < 0) t = 0;
    if (t > trajDuration_) t = trajDuration_;

    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t3*t;
    double t5 = t4*t;

    double s = quintic_coeffs_(0)*t5 + quintic_coeffs_(1)*t4 + 
               quintic_coeffs_(2)*t3 + quintic_coeffs_(3)*t2 + 
               quintic_coeffs_(4)*t + quintic_coeffs_(5);

    double s_dot = 5*quintic_coeffs_(0)*t4 + 4*quintic_coeffs_(1)*t3 + 
                   3*quintic_coeffs_(2)*t2 + 2*quintic_coeffs_(3)*t + 
                   quintic_coeffs_(4);

    double s_ddot = 20*quintic_coeffs_(0)*t3 + 12*quintic_coeffs_(1)*t2 + 
                    6*quintic_coeffs_(2)*t + 2*quintic_coeffs_(3);

    // Ora passiamo s, s_dot, s_ddot (normalizzati) alla funzione lineare
    return compute_linear_trajectory(s, s_dot, s_ddot);
}
