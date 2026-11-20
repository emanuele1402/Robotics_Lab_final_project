#include "kdl_control.h"
#include "utils.h"
KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

}


Eigen::VectorXd KDLController::velocityCtrlNull(const Vector6d &cartvel,
                                                const Eigen::MatrixXd &J,
                                                const Eigen::VectorXd &q)
{
    int n = q.size();
    // Usa la pseudoinversa con damping che abbiamo modificato in utils.h
    Eigen::MatrixXd J_pinv = pseudoinverse(J); 

    Eigen::VectorXd q_task = J_pinv * cartvel;

    Eigen::MatrixXd jnt_limits = robot_->getJntLimits();
    Eigen::VectorXd q0dot(n);
    q0dot.setZero();

    const double EPS = 0.05; // Margine leggermente più ampio per sicurezza

    for (int i = 0; i < n; ++i) {
        double qi = q(i);
        double qmin = jnt_limits(i, 0);
        double qmax = jnt_limits(i, 1);
        
        // Calcolo distanze con clamp di sicurezza
        double d_max = std::max(EPS, qmax - qi);
        double d_min = std::max(EPS, qi - qmin);

        double numer = (qmax + qmin - 2 * qi);
        // Usiamo le distanze clippate per il denominatore
        double denom = std::pow(d_max * d_min, 2);
        
        // Calcolo gradiente
        double gain = 0.1; // Inizia con un lambda molto basso per lo spazio nullo
        q0dot(i) = gain * (std::pow(qmax - qmin, 2) * numer / denom);
        
        // Saturation: impedisce allo spazio nullo di "sparare" velocità folli
        q0dot(i) = std::max(-1.0, std::min(1.0, q0dot(i)));
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd null_space_term = (I - J_pinv * J) * q0dot;

    return q_task + null_space_term;
}



