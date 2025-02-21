#include <Eigen/Core>

Eigen::Vector3f forward_dynamics(
    const Eigen::Vector3f& q,
    const Eigen::Vector3f& qd,
    const Eigen::Vector3f& tau,
    const bool is_left);

Eigen::Vector3f inverse_dynamics(
    const Eigen::Vector3f& q,
    const Eigen::Vector3f& dq,
    const Eigen::Vector3f& ddq,
    const bool is_left);
