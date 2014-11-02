#include <rbmt_base_ctrl/rbmt_base_kinematics.h>

namespace rbmt_base_ctrl {

BaseKinematics::BaseKinematics() {}

BaseKinematics::~BaseKinematics() {}

bool BaseKinematics::init() {
	//
	n_wheel = 3;// The base of RBMT has 3 omniwheels
	frame_id = "link_base";

	wheels.resize(n_wheel);
	for (size_t i=0; i<wheels.size(); ++i) 
		wheels.at(i).init();

	// F
	// J_1_f is calculated using the matrix elements of the rolling constraints for the Swedish 90-degree wheel. 
  Eigen::MatrixXd tmp_J_1f(3,3);
  tmp_J_1f << sqrt(3.0)/2.0, -1.0/2.0, -1.0,
             0.0        ,  1.0    , -1.0,
          -sqrt(3.0)/2.0, -1.0/2.0, -1.0;
  J_1f = tmp_J_1f;

  // J_2 --> diagonal matrix of wheel radius; J_2 = diag(r1, r2, ..., rn)
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> tmp_J_2(n_wheel);
  for (size_t i=0; i<n_wheel; ++i) {
  	tmp_J_2.diagonal()[i] = wheels.at(i).wheel_radius;
  }
  J_2 = tmp_J_2;
}

Wheel::Wheel() {}

Wheel::~Wheel() {}

bool Wheel::init() {
	// TODO @tttor: fix these values
	joint_name = "wheel_joint";
	link_name = "wheel_link";
	// wheel radius 10cm
  wheel_radius = 0.010;
	wheel_speed_actual = 0.0;
	wheel_speed_cmd = 0.0;
	wheel_speed_error = 0.0;
	direction_multiplier = 1.0;
}
	
}// namespace rbmt_base_ctrl