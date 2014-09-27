#include <rbmt_base_ctrl/rbmt_base_kinematics.h>

namespace rbmt_base_ctrl {

BaseKinematics::BaseKinematics() {}

BaseKinematics::~BaseKinematics() {}

bool BaseKinematics::init() {
	const size_t n_wheel = 3;// The base of RBMT has 3 omniwheels
	const std::string base_frame_id = "link_base";

	wheels.resize(n_wheel);
	for (size_t i=0; i<wheels.size(); ++i) wheels.at(i).init();
}

Wheel::Wheel() {}

Wheel::~Wheel() {}

bool Wheel::init() {
	// TODO @tttor: fix these values
	joint_name = "wheel_joint";
	link_name = "wheel_link";
	wheel_radius = 0.005;
	wheel_speed_actual = 0.0;
	wheel_speed_cmd = 0.0;
	wheel_speed_error = 0.0;
	direction_multiplier = 1.0;
}
	
}// namespace rbmt_base_ctrl