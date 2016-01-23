/**
 * Kondo ICS & B3M motor driver
 */
#include <boost/shared_ptr.hpp>
#include <math.h>
#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/actuator_command_interface.h"
#include "hardware_interface/actuator_state_interface.h"
#include "hardware_interface/robot_hw.h"
extern "C" {
#include "kondo_driver/ics.h"
#include "kondo_driver/b3m.h"
}
#include "kondo_driver/setPower.h"

/* Maximum motor num (32 is maximum on spec. sheet) */
const int MAX_MOTOR_NUM = 12;
const int MAX_PULSE = 11500;
const int MIN_PULSE = 3500;
const int CNT_PULSE = 7500;
const double RADIAN_PER_PULSE = 270.0*M_PI/(MAX_PULSE-MIN_PULSE)/180.0;
const double RAD_TO_DEG = 180.0/M_PI*100;

double pulse_to_radian (double pulse)
{
    return (pulse - CNT_PULSE)*RADIAN_PER_PULSE;
}

int radian_to_pulse (double radian)
{
    return CNT_PULSE + radian/RADIAN_PER_PULSE;
}

int deg100_to_radian (double deg100)
{
    return deg100/RAD_TO_DEG;
}

int radian_to_deg100 (double radian)
{
    return radian*RAD_TO_DEG;
}

class KondoMotor {
private:
    bool loopback;
    bool motor_power;
    ros::ServiceServer power_service;
    int id;
    ICSData* ics;
	B3MData* b3m;
    int stretch;
    int speed;
    int curr_limit; // current limit
    int temp_limit;
    int min_angle, max_angle;
public:
    double cmd, pos, vel, eff;
    std::string joint_name;
    
	bool set_power (kondo_driver::setPower::Request &req, kondo_driver::setPower::Response &res) {
		ROS_INFO("id %d, request: %d", this->id, req.request);
		motor_power = req.request;
		res.result = req.request;
		return true;
    }
    
	KondoMotor (B3MData* b3m, std::string actuator_name, hardware_interface::JointStateInterface& state_interface, hardware_interface::PositionJointInterface& pos_interface, bool loopback=false) : cmd(0), pos(0), vel(0), eff(0) {
		motor_power = true;
		this->loopback = loopback;
		this->b3m = b3m;
		ros::NodeHandle nh(std::string("~")+actuator_name);
		if (nh.getParam("id", id)) {
			ROS_INFO("id: %d", id);
		}
		if (!loopback) {
			// Check motor existence
			if (b3m_servo_mode(b3m, id, B3M_OPTIONS_RUN_NORMAL) <= 0) {
			ROS_WARN("Cannot connect to servo ID: %d", id);
			}
		}
		if (nh.getParam("joint_name", joint_name)) {
			ROS_INFO("joint_name: %s", joint_name.c_str());
		}
		if (nh.getParam("min_angle", min_angle)) {
			ROS_INFO("min_angle: %d", min_angle);
		}
		if (nh.getParam("max_angle", max_angle)) {
			ROS_INFO("max_angle: %d", max_angle);
		}
		if (nh.getParam("stretch", stretch)) {
			ROS_INFO("stretch: %d", stretch);
			set_stretch(stretch);
		}
		if (nh.getParam("speed", speed)) {
			ROS_INFO("speed: %d", speed);
			set_speed(speed);
		}
		if (nh.getParam("current_limit", curr_limit)) {
			ROS_INFO("current_limit: %d", curr_limit);
			set_current_limit(curr_limit);
		}
		if (nh.getParam("temperature_limit", temp_limit)) {
			ROS_INFO("temperature_limit: %d", temp_limit);
			set_temperature_limit(temp_limit);
		}
		hardware_interface::JointStateHandle state_handle(joint_name, &pos, &vel, &eff);
		state_interface.registerHandle(state_handle);
		hardware_interface::JointHandle pos_handle(state_interface.getHandle(joint_name), &cmd);
		pos_interface.registerHandle(pos_handle);
		power_service = nh.advertiseService(actuator_name+std::string("/set_power"), &KondoMotor::set_power, this);
    }

    void update (void) {
		int pulse_cmd = 0;
		if (cmd < min_angle*3.14/180) {
			cmd = min_angle;
		}
		if (cmd > max_angle*3.14/180) {
			cmd = max_angle;
		}
//		if (motor_power == true) {
			pulse_cmd = radian_to_deg100(cmd);
//		}
		if (loopback) {
			pos = cmd;
			eff = 0;
		}else{
			int pulse_ret = 0;
			pulse_ret= b3m_set_angle(b3m, id, pulse_cmd);
			if (pulse_ret > 0) {
				pos = deg100_to_radian (pulse_ret);
			}
			/* how can I get speed ? */
			vel = 0;
			/* get servo current */
			int current;
			b3m_get_current(b3m, id, &current);
			if (current > 0) {
				if (current < 64) {
					eff = current;
				} else {
					eff = - (current - 64);
				}
			}
		}
    }

    // Set speed parameter
    void set_speed (unsigned char speed) {
		if (loopback) {
			this->speed = speed;
		}else {
			this->speed = b3m_set_speed(b3m, id, speed);
			ROS_INFO("%s: %d", __func__, this->speed);
		}
	}

	// Set strech parameter
	void set_stretch (unsigned char stretch) {
		if (loopback) {
			this->stretch = stretch;
		}else {
			this->stretch = b3m_set_stretch(b3m, id, stretch);
			ROS_INFO("%s: %d", __func__, this->stretch);
		}
    }

    // Set current limit 
    void set_current_limit (unsigned char curr) {
		if (loopback) {
			curr_limit = curr;
		}else {
			this->curr_limit = b3m_set_stretch(b3m, id, curr);
			ROS_INFO("%s: %d", __func__, this->curr_limit);
		}
		}
		// Set temperature limit
		void set_temperature_limit (unsigned char temp) {
		if (loopback) {
			temp_limit = temp;
		}else {
			this->temp_limit = b3m_set_temperature_limit(b3m, id, temp);
			ROS_INFO("%s: %d", __func__, this->temp_limit);
		}
    }
};

class KondoDriver : public hardware_interface::RobotHW
{
  private:
    bool loopback;
    // Hardware interface
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    // B3M hardware resource
    B3MData b3m;
    // Vector of motors
    std::vector<boost::shared_ptr<KondoMotor> > actuator_vector;
  public:
    KondoDriver (int num, char** actuators) {
		ros::NodeHandle nh("~");
		std::string serial_port;
		nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
		ROS_INFO("serial_port: %s", serial_port.c_str());
		
		loopback = false;
		if (nh.getParam("loopback", loopback)) {
			if (loopback) {
			ROS_WARN("loopback mode: the hardware is not used.");
			}
		}
		if (!loopback) {
			// Initiallize B3M interface
			if (b3m_init(&b3m, serial_port.c_str()) < 0) {
				ROS_ERROR ("Could not init B3M: %s\n", b3m.error);
				exit(0);
			}
		}
		// Load atuators
		for (int i=0; i<num; i++) {
			boost::shared_ptr<KondoMotor> actuator(new KondoMotor(&b3m, std::string(actuators[i]), jnt_state_interface, jnt_pos_interface, loopback));
			actuator_vector.push_back(actuator);
		}
		registerInterface(&jnt_state_interface);
		registerInterface(&jnt_pos_interface);
    }

    // Read & write function
    void update () {
		for (int i=0; i<actuator_vector.size(); i++) {
		  //ROS_INFO("%s: %d", __func__, i);
			actuator_vector[i]->update();
		}
    }

    ~KondoDriver () {
		if (!loopback) {
			b3m_close (&b3m);
		}
    }
    ros::Time getTime() const {return ros::Time::now();}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kondo_driver");
    ros::NodeHandle nh;

    // Create hardware interface 
    KondoDriver robot(argc-1, &argv[1]);
    // Connect to controller manager
    controller_manager::ControllerManager cm(&robot, nh);

    ros::Rate rate(1.0 / robot.getPeriod().toSec());
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()){
		cm.update(robot.getTime(), robot.getPeriod());
		robot.update();
		rate.sleep();
    }
    spinner.stop();

    return 0;
}


#if 0

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

int main(int argc, char** argv)
{
  using namespace transmission_interface;

  // Raw data
  double a_pos;
  double j_pos;

  // Transmission
  SimpleTransmission trans(10.0); // 10x reducer

  // Wrap raw data
  ActuatorData a_data;
  a_data.position.push_back(&a_pos);

  JointData j_data;
  j_data.position.push_back(&j_pos);

  // Transmission interface
  ActuatorToJointPositionInterface act_to_jnt_pos;
  act_to_jnt_pos.registerHandle(ActuatorToJointPositionHandle("trans", &trans, a_data, j_data));

  // Propagate actuator position to joint space
  act_to_jnt_pos.propagate();
}
#endif
