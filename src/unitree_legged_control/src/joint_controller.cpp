/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_control/joint_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace unitree_legged_control 
{

    UnitreeJointController::UnitreeJointController(){
        memset(&lastCmd, 0, sizeof(unitree_legged_msgs::msg::MotorCmd));
        memset(&lastState, 0, sizeof(unitree_legged_msgs::msg::MotorState));
        memset(&servoCmd, 0, sizeof(ServoCmd));
    }

    UnitreeJointController::~UnitreeJointController(){
        sub_ft.reset();
        sub_cmd.reset();
    }

    void UnitreeJointController::setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        if(isHip) sensor_torque = msg->wrench.torque.x;
        else sensor_torque = msg->wrench.torque.y;
    }

    void UnitreeJointController::setCommandCB(const unitree_legged_msgs::msg::MotorCmd::SharedPtr msg)
    {
        lastCmd.mode = msg->mode;
        lastCmd.q = msg->q;
        lastCmd.Kp = msg->Kp;
        lastCmd.dq = msg->dq;
        lastCmd.Kd = msg->Kd;
        lastCmd.tau = msg->tau;
        command.writeFromNonRT(lastCmd);
    }

    bool UnitreeJointController::init(hardware_interface::EffortJointInterface *robot, rclcpp::Node::SharedPtr n)
    {
        isHip = false;
        isThigh = false;
        isCalf = false;
        sensor_torque = 0;
        name_space = n->get_namespace();
        if (!n->get_parameter("joint", joint_name)){
            RCLCPP_ERROR(n->get_logger(), "No joint given in namespace: '%s'", n->get_namespace());
            return false;
        }

        urdf::Model urdf;
        if (!urdf.initParam("robot_description")){
            RCLCPP_ERROR(n->get_logger(), "Failed to parse urdf file");
            return false;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf){
            RCLCPP_ERROR(n->get_logger(), "Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        if(joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" || joint_name == "RR_hip_joint" || joint_name == "RL_hip_joint"){
            isHip = true;
        }
        if(joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" || joint_name == "RR_calf_joint" || joint_name == "RL_calf_joint"){
            isCalf = true;
        }
        joint = robot->getHandle(joint_name);

        sub_ft = n->create_subscription<geometry_msgs::msg::WrenchStamped>(name_space + "/joint_wrench", 1, std::bind(&UnitreeJointController::setTorqueCB, this, std::placeholders::_1));
        sub_cmd = n->create_subscription<unitree_legged_msgs::msg::MotorCmd>("command", 20, std::bind(&UnitreeJointController::setCommandCB, this, std::placeholders::_1));

        controller_state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<unitree_legged_msgs::msg::MotorState>>(n, name_space + "/state", 1);        

        return true;
    }

    void UnitreeJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p, i, d, i_max, i_min, antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p, i, d, i_max, i_min, dummy);
    }

    void UnitreeJointController::starting(const rclcpp::Time& time)
    {
        double init_pos = joint.getPosition();
        lastCmd.q = init_pos;
        lastState.q = init_pos;
        lastCmd.dq = 0;
        lastState.dq = 0;
        lastCmd.tau = 0;
        lastState.tauEst = 0;
        command.initRT(lastCmd);
        pid_controller_.reset();
    }

    void UnitreeJointController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        double currentPos, currentVel, calcTorque;
        lastCmd = *(command.readFromRT());

        if(lastCmd.mode == PMSM) {
            servoCmd.pos = lastCmd.q;
            positionLimits(servoCmd.pos);
            servoCmd.posStiffness = lastCmd.Kp;
            if(fabs(lastCmd.q - PosStopF) < 0.00001){
                servoCmd.posStiffness = 0;
            }
            servoCmd.vel = lastCmd.dq;
            velocityLimits(servoCmd.vel);
            servoCmd.velStiffness = lastCmd.Kd;
            if(fabs(lastCmd.dq - VelStopF) < 0.00001){
                servoCmd.velStiffness = 0;
            }
            servoCmd.torque = lastCmd.tau;
            effortLimits(servoCmd.torque);
        }
        if(lastCmd.mode == BRAKE) {
            servoCmd.posStiffness = 0;
            servoCmd.vel = 0;
            servoCmd.velStiffness = 20;
            servoCmd.torque = 0;
            effortLimits(servoCmd.torque);
        }

        currentPos = joint.getPosition();
        currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.seconds());
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);      
        effortLimits(calcTorque);

        joint.setCommand(calcTorque);

        lastState.q = currentPos;
        lastState.dq = currentVel;
        lastState.tauEst = joint.getEffort();

        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.q = lastState.q;
            controller_state_publisher_->msg_.dq = lastState.dq;
            controller_state_publisher_->msg_.tauEst = lastState.tauEst;
            controller_state_publisher_->unlockAndPublish();
        }
    }

    void UnitreeJointController::stopping(){}

    void UnitreeJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void UnitreeJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void UnitreeJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerBase)
