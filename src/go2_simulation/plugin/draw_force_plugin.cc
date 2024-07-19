/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include <boost/bind.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace gazebo
{
    class UnitreeDrawForcePlugin : public VisualPlugin
    {
        public:
        UnitreeDrawForcePlugin():line(NULL){}
        ~UnitreeDrawForcePlugin(){
            this->visual->DeleteDynamicLine(this->line);
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
        {
            this->visual = _parent;
            this->visual_namespace = "visual/";
            if (!_sdf->HasElement("topicName")){
                RCLCPP_INFO(rclcpp::get_logger("UnitreeDrawForcePlugin"), "Force draw plugin missing <topicName>, defaults to /default_force_draw");
                this->topic_name = "/default_force_draw";
            } else{
                this->topic_name = _sdf->Get<std::string>("topicName");
            }
            if (!rclcpp::ok()){
                rclcpp::init(0, nullptr);
            }

            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);

#if GAZEBO_MAJOR_VERSION >= 10
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), ignition::math::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), ignition::math::Color(0, 1, 0, 1.0));
#else
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), common::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), common::Color(0, 1, 0, 1.0));
#endif

            this->line->setMaterial("Gazebo/Purple");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            this->rosnode = rclcpp::Node::make_shared("unitree_draw_force_plugin");
            this->force_sub = this->rosnode->create_subscription<geometry_msgs::msg::WrenchStamped>(
                this->topic_name+"/the_force", 30, std::bind(&UnitreeDrawForcePlugin::GetForceCallback, this, std::placeholders::_1));
            this->update_connection = event::Events::ConnectPreRender(std::bind(&UnitreeDrawForcePlugin::OnUpdate, this));
            RCLCPP_INFO(this->rosnode->get_logger(), "Load %s Draw Force plugin.", this->topic_name.c_str());
        }

        void OnUpdate()
        {
            this->line->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
        }

        void GetForceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {
            Fx = msg->wrench.force.x / 20.0;
            Fy = msg->wrench.force.y / 20.0;
            Fz = msg->wrench.force.z / 20.0;
            // Fx = msg.wrench.force.x;
            // Fy = msg.wrench.force.y;
            // Fz = msg.wrench.force.z;
        }

        private:
            rclcpp::Node::SharedPtr rosnode;
            std::string topic_name;
            rendering::VisualPtr visual;
            rendering::DynamicLines *line;
            std::string visual_namespace;
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub;
            double Fx=0, Fy=0, Fz=0;
            event::ConnectionPtr update_connection;
    };

    GZ_REGISTER_VISUAL_PLUGIN(UnitreeDrawForcePlugin)
}
