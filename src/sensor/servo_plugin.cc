#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

namespace gazebo
{
  class ServoPlugin : public ModelPlugin
  {
    public: 
      ServoPlugin() : ModelPlugin() 
      {
        this->vel = 0.0;
        this->rot = 0.0;
      }

      ~ServoPlugin() {}

    public:
  
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {

          this->model = _model;
          this->world = _model->GetWorld();
          this->time = this->world->SimTime().Double();

          std::cout << "Initializing ServoPlugin..." << std::endl;

          std::string topic = "/cmd_servo";

          // Acquire servo joint from sdf
          if (_sdf->HasElement("joint"))
          {
            std::string joint = _sdf->Get<std::string>("joint");
            this->servo = this->model->GetJoint(joint);
          } else {
            std::cerr << "ServolLugin: Joint element not found in SDF." << std::endl;
            return;
          }

          if (_sdf->HasElement("topic"))
          {
            topic = _sdf->Get<std::string>("topic");
          }

          if (_sdf->HasElement("parent_link"))
          {
            this->parent_link = _sdf->Get<std::string>("parent_link");
          }

          if (_sdf->HasElement("child_link"))
          {
            this->child_link = _sdf->Get<std::string>("child_link");
          }

          // ROS initialization
          if (!rclcpp::ok())
          {
            //RCLCPP_FATAL_STREAM("No ROS node for Gazebo has been initialized!");
            return;
          }

          rclcpp::NodeOptions options;
          options.parameter_overrides({rclcpp::Parameter("use_sim_time", false)});

          this->node = rclcpp::Node::make_shared("gz_servo", options);
          this->subscription = node->create_subscription<std_msgs::msg::String>(
            topic, 1,
            std::bind(&ServoPlugin::OnMsg, this, std::placeholders::_1)
          );

          this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this->node);

          this->executor_thread = std::thread([this]() { rclcpp::spin(this->node); });

          RCLCPP_INFO(this->node->get_logger(), "Initialized ServoPlugin");

          // Link with update event, so servo velocity remains constant
          this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ServoPlugin::OnUpdate, this)
          );
        }

      void OnUpdate()
      {
        // Update current rotation estimate
        double curr_time = this->world->SimTime().Double();
        double dt = curr_time - this->time;
        double d_theta = dt*this->vel;

        this->rot += d_theta;        
        this->time = curr_time;

        this->servo->SetVelocity(0, this->vel);

        // Publish the transform info
        // TODO way to make this not so expensive? Like static transform?

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->node->get_clock()->now();
        t.header.frame_id = this->parent_link.c_str();
        t.child_frame_id  = this->child_link.c_str();

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, this->rot);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        this->tf_broadcaster->sendTransform(t);
      }

      void OnMsg(const std_msgs::msg::String::SharedPtr msg)
      {
        if (msg->data[0] == '0')
          { this->vel = 0.0;}
        else if (msg->data[0] == '1')
          { this->vel = 1.0;}
        else if (msg->data[0] == '2')
          { this->vel = -1.0;}

        this->servo->SetVelocity(0, this->vel);
      }

    private:
      event::ConnectionPtr updateConnection;
      physics::ModelPtr model;
      physics::JointPtr servo;
      physics::WorldPtr world;
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
      rclcpp::Node::SharedPtr node;
      std::thread executor_thread;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
      std::string parent_link;
      std::string child_link;
      double time;
      double vel;
      double rot;

  };
  GZ_REGISTER_MODEL_PLUGIN(ServoPlugin)
}