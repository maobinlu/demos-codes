#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class MoveObjPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MoveObjPlugin::OnUpdate, this));

      //获取当前的位置 
      pose0 = this->model->WorldPose();
      vel = 0.1;
    }

    public: void OnUpdate()
    {
      double time_now = this->model->GetWorld()->SimTime().Double();
      ignition::math::Pose3d pose = this->model->WorldPose();
      if ((pose.Pos() - pose0.Pos()).Length() > 1.0) {
        vel = -vel;
      }
      // 设置线速度和角速度
      // this->model->SetLinearVel(ignition::math::Vector3d(0.0, sin(time_now*0.5), 0.0));
      this->model->SetLinearVel(ignition::math::Vector3d(0.0, vel, 0.0));
      this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;

    ignition::math::Pose3d pose0;
    double vel;
  };

  GZ_REGISTER_MODEL_PLUGIN(MoveObjPlugin)
}
