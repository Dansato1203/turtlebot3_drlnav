#include <ignition/math.hh>
#include <stdio.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <random>

namespace gazebo
{
  class Obstacle2 : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    event::ConnectionPtr resetConnection;

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;

      // Connect to reset event
      this->resetConnection = event::Events::ConnectWorldReset(
          std::bind(&Obstacle2::OnWorldReset, this));
    }

    void OnWorldReset()
    {
      // Generate a new position
      ignition::math::Vector3d basePosition = GenerateRandomPosition();

      // Update the model's pose
      this->model->SetWorldPose(ignition::math::Pose3d(basePosition, ignition::math::Quaterniond(0, 0, 0)));

      // Set the animation starting from the new base position
      SetAnimation(basePosition);
      std::cout << "Animation reset to new base position: " << basePosition << std::endl;
    }

    ignition::math::Vector3d GenerateRandomPosition()
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> disX(1.5, 4.0);
      std::uniform_real_distribution<> disY(0, 0.35);

      double x = disX(gen);
      //double y = disY(gen);
      double y = -0.27;
      double z = 0;  // Assuming the object stays on the ground plane

      return ignition::math::Vector3d(x, y, z);
    }

    void SetAnimation(const ignition::math::Vector3d &basePosition)
    {
      gazebo::common::PoseAnimationPtr anim(
        new gazebo::common::PoseAnimation("move2", 160.0, true));

      std::vector<ignition::math::Vector3d> offsets = {
          {0.5, 0.0, 0.0}, {1.5, 0.0, 0.0}, {2.5, 0.0, 0.0}, {3.5, 0.0, 0.0},
          {4.5, 0.0, 0.0}, {6.5, 0.0, 0.0}, {7.0, 0.0, 0.0}, {8.0, 0.0, 0.0}
      };

      for (size_t i = 0; i < offsets.size(); ++i) {
          auto key = anim->CreateKeyFrame(i * 10.0);  // Example time steps
          ignition::math::Vector3d newPos = basePosition - offsets[i];
          key->Translation(newPos);
          key->Rotation(ignition::math::Quaterniond(0, 0, 0));
      }

      this->model->SetAnimation(anim);
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(Obstacle2)
}
