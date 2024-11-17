#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <random>

namespace gazebo
{
  class RandomPositionOnSimStart : public ModelPlugin
  {
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;

      // Connect to the update event
      this->updateConnection = event::Events::ConnectWorldReset(
          std::bind(&RandomPositionOnSimStart::OnUpdate, this));
    }

    void OnUpdate()
    {
      // Get the simulation time
      double simTime = this->model->GetWorld()->SimTime().Double();

      // Check if the simulation time is near zero
      if (simTime < 1.0) {  // Adjust this threshold as needed
        // Generate a random position
        ignition::math::Vector3d newPosition = this->GenerateRandomPosition();
        // Set the model to the new random position
        this->model->SetWorldPose(ignition::math::Pose3d(newPosition, ignition::math::Quaterniond(0, 0, 0)));
      }
    }

    ignition::math::Vector3d GenerateRandomPosition()
    {
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> disX(-4, -2.5); // Customize your range for x
      std::uniform_real_distribution<> disY(0, 0.35);  // Customize your range for y

      double x = disX(gen);
      //double y = disY(gen);
      double y = 0.27;
      double z = 0;  // Assuming the object stays on the ground plane

      return ignition::math::Vector3d(x, y, z);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RandomPositionOnSimStart)
}
