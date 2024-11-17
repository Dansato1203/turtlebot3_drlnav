#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // モデルを保存
      this->model = _model;
    }

    public: void SetPosition(const ignition::math::Vector3d &pos)
    {
      // モデルの新しい位置を設定
      model->SetWorldPose(ignition::math::Pose3d(pos, ignition::math::Quaterniond(1, 0, 0, 0)));
    }

    private: physics::ModelPtr model;
  };
  // プラグインを登録
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
