#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{

    class ModelPush : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        void OnUpdate();
    };
}