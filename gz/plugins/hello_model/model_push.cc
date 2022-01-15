#include "model_push.hh"

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
    void ModelPush::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    void ModelPush::OnUpdate()
    {
        // Apply a small linear velocity to the model in X axis.
        this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    }
};