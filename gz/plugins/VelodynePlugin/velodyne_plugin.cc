#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
    class VelodynePlugin : public ModelPlugin
    {

    public:
        VelodynePlugin() {}

    public:
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Safety check
            if (_model->GetJointCount() == 0)
            {
                std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
                return;
            }

            // Store the model pointer for convenience.
            this->model = _model;
            this->joint = _model->GetJoints()[0];
            this->pid = common::PID(0.1, 0, 0);
            this->model->GetJointController()->SetVelocityPID(
                this->joint->GetScopedName(), this->pid);

            // Default to zero velocity
            double velocity = 0;

            // Check that the velocity element exists, then read the value
            if (_sdf->HasElement("velocity"))
                velocity = _sdf->Get<double>("velocity");

            this->SetVelocity(velocity);
            this->node->Init(this->model->GetWorld()->Name());

            // Create a topic name
            std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

            // Subscribe to the topic, and register a callback
            this->sub = this->node->Subscribe(topicName,
                                              &VelodynePlugin::OnMsg, this);
        }

    public:
        void SetVelocity(const double &_vel)
        {
            this->model->GetJointController()->SetVelocityTarget(
                this->joint->GetScopedName(), _vel);
        }

    private:
        void OnMsg(ConstVector3dPtr &_msg)
        {
            this->SetVelocity(_msg->x());
        }

    private:
        transport::NodePtr node;
        transport::SubscriberPtr sub;
        physics::ModelPtr model;
        physics::JointPtr joint;
        common::PID pid;
    };
    GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}