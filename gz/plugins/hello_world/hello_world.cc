#include <gazebo/gazebo.hh>

namespace gazebo
{
    class WorldPluginTutorial : public WorldPlugin
    {
    public:
        WorldPluginTutorial() : WorldPlugin()
        {
            printf("Hello World!\n");
            gzdbg << "debug" << std::endl;
            gzmsg << "message" << std::endl;
            gzwarn << "warning" << std::endl;
            gzerr << "error" << std::endl;
        }

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}