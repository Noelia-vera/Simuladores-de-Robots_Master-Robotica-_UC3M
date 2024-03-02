#include <gazebo/gazebo.hh> // Se debe incluir esta cabecera de Gazebo

namespace gazebo // Debe ser parte del namespace gazebo
{
    class WorldPluginTutorial : public WorldPlugin // Como ejemplo básico, heredar de WorldPlugin
    {
    public:
        WorldPluginTutorial() : WorldPlugin() // Empty constructor (se refiere a la ausencia de argumentos)
        {
            printf("¡Hola mundo!\n");
        }
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) // Recibe un puntero al mundo y a SDFs
        {
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial) // Necesario registrar el plugin
}
