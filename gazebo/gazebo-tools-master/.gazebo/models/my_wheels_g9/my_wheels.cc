#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream> // Se agrega para incluir std::endl
#include <cmath>    // Se agrega para incluir M_PI
namespace gazebo
{

    class MyWheels : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            model = _model;

            if (!_sdf->HasElement("left_joint"))
                gzerr << "MyWheels plugin missing <left_joint> element\n";

            if (!_sdf->HasElement("right_joint"))
                gzerr << "MyWheels plugin missing <right_joint> element\n";

            leftJoint = _model->GetJoint(_sdf->GetElement("left_joint")->Get<std::string>());
            rightJoint = _model->GetJoint(_sdf->GetElement("right_joint")->Get<std::string>());

            if (!leftJoint)
                gzerr << "Unable to find left joint[" << _sdf->GetElement("left_joint")->Get<std::string>() << "]\n";
            if (!rightJoint)
                gzerr << "Unable to find right joint[" << _sdf->GetElement("right_joint")->Get<std::string>() << "]\n";

            // Set the final position
            initialPosition = model->WorldPose().Pos();
            finalPosition = ignition::math::Vector3d(16.5, 10.5, 0.0);
            initialHeading = model->WorldPose().Rot().Yaw();

            // Listen to the update event. This event is broadcast every simulation iteration.
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyWheels::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            // Phase 1: Girar en su lugar hasta alcanzar el ángulo deseado
            double angleToFinal = atan2(finalPosition.Y() - initialPosition.Y(), finalPosition.X() - initialPosition.X());
            double currentHeading = model->WorldPose().Rot().Yaw();
            double angleDifference = angleToFinal - currentHeading;

            if (fabs(angleDifference) > 0.01) {
                // Gira solo si no ha alcanzado el ángulo deseado
                leftJoint->SetVelocity(0, 1.0);
                rightJoint->SetVelocity(0, -1.0);
                double angleToFinalDegrees = angleToFinal * (180.0 / M_PI);
                std::cout << "Initial Position: " << initialPosition << std::endl;
                std::cout << "Final Position: " << finalPosition << std::endl;
                std::cout << "Angle to Final: " << angleToFinal << " radianes" << std::endl;
                std::cout << "Angle to Final: " << angleToFinalDegrees << " degrees" << std::endl;
            
                return; // Salir para no pasar a la siguiente fase
            }

            // Phase 2: Detener el movimiento de las ruedas
            leftJoint->SetVelocity(0, 0.0);
            rightJoint->SetVelocity(0, 0.0);

            // Phase 3: Avanzar recto hacia el punto final
            ignition::math::Vector3d currentPosition = model->WorldPose().Pos();
            double distanceToFinal = currentPosition.Distance(finalPosition);

            if (distanceToFinal > 0.1) {
                // Avanzar solo si no ha alcanzado el punto final
                double velocity = 8.0;
                leftJoint->SetVelocity(0, velocity);
                rightJoint->SetVelocity(0, velocity);
                std::cout << "Distance to Final: " << distanceToFinal << std::endl;
                return; // Salir para no repetir este proceso en la próxima actualización
            }
        }

    private:
        physics::ModelPtr model; // Pointer to the model
        physics::JointPtr leftJoint, rightJoint;
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        ignition::math::Vector3d finalPosition; // Declarar finalPosition como una variable
        ignition::math::Vector3d initialPosition;
        double initialHeading;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MyWheels)
}

