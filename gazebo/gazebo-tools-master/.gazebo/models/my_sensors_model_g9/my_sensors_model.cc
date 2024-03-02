#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <iostream> // Se agrega para incluir std::endl
#include <cmath>    // Se agrega para incluir M_PI

namespace gazebo
{

    class MySensorsModel : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            int sensor_num = _model->GetSensorCount();
            model = _model;
            
            sensor = sensors::get_sensor("my_sensor");

            if (!sensor)
            {
                gzerr << "the plugin requires a LaserSensor.\n";
                return;
            }
            
            sensor->SetActive(true);

            gzdbg << "Opened " << sensor->ScopedName() << "\n";

            raySensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
            if (!raySensor)
            {
                gzerr << "dynamic_pointer_cast to RaySensor failed!\n";
                return;
            }
            // this block of parameters is obtained from gazebo
            gzdbg << "AngleMax [deg] " << raySensor->AngleMax().Degree() << "\n";
            gzdbg << "AngleMin [deg] " << raySensor->AngleMin().Degree() << "\n";
            gzdbg << "RangeMax [m] " << raySensor->RangeMax() << "\n";
            gzdbg << "RangeMin [m] " << raySensor->RangeMin() << "\n";
            gzdbg << "AngleResolution [deg] " << raySensor->AngleResolution() * 180.0 / M_PI << "\n";
            gzdbg << "RangeCount " << raySensor->RangeCount() << "\n";
            gzdbg << "UpdateRate " << raySensor->UpdateRate() << "\n";
            
            
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
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MySensorsModel::OnUpdate, this));
        }

        // Called by the world update start event
        void OnUpdate()
        {
            std::vector<double> ranges;
            raySensor->Ranges(ranges);
            //gzdbg << "ranges.size() " << ranges.size() << "\n"; // RangeCount
            gzdbg << std::accumulate(ranges.begin(), ranges.end(), 0.0) / ranges.size() << "\n";
            
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
        sensors::RaySensorPtr raySensor;
        sensors::SensorPtr sensor;             // Pointer to the sensor
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        ignition::math::Vector3d finalPosition; // Declarar finalPosition como una variable
        ignition::math::Vector3d initialPosition;
        double initialHeading;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MySensorsModel)
}
