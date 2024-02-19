#Noelia Fernández Talavera
#Máster de Robotica y Automatización
#Simuladores de Robots

#En este codigo se implementa un método que pretende llevar a un robot desde su posición inicial a la esquina opuesta del mapa. Para ello
#el planteamiento que se ha hecho es una solución hidrida entre ir de manera directa de un punto a otro pero ir esquivando los obstaculos
#rodenadolos, como si fuera un algoritmo tipo bug.
#Inicialmente se calcula la orientación y distancia que hay desde su posición actual hasta la meta y se va dirigiendo hacia ella. Sin embargo, 
#también tiene implementados unos sensores de ultrasonidos incluidos en el propio robot para detectar las paredes e ir girando hacia el lado
#más libre, rodeando así las paredes hasta el objetivo.
#############################################################################################################################
#---------------------------------------------------INSTRUCCIONES DE FUNCIONAMIENTO------------------------------------------
#1º: El usuario debe indicar controlador quiere usar
#2º: Introducir las coordenadas de meta
#############################################################################################################################

#Lo primero es impertar las librerias necesarias para que todos los sensores funcionen

from controller import Robot, Motor, GPS, InertialUnit, DistanceSensor
import math

#En esta funcion se va a definir el funcionamiento del codigo.

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep()) #Se introdice el término para el tiempo
    MAX_SPEED = 6.28    #Se define la valocidad máxima del robot
    
    # Configuracion de los motores
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # Configuracion del GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    
    # Configuracion de la IMU
    imu = robot.getDevice('inertial unit')
    imu.enable(timestep)
    
    # Configurar sensores de distancia. En la imagen adjunta en el proyecto se pueden ver que sensores son los que se usan en este caso
    front_sensor = robot.getDevice('ps0')
    left_sensor = robot.getDevice('ps6')
    right_sensor = robot.getDevice('ps7')
    for sensor in [front_sensor, left_sensor, right_sensor]:
        sensor.enable(timestep)
        
    # Coordenadas de la meta (sustituir con las coordenadas reales)
    goal_position = [17.0, 10.0, 0.0]  # Ejemplo de coordenadas [x, y, z]
    
    # Variable para controlar si el robot sigue la ruta
    follow_route = False
    
    while robot.step(timestep) != -1:
        # Obtener las lecturas del GPS
        gps_values = gps.getValues()
        print("GPS values:", gps_values)  # Imprimir las coordenadas GPS

        # Obtener las lecturas del IMU
        imu_values = imu.getRollPitchYaw()
        current_angle = imu_values[2]  # Ángulo de orientación en el plano XY
        print("IMU values:", imu_values)  #Imprime los valores de IMU por pantalla

        
        # Calcular el ángulo hacia el objetivo utilizando la IMU
        target_angle = math.atan2(goal_position[1] - gps_values[1], goal_position[0] - gps_values[0])
        
        # Calcular el error de orientación (diferencia entre el ángulo hacia el objetivo y el ángulo actual)
        orientation_error = target_angle - current_angle
        
        # Ajustar la velocidad de los motores en función del error de orientación
        left_speed = MAX_SPEED - orientation_error
        right_speed = MAX_SPEED + orientation_error
        
        # Limitar las velocidades máximas
        left_speed = min(max(left_speed, 0), MAX_SPEED)
        right_speed = min(max(right_speed, 0), MAX_SPEED)
        
        # Establecer las velocidades de los motores
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        # Si se detecta un obstáculo en frente, girar y rodearlo
        if front_sensor.getValue() > 80:
            # Detener el robot
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            
            # Girar hacia la izquierda
            left_motor.setVelocity(-MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)
            
            # Esperar un tiempo para girar
            robot.step(1000)
            
            # Continuar avanzando
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)

        # Verificar si el robot ha llegado a la meta
        distance_to_goal = math.sqrt((gps_values[0] - goal_position[0])**2 +
                                     (gps_values[1] - goal_position[1])**2 +
                                     (gps_values[2] - goal_position[2])**2)
        if distance_to_goal < 0.1:
            print("Llegamos a la meta!")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break
        
#Llama a la funciñon creada para ejecutar el movimiento del robot
if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)
