# Simuladores-de-Robots_Master-Robotica-_UC3M
## _Master de Robótica y Automatización, Universidad Carlos 3 de Madrid_
### Simuladores de Robots - GAZEBO 
</p>

***
#### [ENLACE AL REPOSITORIO GITHUB ](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M?tab=readme-ov-file)

</p>

***
#### INSTALACIONES PREVIAS:
Install dependencies
```bash
sudo -H pip install -r requirements.txt
```


***
#### ORGANIZACIÓN DE CARPETAS:
* **webots:** practica de simuladores usando webots
* **gazebo:**  practica de simuladores usando gazebo
***

#### ORGANIZACIÓN DE CARPETAS DENTO DE GAZEBO:
* **.gazebo:** Es donde se encuentran los modelos de los robots. Dado que algunos tienen implementados sensores y otros no, se han creado varios modelos del mismo robot para incorporarles diferentes plugins de movimiento. Dentro de la carpeta modelos se encuentra el poineer sin sensores, el pioneer 2, y el pioneer_sensor que tiene movimiento y sensor de distancia.
* **assests:**  diferentes mundos en formato .csv.
* ***pioneer:** modelo del robot usado en las simulaciones.
* ***world:** mundos con los distintos funcionamientos de los robots.
* ***gazebo-map-from-csv.py:** .py que permite convertir cualquier mundo a .world.xml.
* ***Imagenes y videos:** demostraciones de los funcionamientos de los mundos.

***
#### TIPOS DE ALGOTIRMO DESARROLLADOS Y MUNDOS:

Para realizar esta practica se han genreado archivos de mundo tipo .world.xml. En el código se introduce el modelo del robot que se quiere usar y en el modelo se implementan los plugins que contienen la configuración de ejecución del robot.

* **OBSTACLES.WORLD.XML:** En este mundo se encuentra un robot en la equina superior izquierda que navega hasta el esquina contraria del escenario en el que se encuentra.
* **OBSTACLES_2.WORLD.XML:**  En este mundo hay dos robots, uno en cada esquina superior, que navegan hasta las esquinas inferiores contrarias cruzándose por el camino. 
* **OBSTACLES_KINECT.WORLD.XML:** En este mundo esta incluido un robot con el modelo de la cámara kinect.
* **lABERINTO_P.WORLD.XML:** En este mundo se ha incluido un robot con movimiento esquivando obstáculos y un sensor de distancia mediante un laser.
* **MAP0-MAP11.WORLD.XML:** Distintos laberintos de diferentes geometías y áreas.

***
### INSTRUCCIONES DE EJECUCIÓN

1.  El usuario debe navegar dentro de la carpeta de los mundos y lanzar por terminal el plugin que quiere usar y el mundo.
```bash
GAZEBO_PLUGIN_PATH=/ruta/a/plugin/build gazebo --verbose --pause map.world.xml
```
2. En el caso de que se quieran ejecitar dos plugins diferentes de un mismo mundo porque hay dos robots incluidos, la forma de hacerlo es la siguiente:
```bash
GAZEBO_PLUGIN_PATH=/ruta/a/plugin1/build:/ruta/a/plugin2/build  gazebo --verbose --pause map.world.xml
```
3. Disfrutar de la ejecución dándole al play en Gazebo.
4. En caso de querer ejejutar otros plugins es necesario ejecutarlos de manera previa.

***

### ROBOT E-PUCK
Esta imagen sirve para comprobar que sensores se han activado en cada .py programado
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/e-puck.png">
</p>

***

### EJEMPLO DE LOS RESULTADOS.

En este apartado aparecen algunos ejemplos de la visualización de los algotimos en distintos escenarios

#### 1. [OBSTACLES.WORLD.XML](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/esquina.mp4)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>
[VIDEO](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/esquina.mp4)

#### 2. [OBSTACLES_2.WORLD.XML](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png">
</p>

[![VIDEO](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/esquina_doble.mp4)](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/esquina_doble.mp4)

#### 3. [KINECT](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/kinect.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/kinect.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/kinect_2.png">

</p>

#### 4. [LABERINTO_P](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/sensor.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gazebo/gazebo-tools-master/Imagenes%20y%20videos/sensor.png">
</p>
***

***