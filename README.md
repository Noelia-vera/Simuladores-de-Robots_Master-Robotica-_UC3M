# Simuladores-de-Robots_Master-Robotica-_UC3M
## _Master de Robótica y Automatización, Universidad Carlos 3 de Madrid_
### Simuladores de Robots - WEBOTS 
</p>

***
#### [ENLACE AL REPOSITORIO GITHUB ](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M?tab=readme-ov-file)

</p>

***
#### INSTALACIONES PREVIAS:
Es necesario instalar las librerías para el contador de timpo y el ploteo de las gráficas.

pip install time
</p>
pip install matplotlib
</p>
pip3 install pygame

***
#### ORGANIZACIÓN DE CARPETAS:
* **controllers:** archivos .py que contienen los controladores para ejecutar en el robot
* **images:**  imágen del robot e-puck y de los mapas (12 posibilidades)
* **webots-tools-master:** imagenes de los mapas en .jpg, archivos de los mapas en formato .wbt, archivos .py para transformar un .csv a un .wbt y carpeta de assets con los .cvs que contienen los distintos mapas.
* **results:** Imágenes de los resultados obtenidos.

***
#### TIPOS DE ALGOTIRMO DESARROLLADOS Y ESCENARIOS:
* **ESQUINA:** algotimo que esquiva obstáculos y navega por el mapa tipo .wbt
* **BFS y ASTAR:**  algoritmos de path planning qye necesitan de un .csv para ejecutarse (incluidos).
* **LABERINTO Y MAP1:** se han creado estos dos escenarios propios con las indicaciones de dimensiones en función del nombre y obstáculos diseñados de manera propia.
* **MAP2-MAP11:** dados por el profesor en otra asignatura del master.

***
### INSTRUCCIONES DE EJECUCIÓN

1. El usuario debe indicar que controlador quiere usar, en caso de que sea el de esquina, no es necesario la lectura de un .csv. Sin embargo, si es un algoritmo de path planning, será necesario indicar el .csv que se corresponda con el mapa.
2. Introducir las coordenadas de meta.
3. Al iniciar la simulación aparece una ventana emergente con el mapa de los nodos explorados. Colocar con el ratón la ventana en una zona de la pantalla que permita ver la simulación, por ejemplo, encima del código.
4. darle al botón de "cerrar la pantalla emergente (X)" para que comience la simulación o sino el robot no se moverá. Tranquilo, el mapa con los nodos explorados no se irá de ahi.

***

### ESQUINA.PY
En este codigo se implementa un método que pretende llevar a un robot desde su posición inicial a la esquina opuesta del mapa. Para ello el planteamiento que se ha hecho es una solución hidrida entre ir de manera directa de un punto a otro pero esquivas a la vez los obstaculos rodenadolos, como si fuera un algoritmo tipo bug. Inicialmente se calcula la orientación y distancia que hay desde su posición actual hasta la meta y se va dirigiendo el robot hacia ella. Sin embargo, también tiene implementados unos sensores de ultrasonidos incluidos en el propio robot para detectar las paredes e ir girando hacia el lado más libre, rodeando así las paredes hasta el objetivo. Los sensores de ultrasonidos que tiene activos son los ps0, ps6 y ps7, además de un GPS y una IMU. En este codigo no se lee el .csv del mapa

### ESQUINA_CAMERA.PY
En este codigo se implementa un método que pretende llevar a un robot desde su posición inicial a la esquina opuesta del mapa. Para ello el planteamiento que se ha hecho es una solución hidrida entre ir de manera directa de un punto a otro pero esquivas a la vez los obstaculos rodenadolos, como si fuera un algoritmo tipo bug. Inicialmente se calcula la orientación y distancia que hay desde su posición actual hasta la meta y se va dirigiendo el robot hacia ella. Sin embargo, también tiene implementados unos sensores de ultrasonidos incluidos en el propio robot para detectar las paredes e ir girando hacia el lado más libre, rodeando así las paredes hasta el objetivo. Los sensores de ultrasonidos que tiene activos son los ps0, ps6 y ps7, además de un GPS y una IMU. En este codigo no se lee el .csv del mapa
La única diferencia con el algoritmo anterior es que aqui está activa la cámara del propio robot para ver desde su perspectiva el recorrido. En este caso, si se quiere que la velocidad de la simulación aumente no es posible por el procesamiento de la imagen de la cámara. 

### BFS_PLOTMAP.PY
Este código implementa el algoritmo breadth first search en el que se usa una heurística para calcular la distancia desde el nodo inical al final y buscar un camino greedy. Con este código no solo se puede ir a la  esquina opuesta del mapa sino a cualquier parte de este. Para ello se ha hecho uso de la libreria pygame para sacar por pantalla el mapa con los nodos explorados, el punto inical y el final. Tras obtenerse el mejor camino para llegar al destino, el robot e-puck, el cual lleva un GPS y una IMU para saber cual va siendo la posición y orientación del robot, va haciendo el trayecto siguiendo los nodos que llevan hasta el punto final.

### BFS_PLOTMAP_CAMERA.PY
Este código implementa el algoritmo breadth first search en el que se usa una heurística para calcular la distancia desde el nodo inical al final y buscar un camino greedy. Con este código no solo se puede ir a la  esquina opuesta del mapa sino a cualquier parte de este. Para ello se ha hecho uso de la libreria pygame para sacar por pantalla el mapa con los nodos explorados, el punto inical y el final. Tras obtenerse el mejor camino para llegar al destino, el robot e-puck, el cual lleva un GPS y una IMU para saber cual va siendo la posición y orientación del robot, va haciendo el trayecto siguiendo los nodos que llevan hasta el punto final. La única diferencia con el algoritmo anterior es que aqui está activa la cámara del propio robot para ver desde su perspectiva el recorrido. En este caso, si se quiere que la velocidad de la simulación aumente no es posible por el procesamiento de la imagen de la cámara.

### ASTAR_PLOTMAP.PY
Este código implementa el algoritmo ASTAR en el que se usa una heurística para calcular la distancia desde el nodo inical al final y buscar el camino más eficiente. Con este código no solo se puede ir a la  esquina opuesta del mapa sino a cualquier parte de este. Para ello se ha hecho uso de la libreria pygame para sacar por pantalla el mapa con los nodos explorados, el punto inical y el final. Tras obtenerse el mejor camino para llegar al destino, el robot e-puck, el cual lleva un GPS y una IMU para saber cual va siendo  la posición y orientación del robot, va haciendo el trayecto siguiendo los nodos que llevan hasta el punto final.

### ASTAR_PLOTMAP_CAMERA.PY
Este código implementa el algoritmo ASTAR en el que se usa una heurística para calcular la distancia desde el nodo inical al final y buscar el camino más eficiente. Con este código no solo se puede ir a la  esquina opuesta del mapa sino a cualquier parte de este. Para ello se ha hecho uso de la libreria pygame para sacar por pantalla el mapa con los nodos explorados, el punto inical y el final. Tras obtenerse el mejor camino para llegar al destino, el robot e-puck, el cual lleva un GPS y una IMU para saber cual va siendo  la posición y orientación del robot, va haciendo el trayecto siguiendo los nodos que llevan hasta el punto final.La única diferencia con el algoritmo anterior es que aqui está activa la cámara del propio robot para ver desde su perspectiva el recorrido. En este caso, si se quiere que la velocidad de la simulación aumente no es posible por el procesamiento de la imagen de la cámara.

***

### ROBOT E-PUCK
Esta imagen sirve para comprobar que sensores se han activado en cada .py programado
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/e-puck.png">
</p>

***

### EJEMPLO DE LOS RESULTADOS.

En este apartado aparecen algunos ejemplos de la visualización de los algotimos en distintos escenarios

#### 1. [ESQUINA](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>

#### 2. [BFS](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png">
</p>

#### 3. [A*](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png">
</p>

***

### MAPAS
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/laberinto.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map1.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map2.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map3.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map4.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map5.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map6.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map7.png">   
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map8.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map9.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map10.png">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/images/map11.png">

</p>

***

### EJEMPLO DE LOS RESULTADOS.

En este apartado aparecen algunos ejemplos de la visualización de los algotimos en distintos escenarios

#### 1. [ESQUINA](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>

#### 2. [BFS](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png">
</p>

#### 3. [A*](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png)

<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados_laberinto.png">
</p>