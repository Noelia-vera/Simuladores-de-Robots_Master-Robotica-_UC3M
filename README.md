# Simuladores-de-Robots_Master-Robotica-_UC3M
## _Master de Robótica y Automatización, Universidad Carlos 3 de Madrid_
### Simuladores de Robots - GYMNASIUM 
</p>

***
#### [ENLACE AL REPOSITORIO GITHUB ](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M?tab=readme-ov-file)

</p>

***
#### INSTALACIONES PREVIAS:
Es necesario instalar las librerís para que funcione gymnasium

pip install time
</p>
pip install gymnasium[classic-control]

***
#### ORGANIZACIÓN DE CARPETAS:
* **assests:** distintos mapas con laberintos para probar el simulador.
* **examples:**  archivos .py para conseguir llegar desde el inicio del laberinto hasta la esquina opuesta.
* **gymnasium_csv:** configuración del simulador.
* **imagenes:** Imágenes de los resultados obtenidos.
* **more examples:** ejemplos de quepeós videojuegos
* **videos:** videos con los resultados obtenidos

***
#### TIPOS DE ALGOTIRMO DESARROLLADOS Y ESCENARIOS:
En esta parte se va a presentar las cosas nuevas que se han hecho para esta práctica
* **laberinto:** mapa con algunos obstáculos
* **laberinto_p:** mapa con mayor numero de obstáculos
* **goal_0.py:** algoritmo para llegar de una esquina a otra en el mapa 'laberinto_p'
* **goal_3.py:** algoritmo para llegar de un punto a otro en el mapa3 que es más grande
* **goal_11.py:** algoritmo para llegar de un punto a otro en el mapa 11 que es mucho más grande y con más obstáculos
* **laberinto.py:** algoritmo para llegar de una esquina a otra en un laberinto con muchos obstáculos.
* **q-learning_11.py** algoritmo de parendizaje por refuerzo que busca el camino para llegar de un punto a otro en el mapa 11.
* **q-learning_laberinto.py:** algoritmo de parendizaje por refuerzo que busca el camino para llegar de un punto a otro en el laberinto
* **MAP2-MAP11:** dados por el profesor en otra asignatura del master.

Da modo de prueba y como su fuera la implementación de un juego, se ha realizado el siguiente archivo:
* **cartpole-display.py:** aqui hay un pédulo invertido que se mueve solo en el eje horizontal e intenta mantener el pédulo en vertical para que no se caiga.

***
### INSTRUCCIONES DE EJECUCIÓN

1. El usuario solo debe ejecutar el archivo .py que quiera probar.

***
## RESULTADOS CON IMAGENES Y VIDEOS

### [GOAL_0.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png)
En este codigo se implementa un método que va de una esquina a la opuesta del laberinto con obstáculos intermedios.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>

### [GOAL_3.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png)
En este codigo se implementa un método que va de una esquina a la opuesta del laberinto con obstáculos intermedios.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>

### [GOAL_11.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png)
En este codigo se implementa un método que va de un punto a otro del laberinto con obstáculos intermedios.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>

### [OBSTACLES.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png)
En este codigo se implementa un método que va de una esquina a la opuesta del laberinto con obstáculos intermedios.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>

### [CARTPOLR-DISPLAY.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png)
En este codigo se implementa el quilibrio de un péndulo donde el control solo se puede hacer moviendose en el eje horizontal.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/resultados/resultados10.png">
</p>

***
