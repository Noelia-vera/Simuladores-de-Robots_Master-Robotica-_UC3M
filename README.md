# Simuladores-de-Robots_Master-Robotica-_UC3M
## _Master de Robótica y Automatización, Universidad Carlos 3 de Madrid_
### Simuladores de Robots - GYMNASIUM 
</p>

***
#### [ENLACE AL REPOSITORIO GITHUB ](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/tree/main/gymnasium)

</p>

***
#### INSTALACIONES PREVIAS:
Es necesario instalar las librerís para que funcione gymnasium

```bash
pip install time
pip install gymnasium[classic-control]
```

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
* **laberinto:** mapa con algunos obstáculos.
* **laberinto_p:** mapa con mayor numero de obstáculos.
* **goal_0.py:** algoritmo para llegar de una esquina a otra en el mapa 'laberinto_p'
* **goal_3.py:** algoritmo para llegar de un punto a otro en el mapa3 que es más grande.
* **goal_11.py:** algoritmo para llegar de un punto a otro en el mapa 11 que es mucho más grande y con más obstáculos.
* **laberinto.py:** algoritmo para llegar de una esquina a otra en un laberinto con muchos obstáculos.
* **q-learning_11.py** algoritmo de parendizaje por refuerzo que busca el camino para llegar de un punto a otro en el mapa 11.
* **q-learning_laberinto.py:** algoritmo de parendizaje por refuerzo que busca el camino para llegar de un punto a otro en el laberinto
* **MAP2-MAP11:** dados por el profesor en otra asignatura del master.

A modo de prueba y como su fuera la implementación de un juego, se ha realizado el siguiente archivo:
* **cartpole-display.py:** aqui hay un pédulo invertido que se mueve solo en el eje horizontal e intenta mantener el pédulo en vertical para que no se caiga. El método que se usa es el de q-learning

***
### INSTRUCCIONES DE EJECUCIÓN

1. El usuario solo debe ejecutar el archivo .py que quiera probar.

***
## RESULTADOS CON IMAGENES Y VIDEOS

### [GOAL_0.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/1.png)
En este codigo se implementa un método que va de una esquina a la opuesta del laberinto con obstáculos intermedios. En este caso lo importante es fijarse en los mensajes de la consola ya que va dando posiciones de las celdas. El refuerzo en todos los casos es nulo ya que no se choca contra ningun obstáculo.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/1.png">
</p>

[VIDEO](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/videos/gym1.mp4)

### [GOAL_3.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/2.png)
En este codigo se implementa un método que va de una esquina a la opuesta del laberinto con obstáculos intermedios. En este caso lo importante es fijarse en los mensajes de la consola ya que va dando posiciones de las celdas. El refuerzo en todos los casos es nulo ya que no se choca contra ningun obstáculo.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/2.png">
</p>

[VIDEO](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/videos/gym2.mp4)

### [GOAL_11.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/3.png)
En este codigo se implementa un método que va de un punto a otro del laberinto con obstáculos intermedios. En este caso lo importante es fijarse en los mensajes de la consola ya que va dando posiciones de las celdas. El refuerzo en todos los casos es nulo ya que no se choca contra ningun obstáculo.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/3.png">
</p>

[VIDEO](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/videos/gym3.mp4)

### [OBSTACLES.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/4.png)
En este codigo se implementa un método que va de una esquina a la opuesta del laberinto con obstáculos intermedios. Al contrario que ocurre antes, ahora en la pantalla se muestran observaciones y no posiciones. Los refuerzos siguen siendo nulos porque no se choca contra paredes.
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/4.png">
</p>

[VIDEO](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/videos/gym4.mp4)

### [CARTPOLE-DISPLAY.PY](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/6.png)
En este codigo se implementa el quilibrio de un péndulo donde el control solo se puede hacer moviendose en el eje horizontal. 
<p algin="center">
    <img src="https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/imagenes/6.png">
</p>

[VIDEO](https://github.com/Noelia-vera/Simuladores-de-Robots_Master-Robotica-_UC3M/blob/main/gymnasium/videos/gym6.mp4)

### ALGORITMOS Q-LEARNING.PY
En este caso se usa aprendizaje por refuerzo para encontrar el camino que lleva desde el punto de inicio al punto final. Para mapas pequeños es sencillo porque el numero de iteraciones se encuentra entre 100 y 200. Sin embargo, se ha probado el algortimo en los mapas 3, 11 y laberinto donde el área es mayor. La solución se termina encontrando pero el tiempo de computo es mucho ya que son necesarias muchas iteraciones, más de 800  todos los casos. 
***
