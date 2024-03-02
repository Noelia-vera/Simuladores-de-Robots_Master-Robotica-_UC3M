#Noelia Fernández Talavera
#Máster de Robotica y Automatización
#Introducción a la planificación de Robots

#Este código implementa el algoritmo ASTAR en el que se usa una heurística para calcular la distancia
#desde el nodo inical al final y buscar el camino más eficiente


#! /usr/bin/env python
#Importamos la bibliotecas para contabilizar el tiempo y plotear
import time
import matplotlib.pyplot as plt
"""
# Notactión
## Mapa
En mapa original:
* 0: libre
* 1: ocupado (muro/obstáculo)
Vía código incorporamos:
* 2: visitado
* 3: start
* 4: goal
## Nodo
Nós
* -2: parentId del nodo start
* -1: parentId del nodo goal PROVISIONAL cuando aun no se ha resuelto
# Específico de implementación Python
* Índices empiezan en 0
* charMap
"""

# # Initial values are hard-coded (A nivel mapa)

#FILE_NAME = "/usr/local/share/master-ipr/map1/map1.csv" # Linux-style absolute path
#FILE_NAME = "C:\\Users\\USER_NAME\\Downloads\\master-ipr\\map1\\map1.csv" # Windows-style absolute path, note the `\\` and edit `USER_NAME`
#FILE_NAME = "../../../../map1/map1.csv" # Linux-style relative path
FILE_NAME = "C:\\Users\\noeli\\Desktop\\webots-tools-master\\controllers\\bfs\\map1.csv" # Windows-style relative path, note the `\\`
START_X = 2
START_Y = 2
END_X = 4
END_Y = 5

#Definimos coordenadas para plotear el gráfico de nodos explorados
x_coords = []
y_coords = []

# # Define Node class (A nivel grafo/nodo)
class Node:
    def __init__(self, x, y, myId, parentId, f):
        self.x = x
        self.y = y
        self.myId = myId
        self.parentId = parentId
        self.f = f
    def dump(self):
        print("---------- x "+str(self.x)+\
                         " | y "+str(self.y)+\
                         " | id "+str(self.myId)+\
                         " | parentId "+str(self.parentId))

# # Mapa
# ## Creamos estructura de datos para mapa
charMap = []

# ## Creamos función para volcar estructura de datos para mapa
def dumpMap():
    for line in charMap:
        print(line)

# ## De fichero, llenar estructura de datos de fichero (`to parse`/`parsing``) para mapa
with open(FILE_NAME) as f:
    line = f.readline()
    while line:
        charLine = line.strip().split(',')
        charMap.append(charLine)
        line = f.readline()

# ## A nivel mapa, integramos la info que teníamos de start & end
charMap[START_X][START_Y] = '3' # 3: start
charMap[END_X][END_Y] = '4' # 4: goal

# ## Volcamos mapa por consola
dumpMap()

# # Grafo búsqueda

# ## Para crear un algotirmo más greedy, hay que poner en valor lo que cuestan las decisiones. En este caso, para conseguir un algoritmo A star 
# se calcula la distancia desde el punto de inicio al destino. Se puede usar cualquier tipo de heurística que se haya visto en clase.
distance = abs(START_X-END_X) + abs(START_Y-END_Y)  #Cálculo de la distancia desde el nodo inicial al final
init = Node(START_X, START_Y, 0, -2, distance) #el nodo inicial tiene en cuenta la distancia desde el nodo de inicio hasta el nodo final
init.dump() # comprobar que primer nodo bien

# ## `nodes` contendrá los nodos del grafo
nodes = []

# ## Añadimos el primer nodo a `nodes`
nodes.append(init)

# ## Empieza algoritmo
done = False  # clásica condición de parada del bucle `while`
goalParentId = -1  # -1: parentId del nodo goal PROVISIONAL cuando aun no se ha resuelto
# Inicio del tiempo
inicio = time.time()

while not done:
    explored_nodes = 0  #Se incluye una variable que sirva para saber cuantos nodos se han explorado, 
    #de esta forma habrá una variable que sirva de comparativa entre los distintos métodos
    print("--------------------- number of nodes: "+str(len(nodes)))
    for node in nodes:
        node.dump()
        explored_nodes += 1 #Se suman los nodos explorados a medida que avanza el algoritmo
#se crean dos distancias, una que sirva para calcular lo que cuesta llegar hasta el nodo que se esta explorando y otra que calcule desde el nodo explorado al final
        # up
        tmpX = node.x - 1
        tmpY = node.y
        if( charMap[tmpX][tmpY] == '4' ):
            print("up: GOALLLL!!!")
            goalParentId = node.myId  # aquí sustituye por real
            done = True
            break
        elif ( charMap[tmpX][tmpY] == '0' ):
            print("up: mark visited")
            distance1 = abs(START_X-tmpX) + abs(START_Y-tmpY) #Lo que cuesta llegar hasta el nodo actual
            distance2 = abs(tmpX-END_X) + abs(tmpY-END_Y) #Lo que cuesta llegar hasta el final
            newNode = Node(tmpX, tmpY, len(nodes), node.myId, distance1 + distance2) #aqui se tienen en cuenta las dos variables nuevas generadas
            charMap[tmpX][tmpY] = '2'
            nodes.append(newNode)

        # down
        tmpX = node.x + 1
        tmpY = node.y
        if( charMap[tmpX][tmpY] == '4' ):
            print("down: GOALLLL!!!")
            goalParentId = node.myId # aquí sustituye por real
            done = True
            break
        elif ( charMap[tmpX][tmpY] == '0' ):
            print("down: mark visited")
            distance1 = abs(START_X-tmpX) + abs(START_Y-tmpY) #Lo que cuesta llegar hasta el nodo actual
            distance2 = abs(tmpX-END_X) + abs(tmpY-END_Y) #Lo que cuesta llegar hasta el final
            newNode = Node(tmpX, tmpY, len(nodes), node.myId, distance1 + distance2) #aqui se tienen en cuenta las dos variables nuevas generadas
            charMap[tmpX][tmpY] = '2'
            nodes.append(newNode)

        # right
        tmpX = node.x
        tmpY = node.y + 1
        if( charMap[tmpX][tmpY] == '4' ):
            print("right: GOALLLL!!!")
            goalParentId = node.myId # aquí sustituye por real
            done = True
            break
        elif ( charMap[tmpX][tmpY] == '0' ):
            print("right : mark visited")
            distance1 = abs(START_X-tmpX) + abs(START_Y-tmpY) #Lo que cuesta llegar hasta el nodo actual
            distance2 = abs(tmpX-END_X) + abs(tmpY-END_Y) #Lo que cuesta llegar hasta el final
            newNode = Node(tmpX, tmpY, len(nodes), node.myId, distance1 + distance2) #aqui se tienen en cuenta las dos variables nuevas generadas
            charMap[tmpX][tmpY] = '2'
            nodes.append(newNode)

        # left
        tmpX = node.x
        tmpY = node.y - 1
        if( charMap[tmpX][tmpY] == '4' ):
            print("left: GOALLLL!!!")
            goalParentId = node.myId # aquí sustituye por real
            done = True
            break
        elif ( charMap[tmpX][tmpY] == '0' ):
            print("left: mark visited")
            distance1 = abs(START_X-tmpX) + abs(START_Y-tmpY) #Lo que cuesta llegar hasta el nodo actual
            distance2 = abs(tmpX-END_X) + abs(tmpY-END_Y) #Lo que cuesta llegar hasta el final
            newNode = Node(tmpX, tmpY, len(nodes), node.myId, distance1 + distance2) #aqui se tienen en cuenta las dos variables nuevas generadas
            charMap[tmpX][tmpY] = '2'
            nodes.append(newNode)
        nodes.sort (key = lambda x:x.f, reverse = True)

        dumpMap()
    print("Nodos explorados:", explored_nodes) #Saca por pantalla el número de nodos explorados
# Se para de contar el tiempo
fin = time.time()
#Cálculo del tiempo de ejecución
tiempo_transcurrido = fin - inicio


# Crear una lista para almacenar los nodos a graficar
nodes_to_plot = nodes  # Copiamos la lista de nodos

# Crear un gráfico de dispersión con los valores x e y de los nodos
for node in nodes_to_plot:
    x_coords.append(node.x)
    y_coords.append(node.y)

# Graficar los nodos
plt.scatter(x_coords, y_coords, label='Nodes', color='blue', marker='o')

# Etiquetas y leyenda del gráfico
plt.xlabel('Coordenada X')
plt.ylabel('Coordenada Y')
plt.title('Gráfico de nodos')

# Se incluye el inicio y el objetivo en el gráfico
plt.scatter(START_Y, START_X, color='green', marker='s', label='Start')
plt.scatter(END_Y, END_X, color='red', marker='s', label='Goal')

# Se muestra el gráfico
plt.legend()
plt.show()


# ## Display solución hallada
print("%%%%%%%%%%%%%%%%%%%")
ok = False
while not ok:
    for node in nodes:
        if( node.myId == goalParentId ):
            node.dump()
            goalParentId = node.parentId
            if( goalParentId == -2):
                print("%%%%%%%%%%%%%%%%")
                ok = True
print("Tiempo de ejecución:", tiempo_transcurrido, "segundos")
print("Nodos explorados:", explored_nodes)

