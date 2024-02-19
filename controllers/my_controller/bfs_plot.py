#Noelia Fernández Talavera
#Máster de Robotica y Automatización
#Introducción a la planificación de Robots

#Este código implementa el algoritmo un poco más greedy que el dado por el profesor
#en el que se escoge el último nodo de la lista de nodos para comenzar la busqueda a la meta

#Importamos la bibliotecas para contabilizar el tiempo y plotear
import time
import matplotlib.pyplot as plt
#! /usr/bin/env python

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
END_Y = 6


#Definimos coordenadas para plotear el gráfico de nodos explorados
x_coords = []
y_coords = []

# # Define Node class (A nivel grafo/nodo)
class Node:
    def __init__(self, x, y, myId, parentId):
        self.x = x
        self.y = y
        self.myId = myId
        self.parentId = parentId
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
# ## Creamos el primer nodo
init = Node(START_X, START_Y, 0, -2)
init.dump() # comprobar que primer nodo bien

# ## `nodes` contendrá los nodos del grafo
nodes = []

# ## Añadimos el primer nodo a `nodes`
nodes.append(init)


# ## Empieza algoritmo
#En este caso se va a programar el código como un algoritmo recursivo. Para ello se crea la 
#función greedy en la que se coge de la lista de nodos el último nodo y se va pasando por todas
#las direcciones.

done = False  # clásica condición de parada del bucle `while`
goalParentId = -1  # -1: parentId del nodo goal PROVISIONAL cuando aun no se ha resuelto
explored_nodes = 0

# Crear una lista para almacenar los nodos explorados y el camino a graficar
explored_node_coordinates = []
path_coordinates = []
def greedy(nodes, charMap):
    global goalParentId, done
    explored_nodes = 0
    print("--------------------- number of nodes: "+str(len(nodes)))
    #for node in nodes:
    print("Exploring nodes")
    node = nodes[-1]  # Obtener el último nodo de la lista
    dumpMap()

        # up
    tmpX = node.x - 1
    tmpY = node.y
    if( charMap[tmpX][tmpY] == '4' ):
        print("up: GOALLLL!!!")
        goalParentId = node.myId  # aquí sustituye por real
        done = True
    elif ( charMap[tmpX][tmpY] == '0' ) and not done:
        print("up: mark visited")
        explored_nodes += 1 #se suma uno al contador de nodos explorados
        newNode = Node(tmpX, tmpY, len(nodes), node.myId)
        charMap[tmpX][tmpY] = '2'
        nodes.append(newNode)
        explored_nodes += greedy(nodes, charMap)

        # down
    tmpX = node.x + 1
    tmpY = node.y
    if( charMap[tmpX][tmpY] == '4' ):
        print("down: GOALLLL!!!")
        goalParentId = node.myId # aquí sustituye por real
        done = True
    elif ( charMap[tmpX][tmpY] == '0' ) and not done:
        print("down: mark visited")
        explored_nodes += 1 #se suma uno al contador de nodos explorados
        newNode = Node(tmpX, tmpY, len(nodes), node.myId)
        charMap[tmpX][tmpY] = '2'
        nodes.append(newNode)
        explored_nodes += greedy(nodes, charMap)

        # right
    tmpX = node.x
    tmpY = node.y + 1
    if( charMap[tmpX][tmpY] == '4' ):
        print("right: GOALLLL!!!")
        goalParentId = node.myId # aquí sustituye por real
        done = True
    elif ( charMap[tmpX][tmpY] == '0' )and not done:
        print("right : mark visited")
        explored_nodes += 1 #se suma uno al contador de nodos explorados
        newNode = Node(tmpX, tmpY, len(nodes), node.myId)
        charMap[tmpX][tmpY] = '2'
        nodes.append(newNode)
        explored_nodes += greedy(nodes, charMap)


        # left
    tmpX = node.x
    tmpY = node.y - 1
    if( charMap[tmpX][tmpY] == '4' ):
        print("left: GOALLLL!!!")
        goalParentId = node.myId # aquí sustituye por real
        done = True
    elif charMap[tmpX][tmpY] == '0' and not done:
     print("up: mark visited")
     explored_nodes += 1 #se suma uno al contador de nodos explorados
     newNode = Node(tmpX, tmpY, len(nodes), node.myId)
     charMap[tmpX][tmpY] = '2'
     nodes.append(newNode)
     explored_nodes += greedy(nodes, charMap)
    return explored_nodes

print(explored_nodes)

#La clave del algoritmo esta en el siguiente while, en donde se inicializa la variable de contador
#de tiempo, y mientras no se encuentre la solución se recorre la función greedy anterior. Una vez 
#se encuentra la solución se pasa el contador del tiempo
inicio = time.time()
count_explored_nodes = 0
while not done:
    count_explored_nodes += greedy(nodes, charMap)
fin = time.time()
print("Nodos explorados:", count_explored_nodes)

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

# Anotar el inicio y el objetivo en el gráfico
plt.scatter(START_Y, START_X, color='green', marker='s', label='Start')
plt.scatter(END_Y, END_X, color='red', marker='s', label='Goal')

# Mostrar el gráfico
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
                print("%%%%%%%%%%%%%%%%%")
                ok = True
print("Tiempo de ejecución:", tiempo_transcurrido, "segundos")
print("Nodos explorados:", count_explored_nodes)