#Noelia Fernández Talavera
#Máster de Robotica y Automatización
#Simuladores de Robots

#Este código implementa el algoritmo ASTAR en el que se usa una heurística para calcular la distancia
#desde el nodo inical al final y buscar el camino más eficiente. Con este código no solo se puede ir a la 
#esquina opuesta del mapa sino a cualquier parte de este.
# Para ello se ha hecho uso de la libreria pygame para sacar por pantalla el mapa con los nodos explorados, el punto inical y el final.
#Tras obtenerse el mejor camino para llegar al destino, el robot e-puck, el cual lleva un GPS y una IMU para saber cual va siendo 
#la posición y orientación del robot, va haciendo el trayecto siguiendo los nodos que llevan hasta el punto final.

#Se va a comentar el codigo de manera detallada a lo largo de este.


#############################################################################################################################
#---------------------------------------------------INSTRUCCIONES DE FUNCIONAMIENTO------------------------------------------
#1º: El usuario debe indicar que .csv quiere leer para ir de un punto a otro, introduciendo las coordenadas de meta.
#2º: al iniciar la simulación aparece una ventana emergente con el mapa de los nodos explorados. Colocar con el ratón la ventana en una
#zona de la pantalla que permita ver la simulación, por ejemplo, encima del código.
#3º: darle al botón de "cerrar la pantalla emergente (X)" para que comience la simulación o sino el robot no se moverá. 
#Tranquilo, el mapa con los nodos explorados no se irá de ahi.

#############################################################################################################################

#Se importan las librerias necesarias y los elementos que hacen que el controlador funcione, en este caso son:
#Robot, Motor, GPS, InertialUnit, DistanceSensor

import time
from controller import Robot, Motor, GPS, InertialUnit, DistanceSensor
import math
import pygame

# Create the Robot instance.
#ES MUY IMPORTANTE MODIFICAR ESTA LINEA PARA INDICAR EL .CSV QUE SE DESEEA LEER EN CADA MOMENTO. EL USUARIO DEBE CAMBIARLO
FILE_NAME = "laberinto.csv"  # Windows-style relative path

# Get the time step of the current world.
robot = Robot()

#Librerias de tiempo, matemáticas para el calculo de distancias y el ploteo del mapa
timestep = int(robot.getBasicTimeStep())
import time
import matplotlib.pyplot as plt

#Coordenadas de inicio y fin
START_X = 1
START_Y = 1
END_X = 2
END_Y = 6


#Definimos coordenadas para plotear el gráfico de nodos explorados
x_coords = []
y_coords = []

# Define Node class (A nivel grafo/nodo)
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

 ## Para crear un algotirmo más greedy, hay que poner en valor lo que cuestan las decisiones. En este caso, para conseguir un algoritmo A star 
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
    
    
######################################################################
#-----------------------------PINTAR EL MAPA------------------------------
#Aqui comienza el modulo para sacar por pantalla el mapa y los nodos explorados
# initialize the pygame module
pygame.init()

pygame.display.set_caption("A STAR")

# Crea una superficie en la pantalla de 240 x 180
screen_width = 600
screen_height = 600
screen = pygame.display.set_mode((screen_width,screen_height))
    
# Define las variables que se controlan en el main loop
running = True

# Int Map, se inicializa le mapa
map_rows = len(charMap)
map_cols = len(charMap[0])

intMap = []

for i in range(map_rows):
    temp_row =[]
    for j in range(map_cols):
        temp_row.append(int(charMap[i][j]))
    intMap.append(temp_row)


cell_width = screen_width/map_cols
cell_height = screen_height/map_rows

size_coef = min(cell_height, cell_height)/60

# Se crean los rectángulos
rectangles = []
occupied = []
color_ocuppied = (16, 25, 155)
color_free = (255, 255, 255)
color_border = (164, 169, 247)

#Se le asigna el color a las casillas
for i in range(map_rows):
    for j in range(map_cols):
        rectangles.append(pygame.Rect(j*cell_width, i*cell_height, cell_width, cell_height))
        occupied.append(intMap[i][j])


start_color = (0, 0, 255)
start_center = (cell_width*(START_Y+0.5), cell_height*(START_X+0.5))
start_radius = (cell_width*0.2)

end_color = (251, 103, 0)
end_center = (cell_width*(END_Y+0.5), cell_height*(END_X+0.5))
end_radius = (cell_width*0.2)

# Se dibujan unas cruces en los nodos que han sido explorados

font = pygame.font.Font(None, round(60*size_coef))
cross = font.render("x", True, (104, 139, 240))

# Se dibujan las lineas

prev_coor = ((END_Y+0.5)*cell_width, (END_X+0.5)*cell_height)
coordinates = [prev_coor]

ok = False
    
# main loop

# Variables para el desplazamiento de la vista
offset_x = 0
offset_y = 0
dragging = False
prev_mouse_pos = None

# Bucle principal. Este bucle ha sido creado para poder mover la pantalla del mapa en la pantalla ya que, una vez que se inicia el proceso de movimiento
#del robot, la pantalla no se puede mover. Se define que los botones del ratón son los que mueven la ventana emergente
while running:
    # Manejo de eventos
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                dragging = True
                prev_mouse_pos = pygame.mouse.get_pos()
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                dragging = False
        elif event.type == pygame.MOUSEMOTION:
            if dragging:
                mouse_pos = pygame.mouse.get_pos()
                dx = mouse_pos[0] - prev_mouse_pos[0]
                dy = mouse_pos[1] - prev_mouse_pos[1]
                offset_x += dx
                offset_y += dy
                prev_mouse_pos = mouse_pos

    # Dibujar la pantalla
    screen.fill((0, 0, 0))
    

    for i in range(len(rectangles)):
        
        if occupied[i] == 1: color = color_ocuppied
        else: color = color_free

        pygame.draw.rect(screen, color, rectangles[i])
        pygame.draw.rect(screen, color_border, rectangles[i], round(2*size_coef))

    for i in range(map_rows):
        for j in range(map_cols):
            if intMap[i][j] == 2:
                
                cross_x, cross_y = (j+0.5)*cell_width, (i+0.5)*cell_height

                text_rect = cross.get_rect()
                text_rect.center = (cross_x, cross_y)
                
                screen.blit(cross, text_rect)     
    

    # Punto de inicio
    pygame.draw.circle(screen, start_color, start_center, start_radius)

    # Punto final al que se quiere llegar
    pygame.draw.circle(screen, end_color, end_center, end_radius)
    
    # Dibujo del path
    for i in range(len(coordinates)):
        if i>0:
            pygame.draw.line(screen, end_color, coordinates[i], coordinates[i-1], round(4*size_coef))

    pygame.display.flip()  # <--- Update the display
    
ok = False


# se genera el Path que va a realizar el robot con el camino final del inicio al fin
path=[]
while not ok:
    for node in nodes:
        if( node.myId == goalParentId ):
            node.dump()
            path.append((node.x,node.y))
            goalParentId = node.parentId
            if( goalParentId == -2):
                print("%%%%%%%%%%%%%%%%%")
                ok = True
print(path) 

for i in range(len(coordinates)):
    if i>0:
        pygame.draw.line(screen, end_color, coordinates[i], coordinates[i-1], round(4*size_coef))

pygame.display.flip()

# ## Display solución hallada
print("%%%%%%%%%%%%%%%%%%%")

path.reverse()  #Se le da la vuelta al path ya que hay que recorrerlo del inicio al fin y esta ordenado del final al principio.

#################################################################################################################
#Aqui comienza la lógica para mover el robot en función del path final generado

#Se defien la velocidad maxima del robot
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28

# Configurar motores
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(MAX_SPEED)
right_motor.setVelocity(MAX_SPEED)

# Configurar GPS
gps = robot.getDevice('gps')
gps.enable(timestep)

# Configurar IMU
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# Coordenadas del vector a seguir (sustituir con las coordenadas reales)
coordinates_vector = path

# Variable para controlar el índice del objetivo actual
current_goal_index = 0

while robot.step(timestep) != -1:
    # Obtener las lecturas del GPS
    gps_values = gps.getValues()
    print("GPS values:", gps_values)  # Imprimir las coordenadas GPS
    
    # Obtener las lecturas del IMU
    imu_values = imu.getRollPitchYaw()
    current_angle = imu_values[2]  # Ángulo de orientación en el plano XY
    
    # Obtener las coordenadas del objetivo actual
    goal_position = coordinates_vector[current_goal_index]
    print(goal_position)
    # Calcular el ángulo hacia el objetivo utilizando la IMU
    target_angle = math.atan2((goal_position[1]+0.5) - gps_values[1], (goal_position[0]+0.5) - gps_values[0])
    
    # Calcular el error de orientación (diferencia entre el ángulo hacia el objetivo y el ángulo actual)
    orientation_error = target_angle - current_angle
    if abs(orientation_error) < math.radians(0.5):
        left_speed = 1
        right_speed = 1
    else:
        left_speed = -1
        right_speed = 1
    
    print(orientation_error)
  
    # Establecer las velocidades de los motores
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Verificar si el robot ha llegado al objetivo actual. Saca un mensaje por pantalla cuando el robot llega a la meta.
    distance_to_goal = math.sqrt((gps_values[0] - (goal_position[0]+0.5))**2 +
                                 (gps_values[1] - (goal_position[1]+0.5))**2)
    if distance_to_goal < 0.1:
        print("Llegamos al objetivo:", goal_position)
        current_goal_index += 1
        if current_goal_index >= len(coordinates_vector):
            print("Se alcanzaron todos los objetivos.")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            break
print(distance_to_goal)