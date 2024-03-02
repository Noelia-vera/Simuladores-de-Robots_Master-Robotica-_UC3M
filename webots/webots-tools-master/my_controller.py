"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()
FILE_NAME = "..\\..\\..\\..\\map1\\map1.csv" # Windows-style relative path, note the `\\`

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
### ds = robot.getDevice('dsname')
gps = robot.getDevice('gps')
imu = robot.getDevice('inertial unit')

### ds.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)

motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

END_X = 4
END_Y = 10
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



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    ### val = ds.getValue()
    gps_vals = gps.getValues()
    imu_rads = imu.getRollPitchYaw()
    # ## A nivel mapa, integramos la info que teníamos de start & end
    charMap[gps_vals,x][gps_vals,y] = '3' # 3: start
    charMap[END_X][END_Y] = '4' # 4: goal
    # # Grafo búsqueda
# ## Creamos el primer nodo
    init = Node(START_X, START_Y, 0, -2)
    init.dump() # comprobar que primer nodo bien

# ## `nodes` contendrá los nodos del grafo
    nodes = []

# ## Añadimos el primer nodo a `nodes`
    nodes.append(init)
    one = False  # clásica condición de parada del bucle `while`
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

    # Process sensor data here.
    inicio = time.time()
count_explored_nodes = 0
while not done:
    count_explored_nodes += greedy(nodes, charMap)
fin = time.time()
    # Enter here functions to send actuator commands, like:
    motor_left.setVelocity(-5.0)
    motor_right.setVelocity(-5.0)

    ### print('Hello World from Python!', gps_vals, [x*180.0/3.14159 for x in imu_rads])

# Enter here exit cleanup code.
print('Bye from Python!')