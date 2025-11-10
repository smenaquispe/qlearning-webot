from controller import Supervisor
import numpy as np
import random

# --- Parámetros de Q-Learning ---
alpha = 0.1        # tasa de aprendizaje
gamma = 0.9        # factor de descuento
epsilon = 0.1      # probabilidad de explorar
num_states = 4     # estados discretos (según sensores)
num_actions = 3    # [avanzar, girar izq, girar der]

# --- Inicializar tabla Q ---
Q = np.zeros((num_states, num_actions))

# --- Inicializar robot (Supervisor para acceder al objetivo) ---
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Motores
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Sensores de proximidad
ps = []
for i in range(8):
    sensor = robot.getDevice(f'ps{i}')
    sensor.enable(timestep)
    ps.append(sensor)

# Obtener nodos del robot y del objetivo
robot_node = robot.getSelf()
target = robot.getFromDef("TARGET")  # la esfera debe tener DEF TARGET

# --- Funciones auxiliares ---

def get_distance_to_target():
    """Distancia Euclidiana al objetivo"""
    robot_pos = robot_node.getPosition()
    target_pos = target.getPosition()
    return np.sqrt((robot_pos[0] - target_pos[0])**2 + (robot_pos[2] - target_pos[2])**2)

def reset_robot_position():
    """Reinicia la posición del robot al punto inicial"""
    trans_field = robot_node.getField("translation")
    trans_field.setSFVec3f([0, 0.05, 0])  # posición inicial
    robot.step(10 * timestep)

def get_state():
    """Discretiza el estado según los sensores frontales"""
    front_left = ps[7].getValue()
    front_right = ps[0].getValue()
    threshold = 80.0
    if front_left > threshold and front_right > threshold:
        return 0  # obstáculo al frente
    elif front_left > threshold:
        return 1  # obstáculo izquierda
    elif front_right > threshold:
        return 2  # obstáculo derecha
    else:
        return 3  # libre

def choose_action(state):
    """Estrategia ε-greedy"""
    if random.uniform(0, 1) < epsilon:
        return random.randint(0, num_actions - 1)
    else:
        return np.argmax(Q[state, :])

def execute_action(action):
    """Ejecuta la acción correspondiente"""
    if action == 0:  # avanzar
        left_motor.setVelocity(5)
        right_motor.setVelocity(5)
    elif action == 1:  # girar izquierda
        left_motor.setVelocity(-3)
        right_motor.setVelocity(3)
    elif action == 2:  # girar derecha
        left_motor.setVelocity(3)
        right_motor.setVelocity(-3)

def reverse_and_turn():
    """Retrocede y gira aleatoriamente para desatascarse"""
    left_motor.setVelocity(-4)
    right_motor.setVelocity(-4)
    for _ in range(10):
        robot.step(timestep)

    if random.random() < 0.5:
        left_motor.setVelocity(-3)
        right_motor.setVelocity(3)
    else:
        left_motor.setVelocity(3)
        right_motor.setVelocity(-3)

    for _ in range(15):
        robot.step(timestep)

def get_reward(state):
    """Asigna recompensa según estado y cercanía al objetivo"""
    dist = get_distance_to_target()

    # Recompensa grande si llega al objetivo
    if dist < 0.1:
        print("🎯 ¡Objetivo alcanzado!")
        return 100

    # Penalizaciones y recompensas básicas
    if state == 0:  # choca de frente
        return -10
    elif state in [1, 2]:  # cerca de obstáculo
        return -2
    else:
        # recompensa mayor cuanto más cerca esté del objetivo
        return 2 + (1.0 / (dist + 0.1))

# --- Bucle principal ---
while robot.step(timestep) != -1:
    s = get_state()
    a = choose_action(s)
    execute_action(a)

    # Esperar un poco para moverse
    robot.step(5 * timestep)

    s_next = get_state()
    r = get_reward(s_next)

    # Si está frente a un obstáculo, retrocede
    if s_next == 0:
        reverse_and_turn()

    # Actualizar Q
    Q[s, a] = Q[s, a] + alpha * (r + gamma * np.max(Q[s_next, :]) - Q[s, a])

    # Si llegó al objetivo, reiniciar episodio
    if get_distance_to_target() < 0.1:
        reset_robot_position()

    # Mostrar progreso
    print(f"Estado: {s}, Acción: {a}, Recompensa: {r:.2f}, Distancia objetivo: {get_distance_to_target():.2f}")
