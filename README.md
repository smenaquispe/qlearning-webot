# Q-Learning Robot Navigation - Webots

Proyecto de aprendizaje por refuerzo utilizando Q-Learning para que un robot e-puck navegue y alcance un objetivo evitando obstáculos.

## Descripción del Proyecto

Este proyecto implementa un agente de Q-Learning que controla un robot e-puck en el simulador Webots. El robot aprende a navegar en un entorno con obstáculos para alcanzar una esfera objetivo (verde).

## Componentes Principales

### 1. Controlador Q-Learning (`qlearning_controller.py`)

El controlador implementa el algoritmo de Q-Learning con las siguientes características:

**Parámetros de aprendizaje:**

- **α (alpha):** 0.1 - Tasa de aprendizaje
- **γ (gamma):** 0.9 - Factor de descuento
- **ε (epsilon):** 0.1 - Probabilidad de exploración (estrategia ε-greedy)

**Espacio de estados (4 estados discretos):**

- Estado 0: Obstáculo al frente
- Estado 1: Obstáculo a la izquierda
- Estado 2: Obstáculo a la derecha
- Estado 3: Camino libre

**Acciones disponibles (3 acciones):**

- Acción 0: Avanzar
- Acción 1: Girar a la izquierda
- Acción 2: Girar a la derecha

**Sistema de recompensas:**

- +100: Alcanzar el objetivo (distancia < 0.1)
- -10: Colisión frontal
- -2: Cerca de obstáculo lateral
- Positiva variable: Recompensa mayor cuanto más cerca del objetivo

**Funcionalidades:**

- Discretización de sensores de proximidad frontales
- Estrategia de recuperación ante colisiones (retroceso y giro)
- Reinicio automático del episodio al alcanzar el objetivo
- Tabla Q que se actualiza continuamente durante la ejecución

### 2. Mundo Webots (`qlearning-world.wbt`)

El entorno de simulación contiene:

**Elementos del mundo:**

- **Robot:** E-puck equipado con 8 sensores de proximidad
- **Objetivo:** Esfera verde (DEF TARGET) que el robot debe alcanzar
- **Obstáculos:** 4 cajas sólidas distribuidas en el entorno
- **Arena:** Suelo rectangular delimitado
- **Iluminación:** Fondo texturizado con iluminación

**Configuración:**

- El robot inicia en posición `[-0.315, 0.236, -0.002]`
- El objetivo (esfera verde) puede moverse en el entorno
- Los obstáculos están posicionados estratégicamente para crear desafíos de navegación

## Cómo Ejecutar

1. Abre Webots
2. Carga el mundo: `worlds/qlearning-world.wbt`
3. El controlador `qlearning_controller` se asignará automáticamente al robot
4. Ejecuta la simulación
5. Observa cómo el robot aprende a navegar hacia el objetivo

## Observaciones

- El robot aprenderá progresivamente mejores políticas de navegación
- La consola muestra en tiempo real: estado actual, acción tomada, recompensa y distancia al objetivo
- Al alcanzar el objetivo, el robot se reinicia para continuar aprendiendo
- La tabla Q se conserva entre episodios, permitiendo aprendizaje acumulativo

## Requisitos

- Webots R2025a o superior
- Python 3.x
- NumPy

---

_Proyecto de Robótica - UNSA_
