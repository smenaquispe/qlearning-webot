/* Q-Learning controller for e-puck to learn wall avoidance
   Compile and run as Webots controller (C).
   Saves Q-table to "q_table.bin". */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>

#define DISTANCE_SENSORS_NUMBER 8
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] =
    {"ps0","ps1","ps2","ps3","ps4","ps5","ps6","ps7"};

#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] =
    {"led0","led1","led2","led3","led4","led5","led6","led7","led8","led9"};

static WbDeviceTag left_motor, right_motor;

#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28

/* Q-Learning parameters */
#define NUM_STATES 4
#define NUM_ACTIONS 3
double Q[NUM_STATES][NUM_ACTIONS];
double alpha = 0.2;        // learning rate
double gamma_factor = 0.9; // discount factor
double epsilon = 0.15;     // exploration probability

/* action durations (how many robot steps to hold the action) */
int ACTION_DURATION_STEPS = 5;

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step_once() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

/* map raw sensor values (0..4096) to normalized 0..1 (1 = very close) */
static void read_distance_sensors_normalized() {
  for (int i = 0; i < DISTANCE_SENSORS_NUMBER; ++i) {
    double val = wb_distance_sensor_get_value(distance_sensors[i]); // 0..4096 roughly
    distance_sensors_values[i] = val / 4096.0;
    if (distance_sensors_values[i] > 1.0) distance_sensors_values[i] = 1.0;
    if (distance_sensors_values[i] < 0.0) distance_sensors_values[i] = 0.0;
  }
}

/* Discretize into 4 states using frontal sensors */
static int get_state_discrete() {
  double front_left = distance_sensors_values[7];
  double front_right = distance_sensors_values[0];
  double front_center = (distance_sensors_values[7] + distance_sensors_values[0] +
                         distance_sensors_values[1] + distance_sensors_values[6]) / 4.0;

  const double COLLIDE_THRESHOLD = 0.70;
  const double CLOSE_THRESHOLD = 0.35;

  if (front_center > COLLIDE_THRESHOLD) return 0;          // obstacle front
  if (front_left > CLOSE_THRESHOLD && front_left >= front_right) return 1;   // obstacle left
  if (front_right > CLOSE_THRESHOLD && front_right > front_left) return 2;   // obstacle right
  return 3; // free
}

/* Rewards */
static double compute_reward(int state) {
  if (state == 0) return -10.0;
  if (state == 1 || state == 2) return -2.0;
  return +1.0;
}

/* Epsilon-greedy action selection */
static int choose_action(int state) {
  double r = (double)rand() / RAND_MAX;
  if (r < epsilon)
    return rand() % NUM_ACTIONS;
  else {
    int best = 0;
    double bestv = Q[state][0];
    for (int a = 1; a < NUM_ACTIONS; ++a) {
      if (Q[state][a] > bestv) {
        bestv = Q[state][a];
        best = a;
      }
    }
    return best;
  }
}

/* Execute action */
static void execute_action(int action) {
  if (action == 0) {
    double v = 0.9 * MAX_SPEED;
    wb_motor_set_velocity(left_motor, v);
    wb_motor_set_velocity(right_motor, v);
  } else if (action == 1) {
    wb_motor_set_velocity(left_motor, -0.6 * MAX_SPEED);
    wb_motor_set_velocity(right_motor, 0.6 * MAX_SPEED);
  } else if (action == 2) {
    wb_motor_set_velocity(left_motor, 0.6 * MAX_SPEED);
    wb_motor_set_velocity(right_motor, -0.6 * MAX_SPEED);
  }
}

static void stop_motors() {
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
}

/* Save/load Q-table */
static void save_qtable(const char *filename) {
  FILE *f = fopen(filename, "wb");
  if (!f) {
    printf("⚠️  No se pudo abrir %s para guardar Q-table\n", filename);
    return;
  }
  fwrite(Q, sizeof(double), NUM_STATES * NUM_ACTIONS, f);
  fclose(f);
  printf("💾 Q-table guardada en %s\n", filename);
}

static void load_qtable(const char *filename) {
  FILE *f = fopen(filename, "rb");
  if (!f) {
    printf("ℹ️  No se encontró Q-table previa (%s). Se inicializa a 0.\n", filename);
    for (int s = 0; s < NUM_STATES; ++s)
      for (int a = 0; a < NUM_ACTIONS; ++a)
        Q[s][a] = 0.0;
    return;
  }
  fread(Q, sizeof(double), NUM_STATES * NUM_ACTIONS, f);
  fclose(f);
  printf("📂 Q-table cargada desde %s\n", filename);
}

int main(int argc, char **argv) {
  wb_robot_init();
  srand((unsigned int)time(NULL));

  printf("🤖 e-puck Q-Learning controller iniciado...\n");

  int time_step = get_time_step();

  /* Initialize devices */
  for (int i = 0; i < DISTANCE_SENSORS_NUMBER; ++i) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], time_step);
    printf("✅ Sensor de distancia '%s' habilitado.\n", distance_sensors_names[i]);
  }

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  stop_motors();

  const char *qfile = "q_table.bin";
  load_qtable(qfile);

  printf("ℹ️  Parámetros: alpha=%.2f gamma=%.2f epsilon=%.3f action_steps=%d\n",
         alpha, gamma_factor, epsilon, ACTION_DURATION_STEPS);

  int step_count = 0;

  while (true) {
    read_distance_sensors_normalized();
    int state = get_state_discrete();

    int action = choose_action(state);

    for (int t = 0; t < ACTION_DURATION_STEPS; ++t) {
      execute_action(action);
      step_once();
      ++step_count;
    }

    read_distance_sensors_normalized();
    int next_state = get_state_discrete();
    double reward = compute_reward(next_state) - 0.05; /* step penalty */

    double max_next = Q[next_state][0];
    for (int a = 1; a < NUM_ACTIONS; ++a)
      if (Q[next_state][a] > max_next) max_next = Q[next_state][a];

    double old = Q[state][action];
    Q[state][action] = old + alpha * (reward + gamma_factor * max_next - old);

    if (step_count % 50 == 0) {
      printf("Step %d | s=%d a=%d r=%.2f s'=%d | Q[s,a]=%.3f\n",
             step_count, state, action, reward, next_state, Q[state][action]);
    }

    if (next_state == 0) {
      wb_motor_set_velocity(left_motor, -0.8 * MAX_SPEED);
      wb_motor_set_velocity(right_motor, -0.8 * MAX_SPEED);
      for (int k = 0; k < 6; ++k) step_once();
      wb_motor_set_velocity(left_motor, -0.6 * MAX_SPEED);
      wb_motor_set_velocity(right_motor, 0.6 * MAX_SPEED);
      for (int k = 0; k < 6; ++k) step_once();
      stop_motors();
    }

    if (step_count % 2000 == 0)
      save_qtable(qfile);

    if (step_count % 1000 == 0 && epsilon > 0.02)
      epsilon *= 0.995;
  }

  save_qtable(qfile);
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
