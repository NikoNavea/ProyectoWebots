#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define GRID_SIZE 8  
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 100
#define MAX_OPEN_NODES 1000
#define SPEED 4.0
#define WAYPOINT_THRESHOLD 0.1
//Distancia para considerar que se ha alcanzado el punto

typedef struct {
  int x, y;
} Point;

typedef struct {
  int x, y;
  int g, h, f;
  int parent_index;
} Node;


int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
  Node open_list[MAX_OPEN_NODES];
  int open_count = 0;

  Node closed_list[GRID_SIZE * GRID_SIZE];
  int closed_count = 0;

  Node start_node = {start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1};
  start_node.f = start_node.g + start_node.h;
  open_list[open_count++] = start_node;

  while (open_count > 0) {
    int best_index = 0;
    for (int i = 1; i < open_count; i++) {
      if (open_list[i].f < open_list[best_index].f)
        best_index = i;
    }

    Node current = open_list[best_index];
    for (int i = best_index; i < open_count - 1; i++)
      open_list[i] = open_list[i + 1];
    open_count--;
    closed_list[closed_count++] = current;

    if (current.x == goal.x && current.y == goal.y) {
      int length = 0;
      Node n = current;
      while (n.parent_index != -1 && length < max_path_len) {
        path[length++] = (Point){n.x, n.y};
        n = closed_list[n.parent_index];
      }
      path[length++] = (Point){start.x, start.y};
      for (int i = 0; i < length / 2; i++) {
        Point temp = path[i];
        path[i] = path[length - i - 1];
        path[length - i - 1] = temp;
      }
      return length;
    }

    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};
    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE)
        continue;
      if (grid[nx][ny] == 1)
        continue;

      int in_closed = 0;
      for (int i = 0; i < closed_count; i++) {
        if (closed_list[i].x == nx && closed_list[i].y == ny) {
          in_closed = 1;
          break;
        }
      }
      if (in_closed) continue;

      int g = current.g + 1;
      int h = heuristic(nx, ny, goal.x, goal.y);
      int f = g + h;

      int in_open = -1;
      for (int i = 0; i < open_count; i++) {
        if (open_list[i].x == nx && open_list[i].y == ny) {
          in_open = i;
          break;
        }
      }

      if (in_open != -1) {
        if (f < open_list[in_open].f) {
          open_list[in_open].g = g;
          open_list[in_open].h = h;
          open_list[in_open].f = f;
          open_list[in_open].parent_index = closed_count - 1;
        }
      } else if (open_count < MAX_OPEN_NODES) {
        Node neighbor = {nx, ny, g, h, f, closed_count - 1};
        open_list[open_count++] = neighbor;
      }
    }
  }
  return 0;
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
*/
 
int main() {
  wb_robot_init();
  double left_speed = 1.0;
  double right_speed = 1.0;
  int i;

  //motores
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  
  //sensores 
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  //lidar
  
  WbDeviceTag lidar=wb_robot_get_device("lidar");
  wb_lidar_enable(lidar,TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  
  //GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  WbDeviceTag imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu, TIME_STEP);

  //Variables de estado para la navegación
  int grid[GRID_SIZE][GRID_SIZE] = {0};
  Point path[MAX_PATH_LEN];
  int path_length = 0;
  int current_path_index = 0;
  bool replan = true;
  
  while (wb_robot_step(TIME_STEP) != -1) {
   
   //Flags
   bool ds_detect_near=false;
   bool lidar_detect_near=false;
   
   left_speed = 0.0;
   right_speed = 0.0;
   
   //Percepción
   double ds_values[2];
   for (i = 0; i < 2; i++){
     ds_values[i] = wb_distance_sensor_get_value(ds[i]);
     printf("Sensor %d: %.2f\n", i, ds_values[i]);
   }
  if (ds_values[0] < 950.0 || ds_values[1] < 950.0)
     ds_detect_near=true;
        
  
    
    //GPS
    const double *gps_pose = wb_gps_get_values(gps);
    float robot_x = gps_pose[0];
    float robot_y = gps_pose[2]; // Webots usa X-Z como plano horizontal
   
    // Obtener orientación del robot desde la IMU en radianes   
    const double *imu_rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    double robot_yaw = imu_rpy[2]; //Yaw es el ángulo en el plano horizontal
    
    
    //lidar
    // (1) Obtener datos del LIDAR y construir mapa de obstáculos
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    
    // Limpiar la grilla antes de volver a mapear (mapeo local)
    for(int gx=0; gx < GRID_SIZE; ++gx) for(int gy=0; gy < GRID_SIZE; ++gy) grid[gx][gy] = 0;
    
    for (int i = 0; i < resolution; i++) {
      double angle = -fov / 2 + i * (fov / resolution);
      double dist = ranges[i];
      
      //printf("Sensor lidar %d: %.2f\n", i, dist);
      
      if(isinf(dist)) continue; //ignorar infinito

      if (dist < 1.0) {
        float obs_x = robot_x + dist * cos(angle);
        float obs_y = robot_y + dist * sin(angle);

        int cell_x = (int)((obs_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        int cell_y = (int)((obs_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);

        if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE)
          grid[cell_x][cell_y] = 1;  // Marcado como ocupado
      }
      
      if(dist < 0.3)
        lidar_detect_near=true; //obstaculo muy cerca girar
    
      }
      
      //plan path
      // --- PLANIFICACIÓN ---
    if (replan) {
      Point start = {(int)((robot_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE), 
                     (int)((robot_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE)};
      Point goal = {GRID_SIZE - 2, GRID_SIZE - 2}; // Objetivo fijo por ahora
      
      path_length = plan_path(grid, start, goal, path, MAX_PATH_LEN);
      current_path_index = 0; // Reiniciar el seguimiento de la ruta
      replan = false; // No replanificar hasta que sea necesario
      
      if (path_length > 0) {
        printf("Ruta encontrada con %d puntos. Iniciando navegación.\n", path_length);
      } else {
        printf("No se pudo encontrar una ruta.\n");
      }
    }
        
    // --- CONTROL ---
    if (lidar_detect_near || ds_detect_near) {
      // Evasión de emergencia: Girar
      printf("Obstáculo cercano! Evasión de emergencia.\n");
      left_speed = SPEED;
      right_speed = -SPEED;
      replan = true; // Forzar replanificación después de la evasión
    } else if (path_length > 0 && current_path_index < path_length) {
      // NUEVO: Lógica para seguir el camino (Path Following)
      
      // 1. Definir el punto objetivo actual (waypoint)
      Point current_waypoint_grid = path[current_path_index];
      double target_x = (current_waypoint_grid.x - GRID_SIZE / 2) * CELL_SIZE;
      double target_y = (current_waypoint_grid.y - GRID_SIZE / 2) * CELL_SIZE;
      
      // 2. Calcular la distancia y el ángulo hacia el waypoint
      double dx = target_x - robot_x;
      double dy = target_y - robot_y;
      double distance_to_target = sqrt(dx * dx + dy * dy);
      double angle_to_target = atan2(dy, dx);
      
      // 3. Comprobar si hemos llegado al waypoint
      if (distance_to_target < WAYPOINT_THRESHOLD) {
        printf("Waypoint %d/%d alcanzado.\n", current_path_index + 1, path_length);
        current_path_index++; // Ir al siguiente punto
      } else {
        // 4. Control proporcional para girar hacia el waypoint
        double angle_error = angle_to_target - robot_yaw;
        
        // Normalizar el ángulo a [-PI, PI]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;
        
        // El controlador P ajusta la velocidad de las ruedas para girar
        // El factor 0.5 es la ganancia proporcional (puedes ajustarla)
        double turn_speed = angle_error * 0.5; 
        
        left_speed = SPEED - turn_speed;
        right_speed = SPEED + turn_speed;
      }
      
    } else {
      // No hay ruta o ya se completó
      printf("Navegación completada o sin ruta.\n");
      left_speed = 0.0;
      right_speed = 0.0;
    }
      
    // Limitar la velocidad para que no exceda el máximo
    if (left_speed > SPEED) left_speed = SPEED;
    if (left_speed < -SPEED) left_speed = -SPEED;
    if (right_speed > SPEED) right_speed = SPEED;
    if (right_speed < -SPEED) right_speed = -SPEED;
    
    // Asignar velocidad a los motores
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
    
    fflush(stdout); 
  };
  
  wb_robot_cleanup();
  return 0;
}
