#include <webots/distance_sensor.h>
#include <webots/lidar.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* --- Parámetros de Configuración --- */
#define TIME_STEP 64
#define MAX_SPEED 6.0

// Parámetros de la Grilla y el Mundo
#define GRID_SIZE 8
#define CELL_SIZE 0.5
// Parámetros del Planificador A*
#define MAX_PATH_LEN 256
#define MAX_OPEN_NODES 2000

// Parámetros de Navegación y Control
#define TURN_GAIN 1.5
#define ANGLE_TOLERANCE 0.05 

// Distancia en metros a la que el Lidar activará la evasión.
#define LIDAR_EVASION_DISTANCE 0.1
// Porcentaje del Lidar que se considera "frontal" (ej. 0.1 = 10% central).
#define LIDAR_FRONT_CONE_PERCENT 0.10 

/* --- Estructuras y Algoritmo A* (Formato Limpio) --- */
typedef struct {
  int x;
  int y;
} Point;

typedef struct {
  int x;
  int y;
  int g;
  int h;
  int f;
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
  
  Node start_node;
  start_node.x = start.x;
  start_node.y = start.y;
  start_node.g = 0;
  start_node.h = heuristic(start.x, start.y, goal.x, goal.y);
  start_node.f = start_node.g + start_node.h;
  start_node.parent_index = -1;
  
  open_list[open_count++] = start_node;
  
  while (open_count > 0) {
    int best_index = 0;
    for (int i = 1; i < open_count; i++) {
      if (open_list[i].f < open_list[best_index].f) {
        best_index = i;
      }
    }
    
    Node current = open_list[best_index];
    
    for (int i = best_index; i < open_count - 1; i++) {
      open_list[i] = open_list[i + 1];
    }
    open_count--;
    
    closed_list[closed_count++] = current;
    
    if (current.x == goal.x && current.y == goal.y) {
      int length = 0;
      int path_build_index = -1;
      
      for(int i = 0; i < closed_count; ++i) {
        if(closed_list[i].x == current.x && closed_list[i].y == current.y) {
          path_build_index = i;
          break;
        }
      }
      
      while (path_build_index != -1 && length < max_path_len) {
        path[length].x = closed_list[path_build_index].x;
        path[length].y = closed_list[path_build_index].y;
        length++;
        path_build_index = closed_list[path_build_index].parent_index;
      }
      
      for (int i = 0; i < length / 2; i++) {
        Point temp = path[i];
        path[i] = path[length - i - 1];
        path[length - i - 1] = temp;
      }
      return length;
    }
    
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {1, 0, -1, 0};
    
    for (int d = 0; d < 4; d++) {
      int nx = current.x + dx[d];
      int ny = current.y + dy[d];
      
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) {
        continue;
      }
      if (grid[nx][ny] == 1) {
        continue;
      }
      
      bool in_closed = false;
      for (int i = 0; i < closed_count; i++) {
        if (closed_list[i].x == nx && closed_list[i].y == ny) {
          in_closed = true;
          break;
        }
      }
      if (in_closed) {
        continue;
      }
      
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
      
      int current_parent_index = -1;
      for(int i = 0; i < closed_count; ++i) {
        if(closed_list[i].x == current.x && closed_list[i].y == current.y) {
          current_parent_index = i;
          break;
        }
      }
      
      if (in_open != -1) {
        if (g < open_list[in_open].g) {
          open_list[in_open].g = g;
          open_list[in_open].f = g + open_list[in_open].h;
          open_list[in_open].parent_index = current_parent_index;
        }
      } else if (open_count < MAX_OPEN_NODES) {
        Node neighbor;
        neighbor.x = nx;
        neighbor.y = ny;
        neighbor.g = g;
        neighbor.h = h;
        neighbor.f = f;
        neighbor.parent_index = current_parent_index;
        open_list[open_count++] = neighbor;
      }
    }
  }
  return 0;
}

// --- SLAM SIMPLE (Paso 1): La grilla se declara globalmente para que sea persistente ---
// Se inicializará a ceros automáticamente una sola vez al inicio del programa.
int grid[GRID_SIZE][GRID_SIZE];
/* --- PROGRAMA PRINCIPAL --- */
int main() {
  wb_robot_init();

  /* --- Inicialización de Dispositivos --- */
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0.0);
  }
  
  // CORRECCIÓN APLICADA AQUÍ
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (int i = 0; i < 2; i++) {
    // Se usa ds_names[i] (el nombre) en lugar de ds[i] (la variable vacía)
    ds[i] = wb_robot_get_device(ds_names[i]); 
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag imu = wb_robot_get_device("inertial_unit");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  if (imu) {
    wb_inertial_unit_enable(imu, TIME_STEP);
  }

  /* --- Variables de Navegación --- */
  int grid[GRID_SIZE][GRID_SIZE] = {{0}};
  Point path[MAX_PATH_LEN];
  Point goal = {6,1};
  int path_length = 0;
  int current_path_index = 0;
  
  // --- Variables para Métricas de Desempeño ---
  double total_planning_time_ms = 0.0;
  int last_planned_path_length = 0;
  // Mapa persistente para registrar las celdas visitadas
  int exploration_grid[GRID_SIZE][GRID_SIZE] = {{0}}; 
  int visited_cells_count = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed = 0.0;
    double right_speed = 0.0;

    /* --- 1. PERCEPCIÓN --- */
    const double *gps_pose = wb_gps_get_values(gps);
    const double *imu_rpy = imu ? wb_inertial_unit_get_roll_pitch_yaw(imu) : NULL;
    float robot_x = gps_pose[0];
    float robot_z = gps_pose[1]; 
    float robot_yaw = imu_rpy ? imu_rpy[2] : 0;
    
    const float *ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    
    double ds_val_left = wb_distance_sensor_get_value(ds[0]);
    double ds_val_right = wb_distance_sensor_get_value(ds[1]);
    
    bool lidar_trigger_evasion = false;
    int cone_size = resolution * LIDAR_FRONT_CONE_PERCENT;
    int start_index = resolution / 2 - cone_size / 2;
    int end_index = resolution / 2 + cone_size / 2;

    for (int i = start_index; i < end_index; i++) {
        if (ranges[i] < LIDAR_EVASION_DISTANCE) {
            lidar_trigger_evasion = true;
            break; // Obstáculo detectado, no es necesario seguir buscando
        }
    }
    
    printf("--- DEBUG --- DS Iz: %.0f | DS Der: %.0f | EVADIENDO: %s | Path Len: %d\n",
           ds_val_left, 
           ds_val_right,
           lidar_trigger_evasion ? "SI" : "NO",
           path_length);
    
    int casilla_x = (int)((robot_x + GRID_SIZE * CELL_SIZE / 2.0) / CELL_SIZE);
    int casilla_y = (int)((-robot_z + GRID_SIZE * CELL_SIZE / 2.0) / CELL_SIZE); // Ojo con el signo negativo en la Z;      
   
    printf("--- DEBUG --- CASILLA ACTUAL = ( %i , %i )",casilla_x,casilla_y);
    
    // Si la celda actual no ha sido visitada, la marcamos y contamos
    if (exploration_grid[casilla_x][casilla_y] == 0) {
        exploration_grid[casilla_x][casilla_y] = 1;
        visited_cells_count++;
    }
        
    /* --- 2. LÓGICA DE CONTROL --- */
    if (casilla_x == goal.x && casilla_y == goal.y) {
    
        double total_navigation_time = wb_robot_get_time(); // Tiempo total desde el inicio de la simulación
        double percentage_explored = ((double)visited_cells_count / (double)(GRID_SIZE * GRID_SIZE)) * 100.0;
        printf("\n\n############################################\n");
        printf("¡OBJETIVO ALCANZADO EN LA CASILLA (%d, %d)!\n", goal.x, goal.y);
        printf("############################################\n\n");
        
        printf("--- Métricas de Desempeño ---\n");
        printf("- Tiempo total de navegación: %.2f segundos\n", total_navigation_time);
        printf("- Longitud del último path (celdas): %d\n", last_planned_path_length);
        printf("- Tiempo total de planificación (A*): %.4f milisegundos\n", total_planning_time_ms);
        printf("- Porcentaje del mapa explorado: %.2f %%\n", percentage_explored);
        printf("--------------------------------\n\n");
            
        // Detener los motores inmediatamente
        wb_motor_set_velocity(wheels[0], 0.0);
        wb_motor_set_velocity(wheels[1], 0.0);
        wb_motor_set_velocity(wheels[2], 0.0);
        wb_motor_set_velocity(wheels[3], 0.0);
        
        fflush(stdout); // Asegurarse de que el mensaje se imprima
        
        // Salir del bucle principal para terminar el programa limpiamente
        break; 
    }
    
    bool izq = false;
    bool der = false;
    if (lidar_trigger_evasion) {
        path_length = 0; // Descartar plan actual para forzar replanificación
        if(izq == false){
          if(der == false){
            if(ds_val_left > ds_val_right) izq = true;
            else der = true;
            }
        }
        
        // Comparamos los sensores. Un valor ALTO significa obstáculo CERCANO.
        if (der) {
            // Obstáculo más a la izquierda -> Gira a la DERECHA
            left_speed = MAX_SPEED * 0.9;
            right_speed = -MAX_SPEED * 0.9;
        } else {
            // Obstáculo más a la derecha o de frente -> Gira a la IZQUIERDA
            left_speed = -MAX_SPEED * 0.9;
            right_speed = MAX_SPEED * 0.9;
        }
    } else {
        // REGLA 2: Si no hay obstáculo inmediato, se navega o se planifica.
        izq = false;
        der = false;
        // El mapa del LIDAR se usa para la planificación, no para la evasión inmediata.
        for(int i = 0; i < GRID_SIZE; i++) {
          for(int j = 0; j < GRID_SIZE; j++) {
            grid[i][j] = 0;
          }
        }
        for (int i = 0; i < resolution; i++) {
          double angle = ((double)i / resolution) * 2.0 * M_PI;
          float dist = ranges[i];
          if (isinf(dist) || isnan(dist) || dist > (CELL_SIZE * GRID_SIZE / 2.0)) {
            continue;
          }
          float obs_x = robot_x + dist * cos(angle + robot_yaw);
          float obs_z = robot_z + dist * sin(angle + robot_yaw);
          int cell_x = (int)((obs_x + GRID_SIZE * CELL_SIZE / 2.0) / CELL_SIZE);
          int cell_z = (int)((-obs_z + GRID_SIZE * CELL_SIZE / 2.0) / CELL_SIZE);
          if (cell_x >= 0 && cell_x < GRID_SIZE && cell_z >= 0 && cell_z < GRID_SIZE) {
            grid[cell_x][cell_z] = 1;
          }
        }
        
        if (path_length > 0) {
            // Seguir la ruta A* existente
            Point waypoint = path[current_path_index];
            float target_x = (waypoint.x - GRID_SIZE / 2.0 + 0.5) * CELL_SIZE;
            float target_z = -(waypoint.y - GRID_SIZE / 2.0 + 0.5) * CELL_SIZE;
            float angle_to_target = atan2(target_z - robot_z, target_x - robot_x);
            float angle_error = angle_to_target - robot_yaw;
            
            while(angle_error > M_PI) { angle_error -= 2 * M_PI; }
            while(angle_error < -M_PI) { angle_error += 2 * M_PI; }

            if (fabs(angle_error) < ANGLE_TOLERANCE) {
                left_speed = MAX_SPEED;
                right_speed = MAX_SPEED;
            } else {
                left_speed = MAX_SPEED - TURN_GAIN * angle_error;
                right_speed = MAX_SPEED + TURN_GAIN * angle_error;
            }
            
            float distance_to_target = sqrt(pow(target_x - robot_x, 2) + pow(target_z - robot_z, 2));
            if (distance_to_target < CELL_SIZE) {
                current_path_index++;
                if (current_path_index >= path_length) {
                    path_length = 0; // Ruta completada
                }
            }
        } else {
            // No hay ruta, planificar una nueva
            Point current_cell;
            current_cell.x = (int)((robot_x + GRID_SIZE * CELL_SIZE / 2.0) / CELL_SIZE);
            current_cell.y = (int)((-robot_z + GRID_SIZE * CELL_SIZE / 2.0) / CELL_SIZE);
            
            grid[current_cell.x][current_cell.y] = 0;
            grid[goal.x][goal.y] = 0;
            
            double planning_start = wb_robot_get_time();
            path_length = plan_path(grid, current_cell, goal, path, MAX_PATH_LEN);
            double planning_end = wb_robot_get_time();
            
            total_planning_time_ms += (planning_end - planning_start) * 1000.0;
            current_path_index = 0;

            if (path_length == 0) {
                printf("ADVERTENCIA: No se pudo encontrar una ruta. Girando para desatascar...\n");
                left_speed = MAX_SPEED / 3.0;
                right_speed = -MAX_SPEED / 3.0;
            }
        }
    }

    /* --- 3. APLICACIÓN DE VELOCIDAD --- */
    if (left_speed > MAX_SPEED) { left_speed = MAX_SPEED; }
    if (left_speed < -MAX_SPEED) { left_speed = -MAX_SPEED; }
    if (right_speed > MAX_SPEED) { right_speed = MAX_SPEED; }
    if (right_speed < -MAX_SPEED) { right_speed = -MAX_SPEED; }
    
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    wb_motor_set_velocity(wheels[2], left_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
    
    fflush(stdout);
  };

  wb_robot_cleanup();
  
  return 0;
}