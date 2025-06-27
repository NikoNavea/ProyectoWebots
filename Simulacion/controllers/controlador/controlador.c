#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

// === PARÁMETROS DE CONFIGURACIÓN ===
#define TIME_STEP 128
#define GRID_SIZE 8
#define CELL_SIZE 0.5
#define MAX_PATH_LEN 1000
#define MAX_OPEN_NODES 2000

// --- Parámetros de Movimiento MEJORADOS ---
#define MAX_SPEED 10    // Velocidad más conservadora
#define KP_TURN 0.07    // Control más suave
#define GOAL_THRESHOLD 0.0001 // Umbral más tolerante

// === ESTRUCTURAS Y FUNCIONES A* ===
typedef struct { int x, y; } Point;
typedef struct { int x, y; int g, h, f; int parent_index; } Node;

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void print_grid(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal) {
    printf("\n=== GRID DEBUG ===\n");
    printf("Start: (%d, %d), Goal: (%d, %d)\n", start.x, start.y, goal.x, goal.y);
    printf("Grid (R=Robot, G=Goal, 1=Obstacle, 0=Free):\n");
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (x == start.x && y == start.y) printf("R ");
            else if (x == goal.x && y == goal.y) printf("G ");
            else printf("%d ", grid[y][x]);
        }
        printf("\n");
    }
    printf("==================\n\n");
}

int plan_path(int grid[GRID_SIZE][GRID_SIZE], Point start, Point goal, Point path[], int max_path_len) {
    printf("Planificando ruta desde (%d,%d) hacia (%d,%d)\n", start.x, start.y, goal.x, goal.y);
    
    // Verificar límites
    if (start.x < 0 || start.x >= GRID_SIZE || start.y < 0 || start.y >= GRID_SIZE ||
        goal.x < 0 || goal.x >= GRID_SIZE || goal.y < 0 || goal.y >= GRID_SIZE) {
        printf("ERROR: Posición fuera de límites\n");
        return 0;
    }
    
    // Si el punto de partida o el de llegada está en un obstáculo, no hay ruta.
    if (grid[start.y][start.x] == 1) {
        printf("ERROR: Posición inicial en obstáculo\n");
        return 0;
    }
    if (grid[goal.y][goal.x] == 1) {
        printf("ERROR: Posición objetivo en obstáculo\n");
        return 0;
    }

    Node open_list[MAX_OPEN_NODES]; int open_count = 0;
    Node closed_list[GRID_SIZE * GRID_SIZE]; int closed_count = 0;
    bool closed_map[GRID_SIZE][GRID_SIZE] = {false};
    Node start_node = {start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y), 0, -1};
    start_node.f = start_node.g + start_node.h;
    open_list[open_count++] = start_node;
    
    while (open_count > 0) {
        int best_index = 0;
        for (int i = 1; i < open_count; i++) { 
            if (open_list[i].f < open_list[best_index].f) best_index = i; 
        }
        Node current = open_list[best_index];
        for (int i = best_index; i < open_count - 1; i++) open_list[i] = open_list[i + 1];
        open_count--;
        closed_list[closed_count++] = current;
        closed_map[current.y][current.x] = true;
        
        if (current.x == goal.x && current.y == goal.y) {
            printf("¡Ruta encontrada!\n");
            int length = 0; Node n = current;
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
        
        const int dx[] = {0, 1, 0, -1}, dy[] = {1, 0, -1, 0};
        for (int d = 0; d < 4; d++) {
            int nx = current.x + dx[d], ny = current.y + dy[d];
            if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE || 
                grid[ny][nx] == 1 || closed_map[ny][nx]) continue;
            
            int g = current.g + 1, h = heuristic(nx, ny, goal.x, goal.y), f = g + h;
            int in_open = -1;
            for (int i = 0; i < open_count; i++) { 
                if (open_list[i].x == nx && open_list[i].y == ny) { 
                    in_open = i; break; 
                } 
            }
            int current_in_closed_idx = closed_count - 1;
            if (in_open != -1) {
                if (f < open_list[in_open].f) { 
                    open_list[in_open].g = g; 
                    open_list[in_open].f = f; 
                    open_list[in_open].parent_index = current_in_closed_idx; 
                }
            } else if (open_count < MAX_OPEN_NODES) {
                Node neighbor = {nx, ny, g, h, f, current_in_closed_idx}; 
                open_list[open_count++] = neighbor;
            }
        }
    }
    printf("No se encontró ruta - lista abierta vacía\n");
    return 0;
}

bool is_path_obstructed(Point path[], int length, int current_index, int grid[GRID_SIZE][GRID_SIZE]) {
    for (int i = current_index; i < length; ++i) {
        if (path[i].y >= 0 && path[i].y < GRID_SIZE && path[i].x >= 0 && path[i].x < GRID_SIZE) {
            if (grid[path[i].y][path[i].x] == 1) {
                return true;
            }
        }
    }
    return false;
}

// NUEVA FUNCIÓN: Filtro para limpiar grid de obstáculos fantasma
void clean_grid_around_robot(int grid[GRID_SIZE][GRID_SIZE], double robot_x, double robot_y) {
    int robot_cell_x = (int)((robot_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
    int robot_cell_y = (int)((robot_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
    
    // Limpiar celdas inmediatamente alrededor del robot
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int cell_x = robot_cell_x + dx;
            int cell_y = robot_cell_y + dy;
            if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE) {
                // Solo limpiar la celda del robot mismo
                if (dx == 0 && dy == 0) {
                    grid[cell_y][cell_x] = 0;
                }
            }
        }
    }
}

int main() {
    wb_robot_init();
    WbDeviceTag wheels[4];
    char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    for (int i = 0; i < 4; i++) { 
        wheels[i] = wb_robot_get_device(wheels_names[i]); 
        wb_motor_set_position(wheels[i], INFINITY); 
        wb_motor_set_velocity(wheels[i], 0.0); 
    }
    WbDeviceTag lidar = wb_robot_get_device("lidar"); wb_lidar_enable(lidar, TIME_STEP);
    WbDeviceTag gps = wb_robot_get_device("gps"); wb_gps_enable(gps, TIME_STEP);
    WbDeviceTag compass = wb_robot_get_device("compass"); wb_compass_enable(compass, TIME_STEP);

    int grid[GRID_SIZE][GRID_SIZE] = {0};
    Point path[MAX_PATH_LEN];
    int path_length = 0;
    int current_path_index = 0;
    
    // Objetivo más cercano para pruebas
    Point goal_world = {1.0, 1.0};
    Point goal_grid;
    
    int debug_counter = 0;
    bool first_run = true;

    while (wb_robot_step(TIME_STEP) != -1) {
        const double *pose = wb_gps_get_values(gps);
        double robot_x = pose[0], robot_y = pose[2];
        const double *north = wb_compass_get_values(compass);
        double robot_theta = atan2(north[0], north[2]);
        
        // Actualizar posición del objetivo en grid
        goal_grid.x = (int)((goal_world.x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        goal_grid.y = (int)((goal_world.y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
        
        if (first_run) {
            printf("Configuración inicial:\n");
            printf("Robot mundo: (%.2f, %.2f)\n", robot_x, robot_y);
            printf("Objetivo mundo: (%.2f, %.2f)\n", goal_world.x, goal_world.y);
            printf("GRID_SIZE: %d, CELL_SIZE: %.2f\n", GRID_SIZE, CELL_SIZE);
            first_run = false;
        }
        
        // *** CLAVE: LIMPIAR COMPLETAMENTE EL GRID EN CADA ITERACIÓN ***
        memset(grid, 0, sizeof(grid));
        
        const float *ranges = wb_lidar_get_range_image(lidar);
        int resolution = wb_lidar_get_horizontal_resolution(lidar);
        double fov = wb_lidar_get_fov(lidar);
        
        // NUEVA LÓGICA: Detección de obstáculos más conservadora
        double safety_margin = 0.15; // Reducido de 0.2 a 0.15
        double min_detection_distance = 0.1;  // Ignorar lecturas muy cerca
        double max_detection_distance = 1.5;  // Reducido de 2.0 a 1.5
        
        int obstacles_detected = 0; // Para debug
        
        for (int i = 0; i < resolution; i++) {
            double angle = fov * (double)i / resolution - fov / 2.0;
            double dist = ranges[i];
            
            // Filtrar lecturas inválidas y muy cercanas/lejanas
            if (isinf(dist) || isnan(dist) || dist < min_detection_distance || dist > max_detection_distance) {
                continue;
            }

            double global_angle = robot_theta + angle;
            double obs_x = robot_x + (dist - safety_margin) * cos(global_angle);
            double obs_y = robot_y + (dist - safety_margin) * sin(global_angle);
            
            int cell_x = (int)((obs_x + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
            int cell_y = (int)((obs_y + GRID_SIZE * CELL_SIZE / 2) / CELL_SIZE);
            
            if (cell_x >= 0 && cell_x < GRID_SIZE && cell_y >= 0 && cell_y < GRID_SIZE) {
                grid[cell_y][cell_x] = 1;
                obstacles_detected++;
            }
        }
        
        // Limpiar posición del robot para asegurar que no se marque como obstáculo
        clean_grid_around_robot(grid, robot_x, robot_y);
        
        // NUEVA LÓGICA: Comportamiento de emergencia más específico
        bool emergency_stop = false;
        double min_distance = INFINITY;
        double emergency_threshold = 0.25; // Aumentado ligeramente
        
        // Revisar solo el frente inmediato para emergencias
        int front_start = (int)(resolution * 0.45);  // Más centrado
        int front_end = (int)(resolution * 0.55);    // Más centrado
        
        for (int i = front_start; i < front_end; i++) {
            double dist = ranges[i];
            if (!isinf(dist) && !isnan(dist) && dist > 0.05) {
                if (dist < min_distance) {
                    min_distance = dist;
                }
            }
        }
        
        // Activar emergencia solo si hay obstáculo MUY cerca
        if (min_distance < emergency_threshold) {
            emergency_stop = true;
            printf("¡EMERGENCIA! Obstáculo frontal a %.2f metros\n", min_distance);
        }
        
        // Debug mejorado
        if (debug_counter % 50 == 0) {
            printf("Obstáculos detectados: %d, Min distancia frontal: %.2f\n", 
                   obstacles_detected, min_distance);
        }
        
        bool needs_replan = (path_length == 0) || (current_path_index >= path_length) || 
                           is_path_obstructed(path, path_length, current_path_index, grid);
        
        if (needs_replan && !emergency_stop) {
            Point start_grid = {
                (int)((robot_x + GRID_SIZE*CELL_SIZE/2)/CELL_SIZE), 
                (int)((robot_y + GRID_SIZE*CELL_SIZE/2)/CELL_SIZE)
            };
            
            // Debug cada 50 iteraciones
            if (debug_counter % 50 == 0) {
                print_grid(grid, start_grid, goal_grid);
            }
            debug_counter++;
            
            path_length = plan_path(grid, start_grid, goal_grid, path, MAX_PATH_LEN);
            current_path_index = 1;
            
            if (path_length == 0) {
                printf("ADVERTENCIA: No se pudo encontrar una ruta.\n");
            } else {
                printf("Nueva ruta encontrada con %d pasos.\n", path_length);
            }
        }
        
        double left_speed = 0.0, right_speed = 0.0;
        
        if (emergency_stop) {
            // Comportamiento de emergencia: parar y girar ligeramente
            left_speed = -0.5;
            right_speed = 1.0;
            printf("Ejecutando maniobra de emergencia\n");
        }
        else if (path_length > 1 && current_path_index < path_length) {
            Point next_waypoint = path[current_path_index];
            double target_x = (next_waypoint.x - GRID_SIZE / 2.0) * CELL_SIZE;
            double target_y = (next_waypoint.y - GRID_SIZE / 2.0) * CELL_SIZE;
            
            double angle_to_target = atan2(target_y - robot_y, target_x - robot_x);
            double angle_diff = atan2(sin(angle_to_target - robot_theta), cos(angle_to_target - robot_theta));
            
            // Control proporcional más suave
            double turn_speed = KP_TURN * angle_diff;
            
            // Reducir velocidad si hay obstáculos cerca pero no en emergencia
            double speed_factor = 1.0;
            if (min_distance < 0.8 && min_distance > emergency_threshold) {
                speed_factor = (min_distance - emergency_threshold) / (0.8 - emergency_threshold);
                speed_factor = fmax(0.2, speed_factor); // Mínimo 20% de velocidad
            }
            
            double forward_speed = MAX_SPEED * speed_factor * (1.0 - 0.6 * fabs(angle_diff) / M_PI);

            left_speed = forward_speed - turn_speed;
            right_speed = forward_speed + turn_speed;
            
            double dist_to_waypoint = sqrt(pow(target_x - robot_x, 2) + pow(target_y - robot_y, 2));
            if (dist_to_waypoint < CELL_SIZE * 0.8) {  // Más tolerante
                current_path_index++;
                printf("Avanzando al siguiente waypoint: %d/%d\n", current_path_index, path_length);
            }
        } else {
            // Comportamiento de búsqueda más suave cuando no hay ruta
            left_speed = -0.8;
            right_speed = 0.8;
        }

        double dist_to_goal = sqrt(pow(goal_world.x - robot_x, 2) + pow(goal_world.y - robot_y, 2));
        if (dist_to_goal < GOAL_THRESHOLD) {
            printf("\n¡OBJETIVO ALCANZADO!\n");
            left_speed = 0; right_speed = 0;
        }
        
        // Saturar velocidades para no exceder el límite de los motores
        double max_motor_speed = 9.5; // Margen de seguridad bajo el límite de 10
        
        if (left_speed > max_motor_speed) left_speed = max_motor_speed;
        if (left_speed < -max_motor_speed) left_speed = -max_motor_speed;
        if (right_speed > max_motor_speed) right_speed = max_motor_speed;
        if (right_speed < -max_motor_speed) right_speed = -max_motor_speed;
        
        wb_motor_set_velocity(wheels[0], left_speed);
        wb_motor_set_velocity(wheels[1], right_speed);
        wb_motor_set_velocity(wheels[2], left_speed);
        wb_motor_set_velocity(wheels[3], right_speed);
        
        if (dist_to_goal < GOAL_THRESHOLD) break;
    }

    wb_robot_cleanup();
    return 0;
}