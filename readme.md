# Navegación Autónoma de Robot 4WD en Webots

Este proyecto presenta la simulación de un robot móvil de 4 ruedas (4WD) capaz de navegar de forma autónoma en un entorno con obstáculos. El robot utiliza un sensor LIDAR para mapear su entorno, el algoritmo A* para planificar la ruta más corta y sensores de distancia para una evasión de colisiones reactiva.

## Índice
1. [Características Principales](#características-principales)
2. [Requisitos Previos](#requisitos-previos)
3. [Instalación](#instalación)
4. [Cómo Ejecutar la Simulación](#cómo-ejecutar-la-simulación)
5. [Estructura del Proyecto](#estructura-del-proyecto)

## Características Principales
- **Navegación Autónoma:** El robot se desplaza de un punto de inicio a un punto objetivo definido en el código.
- **Mapeo en Tiempo Real:** Construye y actualiza una grilla de ocupación de 8x8 del entorno utilizando datos del sensor LIDAR.
- **Planificación de Rutas con A***: Calcula la ruta óptima en el mapa actual para evitar obstáculos de manera eficiente.
- **Evasión de Obstáculos Reactiva:** Utiliza dos sensores de distancia frontales como una capa de seguridad para evitar colisiones inminentes, sobreponiéndose al plan de A* si es necesario.
- **Control Skid-Steer:** Implementación de control de movimiento para una plataforma de 4 ruedas.

## Requisitos Previos

Asegúrate de tener instalado el siguiente software antes de comenzar:

- **Webots:** Versión **R2023b** o superior.
  - [Página de descarga oficial de Webots](https://cyberbotics.com/download)
- **C**
## Instalación

Sigue estos pasos para configurar el proyecto en tu máquina local.

1.  **Clonar el repositorio:**
    Abre una terminal o Git Bash y ejecuta el siguiente comando:
    ```bash
    git clone [https://github.com/tu_usuario/webots-robot-navegacion.git](https://github.com/tu_usuario/webots-robot-navegacion.git)
    ```
    *(Reemplaza la URL con la de tu repositorio real cuando lo tengas)*

2.  **Navegar al directorio del proyecto:**
    ```bash
    cd webots-robot-navegacion
    ```

## Cómo Ejecutar la Simulación

Para ver al robot en acción, sigue estas instrucciones:

1.  **Abrir Webots:** Inicia la aplicación Webots en tu sistema.

2.  **Abrir el Mundo de Simulación:**
    - En Webots, ve al menú `File` > `Open World...`.
    - Navega hasta el directorio donde clonaste el proyecto.
    - Selecciona el archivo del mundo: `worlds/mi_entorno.wbt` y haz clic en `Open`.

3.  **Iniciar la Simulación:**
    - Una vez que el mundo se haya cargado, verás el entorno con el robot y los obstáculos.
    - Para iniciar la simulación, presiona el botón **"Real-time" (play)** en la barra de herramientas superior (el ícono `▶️`).

4.  **Observar el Comportamiento:**
    - La simulación comenzará.
    - La **consola de Webots** mostrará los mensajes generados por el controlador del robot, como la grilla de ocupación actualizada y el estado actual (ej. "Planificando ruta...", "Evasión de obstáculo activada").
    - El robot debería comenzar a moverse, evitando los obstáculos en su camino hacia el punto objetivo.

## Estructura del Proyecto

El repositorio está organizado siguiendo las convenciones estándar de Webots:
/
├── controllers/
│   └── mi_controlador/
│       ├── mi_controlador.c  <-- Lógica principal del robot
│       
│
├── worlds/
│   └── mi_entorno.wbt         <-- Archivo del mundo de Webots con el entorno y el robot
│
├── protos/
│   └── MiRobot4WD.proto       <-- Definición del prototipo del robot
│
└── README.md                  <-- Este archivo de instrucciones

- **`controllers/`**: Contiene el código fuente que controla el comportamiento del robot.
- **`worlds/`**: Contiene los archivos `.wbt` que definen los entornos de simulación.
- **`protos/`**: (Opcional) Contiene las definiciones de nodos de robot personalizados, si se crean.

