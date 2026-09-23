# Pepper: migración de ROS Melodic a ROS 2 Jazzy + Gazebo Harmonic

Contexto de trabajo para esta carpeta. Escrito el 2026-09-22 tras la sesión de análisis
inicial. **Todavía no se ha escrito una sola línea de código del port**: lo que hay aquí
es el diagnóstico, y está verificado en la máquina, no supuesto.

---

## 1. Objetivo

Actualizar el trabajo con **Pepper simulado** que antes corría en
ROS Melodic + Gazebo 9 + Ubuntu 18.04 (documentado en `Pepper Tutorial V8.docx.pdf`, 26 páginas)
para que funcione en el entorno actual: **ROS 2 Jazzy + Gazebo Harmonic**.

El entregable final sería un "Pepper Tutorial V9" equivalente al V8 pero en ROS 2.

## 2. Decisiones ya tomadas

- **Se trabaja en WSL, no en dual boot.** El WSL ya tiene todo instalado y la GPU es accesible.
  El dual boot solo se reconsiderará si algún día se conecta un Pepper *físico* (DDS por red
  y USB son los puntos donde WSL sí estorba).
- **Esto es un port, no una actualización.** No existe un Pepper oficial para ROS 2 + Gazebo
  moderno. Gazebo Classic está EOL desde enero de 2025.
- **Se combinan dos repos de referencia en vez de elegir uno** (ver sección 5).

## 3. Entorno verificado en esta máquina

| | |
|---|---|
| Host | Windows 11, 16 GB RAM, **NVIDIA RTX 4060** (driver 591.86) |
| WSL | Ubuntu 24.04.4 LTS (única distro), WSL2 |
| ROS | **Jazzy** en `/opt/ros/jazzy` |
| Gazebo | **Harmonic** — `gz-sim 8.11.0` vía paquetes `ros-jazzy-gz-*-vendor`, con `ros_gz` |

**Ya instalado:** `slam_toolbox`, `navigation2`, `nav2_bringup`, `joy`, `teleop_twist_joy`,
`xacro`, `robot_state_publisher`.

**Disponible por apt, aún sin instalar:** `ros-jazzy-gz-ros2-control`, `ros-jazzy-ros2-control`,
`ros-jazzy-ros2-controllers`, `ros-jazzy-rqt-joint-trajectory-controller`,
`ros-jazzy-rqt-robot-steering`.

**NO existe para Jazzy:** `ros-jazzy-pepper-meshes`, `ros-jazzy-naoqi-driver`.
Las mallas hay que traerlas de un repo (ver sección 5).

### 3.1 Dos arreglos pendientes del entorno (Fase 0)

**GPU — crítico.** WSL está renderizando por software (`llvmpipe`, `Accelerated: no`) pese a
tener la RTX 4060 y `/dev/dxg` presente. Gazebo iría a pocos fps y parecería culpa de WSL.
Comprobado en vivo que esto lo arregla:

```bash
echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
# verificación: glxinfo -B  ->  Device: D3D12 (NVIDIA GeForce RTX 4060) / Accelerated: yes
```

**RAM.** WSL solo recibe 7.7 GB de los 16 GB y no existe `C:\Users\zeus\.wslconfig`.
Gazebo + Nav2 + RViz2 + YOLO simultáneos se quedan cortos. Crear el fichero con `memory=12GB`.

## 4. Punto de partida: qué cubre el Tutorial V8

El PDF es de Google Docs y **necesita `pypdf` para extraer texto** (en WSL no hay `pip`;
usar el Python de Windows: `python -m pip install pypdf`). Contenido:

1. Instalación Kinetic/Melodic + `pepper_virtual` / `pepper_robot` (forks de `awesomebytes`)
2. Pepper real vía NAOqi SDK (pynaoqi 2.5.5, **Python 2.7**) + `naoqi_bridge`
3. Cuatro mundos de Gazebo: oficina, museo, museo+personas+robots, museo con gente en
   movimiento (`actor_collisions`) y con `pedsim_ros` (clusters de agentes)
4. Operación: `rqt_robot_steering`, `rqt_joint_trajectory_controller`, `/pepper/cmd_vel`,
   `random_driver.cpp`, joystick (`joy_pepper.py`), mapa 2D con pygame
5. SLAM: **gmapping** sobre `/pepper/laser_2` o `/pepper/hokuyo_scan`, `map_server`
6. Navegación: `pepper_nav`, **amcl**, **move_base**
7. Percepción: **darknet_ros / YOLOv2** sobre `/pepper/camera/front/image_raw`

Parches manuales que el V8 documenta y que ya no aplican: `math::Vector3` →
`ignition::math::Vector3d` en `gazebo_ros_model_velocity.cpp`, comentar los `side` en los
xacro, comentar `WheelB/FL/FR_link` y `l_gripper`/`r_gripper` para RViz.

## 5. Los tres repos relevantes (analizados a fondo)

### A. `ros-naoqi/pepper_virtual` + `pepper_robot` — la base actual del V8

ROS 1 catkin + Gazebo Classic. **Sin puerto oficial a ROS 2.** Sirve solo como
documentación de referencia (nombres de topics, parámetros de sensores, mundos).

### B. `HibaSekkat/pepper_ign_moveit2` — LA BASE ELEGIDA

Código del paper *Sekkat et al., "Beyond simulation: ...Pepper open-source digital twin",
Heliyon 10(14), 2024*. BSD-3. Calco estructural de `AndrejOrsula/panda_ign_moveit2`.
Apunta a **Galactic + Fortress**; último push sep-2024, 10 estrellas, sin mantenimiento.

**Lo que aporta (lo caro, ya resuelto):**

- Ya vive en la familia **gz-sim**. Fortress→Harmonic es `gz-sim 6→8`: mismo planeta.
- **116 mallas incluidas**: 64 `.stl` de colisión + 48 `.dae` visuales + 3 texturas.
  Resuelve la ausencia de `ros-jazzy-pepper-meshes`.
- Xacro modular (`arms`, `fingers`, `head`, `legs`, `torso`, `wheels`, `sensors`,
  `visual_collisions`) con `ros2_control_plugin` = `fake | ign | real`.
  Con `fake` se ve a Pepper en RViz2 **sin Gazebo**.
- `ros2_control` con **55 articulaciones** y 6 controladores: `head`, `torso`,
  `left_arm`, `right_arm`, `left_hand`, `right_hand` + `joint_state_broadcaster`.
- **MoveIt 2 completo**: SRDF con grupos (`left_arm`, `right_arm`, `both_arms`, manos,
  `head`, `torso`), poses `down`/`open`/`closed`, OMPL, kinematics, joint_limits, servo.
- Docker + devcontainer + scripts `xacro2sdf` / `xacro2urdf`.

**Los tres agujeros (verificados leyendo el código):**

1. **Cero sensores simulados.** `pepper_sensors.xacro` tiene 30+ links y fixed joints y
   **0 etiquetas `<sensor>`** — son solo frames de TF. El `bridge.launch.py` puentea
   únicamente: `clock`, `joint_state`, `joint_trajectory`, `joint_trajectory_progress`,
   `pose`, `odom`, `odom_tf`, `cmd_vel`. Sin cámara, profundidad, láser, sonar ni IMU.
   **Sin `/scan` no hay slam_toolbox; sin `/image_raw` no hay YOLO.**
2. **La base está rota.** Usa `DiffDrive` con `<back_joint>WheelB</back_joint>`, y
   `back_joint` **no es un parámetro que DiffDrive reconozca**: se ignora en silencio.
   Queda un diferencial de 2 ruedas con una tercera arrastrando, cuando el Pepper real
   es **holonómico**. Las ruedas están además comentadas fuera de `ros2_control`.
3. **Herencia muerta de Gazebo Classic.** El macro `gazebo_misc_references` define
   `mu1/mu2/kp/kd/fdir1/minDepth/turnGravityOff/selfCollide` para ~50 links: son
   extensiones URDF de Classic que **gz-sim ignora**. La fricción de las ruedas que
   *parece* configurada no hace nada (y encima es inconsistente entre WheelFL y WheelFR).

Alcance: es un repo de **manipulación**, no de navegación. Sin Nav2, sin SLAM,
sin mundos tipo museo (solo `follow_target.sdf`), sin YOLO.

### C. `tuncismail/pepper-robot-ros2-gazebo-simulation` — REFERENCIA, no base

ROS 2 **Humble** + Gazebo **Classic 11**, Docker, MIT. Tiene justo lo que a B le falta:
cámaras RGB front/bottom, depth tipo ASUS Xtion, 3 láseres infrarrojos fusionados,
Hokuyo falso de 180°, sonar delantero/trasero, plugin C++ de base **omnidireccional con
odometría calibrada al Pepper real**, y Nav2 tuneado. Está en Gazebo Classic, así que
**no se usa su código**: se le copian los *parámetros físicos* (FOV, rango, ruido,
cinemática de la base), que sí se traducen.

## 6. Mapa de migración del V8

| Tutorial V8 (Melodic) | Jazzy + Harmonic | Esfuerzo |
|---|---|---|
| `catkin_make`, `pepper_sim_ws` | `colcon build`, ament | trivial |
| URDF/xacro de `pepper_description` | **ya hecho por el repo B** | resuelto |
| `gazebo_ros_control` | `gz_ros2_control` | resuelto por B, falta renombrar |
| `gazebo_model_velocity_plugin` | base holonómica en gz-sim | **pendiente, no trivial** |
| `ros-melodic-pepper-meshes` | mallas del repo B | resuelto |
| Sensores (cámara, depth, láseres) | `<sensor>` de gz-sim + `ros_gz_bridge` | **pendiente, pieza crítica** |
| `.world` museo, actores | SDF Classic → SDF 1.10 Harmonic | pendiente |
| `pedsim_ros` (clusters) | evaluar HuNavSim para Harmonic | pendiente, incierto |
| **gmapping** | **slam_toolbox** (ya instalado, hay skill del curso) | fácil, y mejora |
| **map_server + amcl + move_base** | **Nav2** (ya instalado) | media, y mejora mucho |
| `rqt_robot_steering`, `rqt_joint_trajectory_controller` | existen igual | trivial |
| `random_driver.cpp` (roscpp) | rclcpp | fácil |
| `joy_node` + `joy_pepper.py` | `joy` + `teleop_twist_joy` (USB necesita `usbipd-win`) | fácil |
| **darknet_ros / YOLOv2** | `yolo_ros` con YOLOv8/v11 | reescritura, más simple |
| `odom_graph_test.launch` + pygame | lo cubre RViz2/Nav2 | se elimina |
| pynaoqi 2.7 + `naoqi_bridge` | `naoqi_driver2` (Humble/Iron; Jazzy en PRs #17/#18) | dejar para el final |

### Cambios concretos Galactic/Fortress → Jazzy/Harmonic en el repo B

- `ign_ros2_control::IgnitionROS2ControlPlugin` → `gz_ros2_control::GazeboSimROS2ControlPlugin`;
  filename `ign_ros2_control-system` → `gz_ros2_control-system`
- `ignition::gazebo::systems::*` → `gz::sim::systems::*`;
  `ignition-gazebo-*-system` → `gz-sim-*-system`
- `ignition.msgs.*` → `gz.msgs.*` en los 8 puentes de `bridge.launch.py`
- `ros_ign` / `ros_ign_gazebo` → `ros_gz` / `ros_gz_sim`
- En `pepper_robot.repos`: quitar el fork `AndrejOrsula/ros2_controllers` rama `jtc_effort`
  (**ya no hace falta**, en Jazzy el JointTrajectoryController soporta `effort` de serie);
  `gz_ros2_control` rama `galactic` → deb `ros-jazzy-gz-ros2-control`; revisar `pymoveit2`
- `move_group.launch.py` → reescribir con `MoveItConfigsBuilder` (`moveit_configs_utils`).
  **Es la parte que más guerra va a dar.**
- `follow_target.sdf`: nombres de plugins y URIs de Fuel

## 7. Plan por fases

- **Fase 0** — arreglar GPU (`GALLIUM_DRIVER=d3d12`) y RAM (`.wslconfig`), verificar `gz sim`.
- **Fase 1** — clonar repo B, `xacro` + `robot_state_publisher` + RViz2 con
  `ros2_control_plugin:=fake`. **Prueba de fuego más barata: ¿sobrevive el URDF a Jazzy?**
- **Fase 2** — Pepper spawneado en Harmonic, `gz_ros2_control` moviendo cabeza/brazos,
  `cmd_vel` moviendo la base (aquí se arregla lo holonómico).
- **Fase 3** — **sensores**: añadir `<sensor>` gz-sim + `ros_gz_bridge`, parámetros copiados
  del repo C. *Esta es ahora la pieza crítica del proyecto.*
- **Fase 4** — mundo museo.
- **Fase 5** — slam_toolbox + Nav2.
- **Fase 6** — YOLO (`yolo_ros`).
- **Fase 7** (opcional) — gente dinámica (HuNavSim) y/o Pepper real (`naoqi_driver2`).

Nota: el análisis del repo B **reordenó el plan**. Las fases 1 y 2 bajaron de "el grueso
del trabajo" a "port mecánico"; el esfuerzo se desplazó a la Fase 3.

## 8. Licencias

El código de B es BSD-3 y el de C es MIT, pero **las mallas de Pepper son de
Aldebaran/SoftBank bajo CC-BY-NC-ND** en ambos. Uso docente correcto; conviene citarlo.
El paper de Heliyon pide cita explícita (`@article{sekkat2024beyond, ...}`).

## 9. Estado actual

Fase 0 **no ejecutada todavía**. Nada clonado, ningún workspace creado.
El siguiente paso acordado es la Fase 1 (prueba de fuego del URDF en RViz2).

## 10. Contexto del curso

Esta carpeta convive con el resto de `C:\Curso Robotica`, que usa **CoppeliaSim**
(`escenas/`, `scripts/`, `ros2/`, `Tarea 2/`) — otra línea de trabajo distinta.
Hay un MCP de CoppeliaSim configurado que en esta sesión no conectó.
