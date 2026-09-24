# Pepper: migración de ROS Melodic a ROS 2 Jazzy + Gazebo Harmonic

Contexto de trabajo para esta carpeta. Escrito el 2026-09-22 tras la sesión de análisis
inicial (entonces en Windows + WSL). **Actualizado el 2026-09-23**: el trabajo se trasladó a
Ubuntu 24.04 nativo y las secciones de entorno (2, 3, 7, 9, 10) reflejan la máquina nueva. **Todavía no se ha escrito una sola línea de código del port**: lo que hay aquí
es el diagnóstico, y está verificado en la máquina, no supuesto.

---

## 1. Objetivo

Actualizar el trabajo con **Pepper simulado** que antes corría en
ROS Melodic + Gazebo 9 + Ubuntu 18.04 (documentado en `Pepper Tutorial V8.docx.pdf`, 26 páginas)
para que funcione en el entorno actual: **ROS 2 Jazzy + Gazebo Harmonic**.

El entregable final sería un "Pepper Tutorial V9" equivalente al V8 pero en ROS 2.

## 2. Decisiones ya tomadas

- **Se trabaja en Ubuntu 24.04 nativo** (ya no en WSL). Todo lo específico de WSL
  (`GALLIUM_DRIVER=d3d12`, `.wslconfig`, `usbipd-win`, rutas `C:\...`) queda descartado.
  Como ventaja, DDS por red y USB funcionan sin trucos: conectar un Pepper *físico* ya no
  exige cambiar de entorno.
- **Esto es un port, no una actualización.** No existe un Pepper oficial para ROS 2 + Gazebo
  moderno. Gazebo Classic está EOL desde enero de 2025.
- **Se combinan dos repos de referencia en vez de elegir uno** (ver sección 5).
- **Cada pieza se porta en su lenguaje original** (decidido 2026-09-24): lo que era C++ en el
  V8 sigue en C++ (plugins de Gazebo, `random_driver.cpp` con rclcpp, `actor_collisions`), y lo
  que era Python sigue en Python (`laser_publisher.py`, `joy_pepper.py`). Motivo: fidelidad
  al V8 y valor docente (el tutorial enseña ROS 2 en C++ y en Python), con un coste mínimo.
  **Excepción:** YOLO, porque `darknet_ros` (C) pasa a `yolo_ros` (Python, ultralytics).
  Los launch se escriben en Python (`.launch.py`), no en XML.

## 2.1 REQUISITO: conservar la nomenclatura ROS 1 del V8

Decisión del autor del V8 (2026-09-23): **el Pepper en ROS 2 debe exponer los mismos nodos,
topics, controladores y frames que el Pepper de ROS 1 Kinetic/Melodic** (`pepper_virtual`
rama `simulation_that_works` + `marco-quiroz/pepper_robot`). El repo B es base de *código*,
pero su nomenclatura **no** manda: se adapta a esta. Criterio de aceptación de cada fase:
`ros2 topic list` debe coincidir con el `rostopic list` del V8.

Extraído del código original (`pepperGazeboCPU.xacro`, `pepper_trajectory_control.yaml`,
`pepper_control_trajectory_all.launch`, `laser_publisher.py`, `arms_down.sh`):

| Qué | Nombre ROS 1 a conservar | Notas |
|---|---|---|
| Base | `/pepper/cmd_vel` (Twist) | holonómica: x, y, yaw. Límites 0.55 m/s, 2.0 rad/s |
| Odometría | `/pepper/odom`, TF `odom → base_footprint` | ruido XY 0.02, yaw 0.02645 |
| Ground truth | `/pepper/odom_groundtruth` | frame `world` |
| Cámara frontal | `/pepper/camera/front/image_raw`, `/camera_info` | 640×480, 5 Hz, hfov 1.0, frame `CameraTop_optical_frame` |
| Cámara inferior | `/pepper/camera/bottom/image_raw`, `/camera_info` | 640×480, 5 Hz, frame `CameraBottom_optical_frame` |
| Profundidad | `/pepper/camera/depth/image_raw`, `/depth/points`, `/depth/camera_info`, `/ir/image_raw` | 320×240, 30 Hz, hfov 58°, frame `CameraDepth_optical_frame` |
| Láseres | `/pepper/scan_front`, `/pepper/scan_left`, `/pepper/scan_right` | 15 muestras, ±30°, 0.3–7 m, 6.25 Hz |
| Láser fusionado | `/pepper/laser_2` | lo publica `laser_publisher.py` (portarlo) |
| Hokuyo falso | `/pepper/hokuyo_scan` | 720 muestras, ±90°, 0.1–30 m |
| Sonares | `/pepper/sonar_front`, `/pepper/sonar_back` (Range) | 0–5 m, 20 Hz |
| Bumpers | `/pepper/Bumper/Back`, `/FrontLeft`, `/FrontRight` | en gz el tipo cambia a `ros_gz_interfaces/Contacts` |
| Estados | `/pepper/joint_states` | 50 Hz |
| Controladores | `/pepper/LeftArm_controller`, `/pepper/RightArm_controller`, `/pepper/Head_controller`, `/pepper/Pelvis_controller`, `/pepper/joint_state_controller` | + `LeftHand_controller`/`RightHand_controller` (estaban comentados en V8) |
| Mover brazos | `/pepper/LeftArm_controller/command` (JointTrajectory) | así lo usa `arms_down.sh` |

Articulaciones por controlador (**idénticas en el repo B**, no hay que renombrar joints):
`LeftArm` = LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw ·
`RightArm` = RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw ·
`Head` = HeadYaw, HeadPitch · `Pelvis` = HipRoll, HipPitch, KneePitch · manos = LHand, RHand.

**Qué hay que cambiar en el repo B para cumplirlo:**
- Namespace `/pepper` para todo (controller_manager, bridge, robot_state_publisher).
- Renombrar controladores: `left_arm_controller`→`LeftArm_controller`,
  `right_arm_controller`→`RightArm_controller`, `head_controller`→`Head_controller`,
  `torso_controller`→`Pelvis_controller`, `left/right_hand_controller`→`LeftHand/RightHand_controller`.
  Actualizar también `moveit_controller_manager.yaml` para que MoveIt los encuentre.
- En ROS 2 el JointTrajectoryController escucha en `~/joint_trajectory`, no en `~/command`.
  **Hecho:** el spawner lo remapea con `--controller-ros-args "-r ~/joint_trajectory:=~/command"`
  (`pepper_control_trajectory_all.launch.py`). rqt_joint_trajectory_controller tiene
  `joint_trajectory` fijo en el código: se lanza con `pepper_control/launch/rqt_joint_trajectory_controller.launch.py`,
  que remapea sus publishers a `/command` y `robot_description` a `/pepper/robot_description`.
- **Decidido (2026-09-24):** el controlador se llama `joint_state_controller` (nombre V8),
  con tipo `joint_state_broadcaster/JointStateBroadcaster` (su equivalente en ROS 2).
  Documentado en la tabla de nomenclatura del README.
- Los sensores (Fase 3) se crean directamente con estos nombres vía `ros_gz_bridge`.

## 3. Entorno verificado en esta máquina (2026-09-23)

| | |
|---|---|
| SO | **Ubuntu 24.04.5 LTS nativo**, kernel 7.0.0-34-generic, sesión X11 |
| Hardware | 12 hilos de CPU, 15 GB RAM, **NVIDIA RTX 4060** (driver propietario open-kernel 595.91.07) |
| Disco | ~425 GB libres |
| ROS | **Jazzy** en `/opt/ros/jazzy` (no se hace `source` automático: `source /opt/ros/jazzy/setup.bash`) |
| Gazebo | **Harmonic** — `gz-sim 8.15.0` vía paquetes `ros-jazzy-gz-*-vendor`, con `ros_gz` |

**Ya instalado:** `ros_gz`, `joy`, `teleop_twist_joy`, `xacro`, `robot_state_publisher` y, desde
la Fase 0 (2026-09-24), todo lo de la lista siguiente.

**Instalado en la Fase 0 con:**

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit \
  ros-jazzy-rqt-joint-trajectory-controller ros-jazzy-rqt-robot-steering mesa-utils
```

**NO existe para Jazzy:** `ros-jazzy-pepper-meshes`, `ros-jazzy-naoqi-driver`.
Las mallas hay que traerlas de un repo (ver sección 5).

### 3.1 Verificaciones pendientes del entorno (Fase 0)

- **GPU:** en nativo no hace falta ningún arreglo (**no** poner `GALLIUM_DRIVER=d3d12`, era
  solo para WSL). Falta confirmar que se renderiza por hardware:
  `glxinfo -B` (paquete `mesa-utils`) debe mostrar `NVIDIA GeForce RTX 4060`, y
  `gz sim shapes.sdf` debe ir fluido.
- **RAM:** sin acción; los 15 GB están disponibles para Gazebo + Nav2 + RViz2 + YOLO.

## 4. Punto de partida: qué cubre el Tutorial V8

El PDF es de Google Docs. El texto se extrae con `pdftotext -layout` (poppler, ya instalado),
pero **varias correcciones solo están en capturas de pantalla** (el `config.yaml` de Fuel,
las líneas que se comentan en los xacro y en `pepper_arms.xacro`, la configuración de darknet):
hay que mirar las imágenes del PDF. Además, **los archivos que el V8 da por adjuntos no
están en este repo** (carpetas `launch/`, `worlds/` y "Files to Dynamic World", el zip de
navegación, `random_driver.cpp`, `joy_pepper.py`); para portar los mundos del museo habrá
que conseguirlos. Contenido:

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
   extensiones URDF de Classic. **Corrección (2026-09-24, verificado con `gz sdf -p`):**
   el conversor URDF→SDF de sdformat **sí** traduce `mu1`/`mu2` a `<surface><friction>`,
   incluso en colisiones fusionadas por fixed joints. El resto (`turnGravityOff`,
   `selfCollide`...) no se ha verificado. En Sekkat la fricción de las ruedas es además
   inconsistente entre WheelFL y WheelFR.

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
| `joy_node` + `joy_pepper.py` | `joy` + `teleop_twist_joy` (USB directo en `/dev/input/jsX`) | fácil |
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

## 7. Plan por fases (acordado 2026-09-24)

Regla: **cada fase termina con una prueba verificable contra el V8**, y la nomenclatura
ROS 1 del §2.1 se aplica **desde la Fase 1** (no se renombra al final).

| Fase | Qué | Prueba de aceptación | Estado |
|---|---|---|---|
| **0. Entorno** | Instalar paquetes pendientes (§3). | `glxinfo -B` muestra la RTX 4060; `gz sim shapes.sdf` fluido. | ✅ 2026-09-24: paquetes instalados, renderer NVIDIA RTX 4060 (OpenGL 4.6), `gz sim gui` en GPU, RTF 1.00 |
| **1. Pepper en RViz2** | Crear `~/pepper_ws`; traer URDF + mallas del repo B y portarlo a Jazzy, **ya con namespace `/pepper`** y frames del V8. `ros2_control_plugin:=fake`, sin Gazebo. | Pepper completo en RViz2, TF sin errores (incl. `WheelB/FL/FR_link`, `l/r_gripper`), `/pepper/joint_states` publicándose. | ✅ 2026-09-24 (ver abajo) |
| **2. Articulaciones en Harmonic** | Spawn en gz-sim + `gz_ros2_control` con controladores **ya renombrados** (`LeftArm_controller`, `RightArm_controller`, `Head_controller`, `Pelvis_controller`). Portar `arms_down.sh`. | `ros2 topic pub /pepper/LeftArm_controller/command ...` baja el brazo; `rqt_joint_trajectory_controller` funciona. | ✅ 2026-09-24 (ver abajo) |
| **3. Base holonómica + odometría** | Equivalente a `gazebo_model_velocity_plugin` (que mueve el modelo, no simula ruedas): evaluar `VelocityControl` + `OdometryPublisher` de gz-sim. Límites y ruido del V8 (0.55 m/s, 2 rad/s, ruido 0.02 / 0.02645). Portar `random_driver.cpp` (rclcpp) y `joy_pepper.py`. | `rqt_robot_steering` sobre `/pepper/cmd_vel` mueve en x, y, yaw; `/pepper/odom` + TF y `/pepper/odom_groundtruth` publicándose. | 🟡 base y odometría ✅ 2026-09-24; faltan `random_driver` y `joy_pepper` |
| **4. Sensores** | `<sensor>` gz-sim + `ros_gz_bridge` con nombres y parámetros del §2.1 (repo C solo como apoyo): cámaras front/bottom → profundidad → 3 láseres + hokuyo → sonares → bumpers. Portar `laser_publisher.py` (`/pepper/laser_2`). Convertir `pepper_sensors.rviz` a RViz2. | `ros2 topic list` coincide con el `rostopic list` del V8; la vista de sensores en RViz2 equivale a la del V8. | pendiente |
| **5. Mundos** | Oficina (`simple_office_with_people.world`, está en `pepper_virtual`) → museo → museo con personas y robots. SDF Classic → SDF Harmonic. | Los launch con los nombres del V8 (`pepper_gazebo_plugin_museum...`) abren el mundo con Pepper. | pendiente |
| **6. SLAM** | `slam_toolbox` sobre `/pepper/laser_2` o `/pepper/hokuyo_scan`, parámetros equivalentes a los de gmapping del V8. | Mapa del museo guardado con `map_saver`. | pendiente |
| **7. Navegación** | Nav2 (sustituye amcl + move_base) sobre el mapa de la Fase 6. | Objetivo enviado desde RViz2 alcanzado. | pendiente |
| **8. Percepción** | `yolo_ros` (YOLOv8/v11) sobre `/pepper/camera/front/image_raw`. | Detecta personas en el mundo oficina, como la figura del V8. | pendiente |
| **9. Opcionales** | MoveIt 2 (viene en repo B; adaptar a los nombres nuevos), gente dinámica (actores / HuNavSim), Pepper real (`naoqi_driver2`). | — | opcional |

Cambios respecto al plan del 2026-09-22:
1. La nomenclatura ROS 1 se aplica desde la Fase 1 (renombrar al final obligaría a tocar
   launch, YAML, MoveIt y bridge dos veces).
2. La base holonómica tiene fase propia, antes que los sensores: sin mover a Pepper no se
   prueban bien los sensores. Posiblemente es más fácil de lo estimado (§6 decía "no trivial").
3. MoveIt 2 pasa a opcional: no formaba parte del V8 y es lo más costoso de portar
   (`MoveItConfigsBuilder`). El control de brazos del V8 (JointTrajectory) se cubre en la Fase 2.

**Resultado de la Fase 1 (2026-09-24), con decisiones que afectan a lo demás:**
- El paquete se llama `pepper_description` (decisión del autor: **nombres de paquete del V8**).
  Está en este repo; `~/pepper_ws/src/pepper` es un symlink al repo.
- **El URDF base es el del V8, no el de Sekkat.** Los nombres de links y joints coinciden,
  pero Sekkat invirtió el árbol: raíz `base_link` y `base_footprint` como hoja. Con eso no se
  puede publicar `odom → base_footprint`. El V8 (`marco-quiroz/pepper_robot`) tiene la raíz
  en `base_footprint`. De Sekkat solo se toman las mallas (`meshes/1.0/`). Las 6 mallas
  Hip/Pelvis/Torso con el origen corregido vienen del V8.
- Consecuencia para la Fase 2: los `.ros2_control`, `.gazebo` y la config de MoveIt de Sekkat
  se pueden reutilizar (los joints se llaman igual), pero el `<ros2_control>` se escribe sobre
  el URDF del V8. `pepperGazebo*.xacro` y `pepperTransmission*.xacro` del V8 no se portan.
- Dedos opcionales (`fingers:=false` por defecto, como `pepper_robot_CPU.xacro` del V8).

**Resultado de la Fase 2 (2026-09-24):**
- Paquetes `pepper_control` y `pepper_gazebo_plugin` (nombres del V8). Launch:
  `pepper_gazebo_plugin_empty.launch.py`, que hace el spawn como `pepper_MP` y lanza `arms_down.sh`.
- `pepper_description/urdf/pepper_ros2_control.xacro` (se activa con `gazebo:=true`): 17 joints
  (cabeza, pelvis, brazos y manos) con interfaz de posición. Las ruedas quedan fuera.
  Plugin `gz_ros2_control-system` con `<ros><namespace>/pepper</namespace></ros>`.
- Un hook de entorno de `pepper_description` añade `share/` a `GZ_SIM_RESOURCE_PATH`, para que
  Gazebo resuelva `package://pepper_description/meshes/...`.
- **DART aborta** (`dLDLTRemove`) con la masa del V8 en `l/r_gripper` (2e-06). Se subió a
  0.05 kg. `LHand`/`RHand` arrancan en 0.5 (su límite es 0.02–0.98).
- Verificación de GUI sin xdotool: clics y arrastres con `libXtst` vía ctypes, y capturas con
  `Gdk.pixbuf_get_from_window` (scripts temporales de la sesión; se rehacen en pocas líneas). Para parar la simulación no usar `pkill -f <patrón>` dentro de un comando
  que contenga ese patrón, porque se mata a sí mismo.

**Resultado de la Fase 3 (base y odometría, 2026-09-24):**
- Paquete C++ `gazebo_model_velocity_plugin` (mismo nombre que en el V8) con dos sistemas
  de gz-sim que hablan ROS 2 directamente (rclcpp dentro del plugin, como el original):
  `GazeboRosModelVelocity` (port del plugin de awesomebytes: mismos parámetros SDF,
  limitador de velocidad, aceleración y jerk, timeout, odometría con ruido integrada y TF
  `odom → base_footprint`) y `GazeboRosP3D` (equivale a `libgazebo_ros_p3d.so`:
  `/pepper/odom_groundtruth`). Un hook añade `lib/` a `GZ_SIM_SYSTEM_PLUGIN_PATH`.
  Bloques SDF en `pepper_description/urdf/pepper_gazebo.xacro`, copiados del V8.
- **La física de gz-sim descarta `LinearVelocityCmd`/`AngularVelocityCmd` del modelo tras
  cada paso**: hay que reaplicarlos en todos los PreUpdate. Aplicándolos solo a
  `<updateRate>` (50 Hz con pasos de 1 ms) el robot se movía 1/20 de lo pedido.
- Fricción 0 en la colisión de `Tibia` (`mu1`/`mu2`). Sin ella, el roce reduce el giro ~35%.
- Bugs del V8 corregidos: signo de `linear.y` invertido (+y iba a la derecha) y fórmula de
  arco de la odometría incorrecta para movimiento lateral con giro.
- `rqt_robot_steering` de Jazzy tiene una casilla "stamped" (TwistStamped); por defecto
  viene desmarcada, así que publica `Twist`, compatible con el V8.

**Pendiente de respuesta del autor del V8:**
- ¿Tiene los archivos del V8 que no están en el repo? (`museum.world`,
  `museum_with_persons_robots`, `museum_with_people_moving.world`, launch, zip de
  `pepper_nav`, `random_driver.cpp`, `joy_pepper.py`). Necesarios en las Fases 3, 5 y 7.

## 8. Licencias

El código de B es BSD-3 y el de C es MIT, pero **las mallas de Pepper son de
Aldebaran/SoftBank bajo CC-BY-NC-ND** en ambos. Uso docente correcto; conviene citarlo.
El paper de Heliyon pide cita explícita (`@article{sekkat2024beyond, ...}`).

## 9. Estado actual

Máquina migrada a Ubuntu 24.04 nativo; repo clonado en
`~/Proyectos-Robotica/Pepper-in-ROS2-Jazzy-and-Ubuntu-24`. Workspace `~/pepper_ws` creado; `src/pepper` es un symlink a este repo.
Plan por fases reescrito el 2026-09-24 (§7). **Fase 0 completada** el 2026-09-24.
**Fases 1 y 2 completadas** el 2026-09-24. Siguiente paso: **Fase 3** (base holonómica).

`README.md` es el borrador vivo del Tutorial V9: **actualizarlo al cerrar cada fase**
(estado + pasos reproducibles + equivalencias con el V8). Petición explícita del autor.

## 10. Contexto del curso

Esta carpeta vive en `~/Proyectos-Robotica/`. En Windows convivía con el curso en
`C:\Curso Robotica` (línea de trabajo con **CoppeliaSim**: `escenas/`, `scripts/`, `ros2/`,
`Tarea 2/`), que es independiente de este port y no se ha traído a esta máquina.
