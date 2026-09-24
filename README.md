# Pepper en ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04)

Migración de la simulación del robot **Pepper** (Aldebaran/SoftBank) desde
ROS 1 Kinetic/Melodic + Gazebo Classic, documentada en el
[`Pepper Tutorial V8`](Pepper%20Tutorial%20V8.docx.pdf) (proyecto RUTAS), a
**ROS 2 Jazzy + Gazebo Harmonic** sobre **Ubuntu 24.04**.

El objetivo es un **Pepper Tutorial V9**: lo mismo que el V8 (simulación, sensores,
mundos de museo, teleoperación, SLAM, navegación y YOLO), pero en ROS 2.

> **Principio del port:** se conserva la **nomenclatura ROS 1 del V8**: namespace
> `/pepper`, mismos topics, controladores y frames. Quien conozca el V8 debe
> reconocer el `ros2 topic list` del V9.

---

## Estado

| Fase | Qué | Estado |
|---|---|---|
| 0 | Entorno: paquetes, GPU, Gazebo | ✅ completada (2026-09-24) |
| 1 | Pepper en RViz2 (URDF + mallas, sin Gazebo) | ✅ completada (2026-09-24) |
| 2 | Articulaciones en Gazebo Harmonic (`gz_ros2_control`) | ⏳ siguiente |
| 3 | Base holonómica + odometría (`/pepper/cmd_vel`, `/pepper/odom`) | pendiente |
| 4 | Sensores: cámaras, profundidad, láseres, sonares, bumpers | pendiente |
| 5 | Mundos: oficina, museo, museo con personas | pendiente |
| 6 | SLAM con `slam_toolbox` (reemplaza gmapping) | pendiente |
| 7 | Navegación con Nav2 (reemplaza amcl + move_base) | pendiente |
| 8 | Percepción con `yolo_ros` (reemplaza darknet_ros) | pendiente |
| 9 | Opcionales: MoveIt 2, gente dinámica, Pepper real | opcional |

---

## Nomenclatura: V8 (ROS 1) → V9 (ROS 2)

Se mantienen los nombres del V8. Cuando ROS 2 cambia algo, se conserva el **nombre** del V8
y se indica el **tipo** equivalente de ROS 2:

| Nombre (igual que en el V8) | Tipo en ROS 1 (V8) | Tipo equivalente en ROS 2 (V9) |
|---|---|---|
| `/pepper/joint_state_controller` | `joint_state_controller/JointStateController` | `joint_state_broadcaster/JointStateBroadcaster` |
| `/pepper/LeftArm_controller`, `RightArm_controller`, `Head_controller`, `Pelvis_controller` | `velocity_controllers/JointTrajectoryController` | `joint_trajectory_controller/JointTrajectoryController` |

> En ROS 2 el nodo que publica `/pepper/joint_states` se llama por defecto
> `joint_state_broadcaster`. En el V9 se llama `joint_state_controller`, como en el V8, pero es
> el mismo `JointStateBroadcaster` de ROS 2. El topic `/pepper/joint_states` no cambia.

---

## Entorno probado

| | |
|---|---|
| SO | Ubuntu 24.04.5 LTS (nativo) |
| GPU | NVIDIA RTX 4060, driver 595.91 |
| ROS | ROS 2 Jazzy |
| Gazebo | Harmonic (gz-sim 8.15.0) |

---

## Fase 0 — Preparación del entorno

Se parte de una instalación de **ROS 2 Jazzy** (`ros-jazzy-desktop`) con `ros_gz`.

### 1. Instalar dependencias

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-moveit \
  ros-jazzy-rqt-joint-trajectory-controller ros-jazzy-rqt-robot-steering \
  mesa-utils
```

Equivalencias con el V8:

| V8 (Melodic) | V9 (Jazzy) |
|---|---|
| `ros-melodic-gazebo-ros-control` | `ros-jazzy-gz-ros2-control` |
| `ros-melodic-ros-control`, `ros-controllers` | `ros-jazzy-ros2-control`, `ros2-controllers` |
| `ros-melodic-openslam-gmapping` | `ros-jazzy-slam-toolbox` |
| `ros-melodic-navigation` (amcl, move_base, map_server) | `ros-jazzy-navigation2`, `nav2-bringup` |
| `ros-melodic-moveit*` | `ros-jazzy-moveit` |
| `ros-melodic-rqt-joint-trajectory-controller` | `ros-jazzy-rqt-joint-trajectory-controller` |
| `ros-melodic-pepper-meshes` | *no existe para Jazzy*: las mallas vienen en el workspace |

### 2. Comprobar que se renderiza por GPU

```bash
glxinfo -B | grep "OpenGL renderer"
# Esperado: OpenGL renderer string: NVIDIA GeForce RTX 4060/PCIe/SSE2
```

Si aparece `llvmpipe`, el render es por software y Gazebo irá muy lento: revisar el
driver de NVIDIA (`ubuntu-drivers autoinstall`).

### 3. Comprobar Gazebo Harmonic

```bash
source /opt/ros/jazzy/setup.bash
gz sim -r shapes.sdf
```

En otra terminal, el factor de tiempo real debe estar cerca de 1.0:

```bash
gz topic -e -t /stats -n 1 | grep real_time_factor
# Resultado en esta máquina: real_time_factor: 1.00
```

> En el V8 había que editar `~/.ignition/fuel/config.yaml` para que Gazebo 9 arrancara.
> En Gazebo Harmonic ese paso ya no es necesario.

---

## Fase 1 — Pepper en RViz2 (sin Gazebo)

Equivale a `roslaunch pepper_description display.launch` del V8.

### Estructura del workspace

El repositorio contiene los paquetes. El workspace lo enlaza:

```bash
mkdir -p ~/pepper_ws/src
cd ~/pepper_ws/src
git clone https://github.com/KevinInoCol/Pepper-in-ROS2-Jazzy-and-Ubuntu-24 pepper
sudo apt install -y ros-jazzy-joint-state-publisher-gui
cd ~/pepper_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Añadir al `~/.bashrc` (equivalente al `source $ROS_PEPPER_SIM_WS/devel/setup.bash` del V8):

```bash
source /opt/ros/jazzy/setup.bash
source ~/pepper_ws/install/setup.bash
```

### Paquete `pepper_description`

Mismo nombre de paquete que en el V8. El modelo es **el del V8**
(`marco-quiroz/pepper_robot`, rama con el árbol TF corregido): la raíz es
`base_footprint → Tibia → Pelvis → Hip → torso`, igual que en un robot móvil estándar.

| Contenido | Origen |
|---|---|
| `urdf/pepper_*.xacro` (cabeza, brazos, piernas, torso, sensores, ruedas, dedos) | V8, sin cambios salvo las rutas de mallas |
| `urdf/pepper_robot.urdf.xacro` | nuevo: equivale a `pepper_robot_CPU.xacro` sin las partes de Gazebo Classic |
| `meshes/1.0/` (mallas originales de Aldebaran) | repo de Sekkat. Sustituye a `ros-melodic-pepper-meshes`, que no existe en Jazzy |
| `meshes/HipPitch, HipRoll, Torso` | V8: mallas con el origen corregido para el árbol con raíz en `base_footprint` |

> **Por qué no el URDF de Sekkat:** sus nombres de links y joints coinciden con el V8, pero
> su árbol cinemático tiene la raíz en `base_link` (en el torso) y `base_footprint` queda como
> hoja. Así no se puede publicar la TF `odom → base_footprint` que necesitan la odometría y Nav2.

Los archivos `pepperGazebo*.xacro` y `pepperTransmission*.xacro` del V8 (Gazebo Classic)
**no se portan**. En las Fases 2 a 4 se reemplazan por equivalentes de Gazebo Harmonic.

### Lanzar

```bash
ros2 launch pepper_description display.launch.py
# Opciones: gui:=false (sin sliders), fingers:=true (con dedos, como pepper.urdf del V8)
```

Se abren RViz2 y una ventana de sliders para mover cada articulación.

### Verificación

```bash
ros2 topic list          # /pepper/joint_states, /pepper/robot_description, /tf, /tf_static
ros2 topic hz /pepper/joint_states
ros2 run tf2_ros tf2_echo base_footprint CameraTop_optical_frame
```

Resultado:
- 56 links y 55 joints (84 links con `fingers:=true`: los mismos que el `pepper.urdf` del V8).
- 20 articulaciones móviles con los nombres del V8 (`HeadYaw`, `LShoulderPitch`, `LElbowRoll`,
  `RWristYaw`, `HipPitch`, `KneePitch`, `LHand`, `WheelB`...).
- Árbol TF completo (55 transformadas). Los frames que en el V8 había que comentar para
  RViz (`WheelB/FL/FR_link`, `l_gripper`, `r_gripper`) **funcionan sin tocar nada**.
- RViz2: *Global Status: Ok*.

> Las mallas de Hip, Pelvis y Torso se ven más blancas que el resto: al corregirles el
> origen en el V8 (con meshlab) perdieron los materiales. Es estético y no afecta a nada.

---

## Referencias

- **Tutorial V8** (ROS 1): este repositorio, `Pepper Tutorial V8.docx.pdf`.
- [`awesomebytes/pepper_virtual`](https://github.com/awesomebytes/pepper_virtual) (rama
  `simulation_that_works`) y [`marco-quiroz/pepper_robot`](https://github.com/marco-quiroz/pepper_robot):
  la simulación ROS 1 del V8: **referencia de la nomenclatura y base del URDF** (`pepper_description`).
- [`HibaSekkat/pepper_ign_moveit2`](https://github.com/HibaSekkat/pepper_ign_moveit2):
  Pepper en ROS 2 + Ignition con MoveIt 2: de aquí vienen las **mallas** y las referencias de ros2_control y MoveIt.
  Sekkat et al., *"Beyond simulation: ... Pepper open-source digital twin"*, Heliyon 10(14), 2024.
- [`tuncismail/pepper-robot-ros2-gazebo-simulation`](https://github.com/tuncismail/pepper-robot-ros2-gazebo-simulation):
  referencia de parámetros de sensores y base.

## Licencias

El `pepper_description` del V8 es Apache-2.0. El código de las referencias es BSD-3 (Sekkat) y MIT (tuncismail). Las **mallas de Pepper
son de Aldebaran/SoftBank bajo CC-BY-NC-ND**: sólo uso no comercial y con atribución.
