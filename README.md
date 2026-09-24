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
| 2 | Articulaciones en Gazebo Harmonic (`gz_ros2_control`) | ✅ completada (2026-09-24) |
| 3 | Base holonómica + odometría (`/pepper/cmd_vel`, `/pepper/odom`) | 🟡 base y odometría ✅; faltan `random_driver` y joystick |
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
| `/pepper/LeftHand_controller`, `RightHand_controller` | (comentados en el V8) | `joint_trajectory_controller/JointTrajectoryController` |
| `/pepper/<X>_controller/command` | topic nativo del controlador | el nativo en ROS 2 es `~/joint_trajectory`; se remapea a `~/command` |
| `gazebo_ros_control` (`libgazebo_ros_control.so`) | plugin de Gazebo Classic | `gz_ros2_control::GazeboSimROS2ControlPlugin` |
| `pepperTransmission.xacro` | `<transmission>` | bloque `<ros2_control>` (`pepper_ros2_control.xacro`) |
| `libgazebo_ros_model_velocity.so` (`gazebo_model_velocity_plugin`) | plugin de Gazebo Classic (C++) | `gazebo_model_velocity_plugin::GazeboRosModelVelocity`, sistema de gz-sim (C++), mismo paquete y mismos parámetros |
| `libgazebo_ros_p3d.so` (`gazebo_plugins`) | plugin de Gazebo Classic (C++) | `gazebo_model_velocity_plugin::GazeboRosP3D`, sistema de gz-sim (C++) |

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

![Pepper en RViz2](docs/img/fase1_rviz2.png)

> Las mallas de Hip, Pelvis y Torso se ven más blancas que el resto: al corregirles el
> origen en el V8 (con meshlab) perdieron los materiales. Es estético y no afecta a nada.

---

## Fase 2 — Pepper en Gazebo Harmonic moviendo articulaciones

Equivale a `roslaunch pepper_gazebo_plugin pepper_gazebo_plugin_in_office_CPU.launch` del V8,
por ahora en un mundo vacío (los mundos llegan en la Fase 5).

### Paquetes nuevos (nombres del V8)

| Paquete | Contenido | Equivale en el V8 a |
|---|---|---|
| `pepper_control` | `config/pepper_trajectory_control.yaml`, `launch/pepper_control_trajectory_all.launch.py`, `launch/rqt_joint_trajectory_controller.launch.py` | `pepper_virtual/pepper_control` |
| `pepper_gazebo_plugin` | `launch/pepper_gazebo_plugin_empty.launch.py`, `worlds/empty.world`, `scripts/arms_down.sh` | `pepper_virtual/pepper_gazebo_plugin` |

En `pepper_description` se añadió `urdf/pepper_ros2_control.xacro`, que sustituye a
`pepperGazebo*.xacro` y a `pepperTransmission*.xacro`. Se activa con `gazebo:=true`.

### Lanzar

```bash
ros2 launch pepper_gazebo_plugin pepper_gazebo_plugin_empty.launch.py
```

El launch hace lo mismo que el del V8:
1. Abre Gazebo con el mundo.
2. Hace el spawn del robot con el nombre de modelo **`pepper_MP`**.
3. Arranca los controladores (`pepper_control_trajectory_all.launch.py`).
4. Baja los brazos (`arms_down.sh`).

### Controladores

```bash
ros2 control list_controllers -c /pepper/controller_manager
```

```
joint_state_controller joint_state_broadcaster/JointStateBroadcaster          active
LeftArm_controller     joint_trajectory_controller/JointTrajectoryController  active
RightArm_controller    joint_trajectory_controller/JointTrajectoryController  active
Head_controller        joint_trajectory_controller/JointTrajectoryController  active
Pelvis_controller      joint_trajectory_controller/JointTrajectoryController  active
LeftHand_controller    joint_trajectory_controller/JointTrajectoryController  active
RightHand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

### Mover el cuerpo desde terminal (como en el V8)

```bash
# Cabeza
ros2 topic pub --once /pepper/Head_controller/command trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [HeadYaw, HeadPitch], points: [{positions: [0.8, -0.3], time_from_start: {sec: 1}}]}"

# Brazo derecho
ros2 topic pub --once /pepper/RightArm_controller/command trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw],
    points: [{positions: [-1.0, -0.3, 1.2, 1.0, 0.0], time_from_start: {sec: 2}}]}"
```

![Pepper en Gazebo Harmonic](docs/img/fase2_gazebo.png)

### Mover el cuerpo con rqt (como en el V8)

En el V8: `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`. En el V9:

```bash
ros2 launch pepper_control rqt_joint_trajectory_controller.launch.py
```

Se elige `/pepper/controller_manager` y un controlador, se pulsa el botón de encendido y se
mueven los sliders.

> **Por qué un launch y no `ros2 run`:** en ROS 2 este plugin publica siempre en
> `<controlador>/joint_trajectory` y lee `robot_description` sin namespace. El launch lo
> remapea a `/pepper/<X>_controller/command` y a `/pepper/robot_description`, para que el V9
> mantenga un único topic de comandos, el mismo del V8.

![rqt_joint_trajectory_controller con los controladores del V8](docs/img/fase2_rqt_controladores.png)

### Verificación

- Los 7 controladores activos, con los nombres del V8.
- Tras `arms_down.sh`, las articulaciones quedan en las posiciones del V8
  (`LShoulderPitch 1.45`, `LShoulderRoll 0.10`, `LWristYaw -1.0`...).
- `Head_controller/command` y `RightArm_controller/command` mueven las articulaciones al
  valor pedido. `rqt_joint_trajectory_controller` mueve `LShoulderPitch` de 1.45 a -1.77.
- Factor de tiempo real: 1.00. Pepper queda de pie en su sitio.

### Cambios respecto al modelo del V8 (necesarios en Gazebo Harmonic)

| Qué | V8 | V9 | Por qué |
|---|---|---|---|
| Masa/inercia de `l_gripper`, `r_gripper` | 2e-06 kg / 1.1e-09 | 0.05 kg / 1e-05 | El motor de física de Harmonic (DART) **aborta** (`dLDLTRemove` en el solver LCP) con cuerpos tan ligeros colgando de un joint móvil. En Gazebo Classic (ODE) funcionaba |
| Posición inicial de `LHand`, `RHand` | 0 | 0.5 | El límite del joint es 0.02–0.98; empezar en 0 lo deja fuera de rango |
| Ruedas en ros2_control | — | no incluidas | La base se mueve con su propio plugin, como en el V8 (Fase 3) |

---

## Fase 3 — Base holonómica y odometría

Pepper se mueve con `/pepper/cmd_vel` como en el V8: avanza, se desplaza en lateral y gira,
todo a la vez.

### Paquete `gazebo_model_velocity_plugin` (C++)

Mismo nombre que el plugin que el V8 clonaba de `awesomebytes/gazebo_model_velocity_plugin`,
portado a Gazebo Harmonic. Ya **no hace falta el parche manual** del V8
(`math::Vector3` → `ignition::math::Vector3d` en las líneas 394–395).

| Plugin | Topics | Equivale en el V8 a |
|---|---|---|
| `GazeboRosModelVelocity` | sub `/pepper/cmd_vel` · pub `/pepper/odom`, `/pepper/output_vel`, TF `odom → base_footprint` | `libgazebo_ros_model_velocity.so` |
| `GazeboRosP3D` | pub `/pepper/odom_groundtruth` (frame `world`) | `libgazebo_ros_p3d.so` |

Los bloques `<plugin>` están en `pepper_description/urdf/pepper_gazebo.xacro` con **los
mismos parámetros que `pepperGazeboCPU.xacro` del V8**: límites de 0.55 m/s y 2.0 rad/s,
aceleración 0.44 m/s² y 2.4 rad/s², jerk, timeout de 0.5 s, odometría a 20 Hz y ruido gaussiano
medido en el robot real (0.02 en XY, 0.02645 en yaw).

La odometría se calcula como en el V8: se integra la velocidad medida más ruido, así que
**acumula deriva** como la de un robot real. `/pepper/odom_groundtruth` da la posición exacta
para comparar.

### Mover a Pepper

Terminal (igual que el V8):

```bash
ros2 topic pub /pepper/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Con rqt (igual que el V8):

```bash
ros2 run rqt_robot_steering rqt_robot_steering
```

Escribir `/pepper/cmd_vel` en el campo del topic y pulsar Enter. La casilla **stamped** debe
quedar **desmarcada**: en Jazzy rqt_robot_steering también puede publicar `TwistStamped`, y
el plugin usa `Twist`, como el V8.

![rqt_robot_steering publicando en /pepper/cmd_vel](docs/img/fase3_rqt_robot_steering.png)

### Verificación

| Prueba | Posición real (`odom_groundtruth`) | Odometría (`odom`) |
|---|---|---|
| `x = 0.3` durante 3 s | avanza 1.02 m | 1.01 m |
| `y = 0.3` durante 3 s | 1.04 m **a la izquierda** | 1.05 m |
| `z = 1.0` durante 2 s | gira 138.8° | 139.1° |
| `x = 2.0` | velocidad recortada a **0.55 m/s** | — |
| rqt_robot_steering a 0.30 m/s | 2.05 m | 2.05 m |

Las dos odometrías publican a 20 Hz y la TF va de `odom` hasta todos los frames del robot.

### Cambios respecto al V8

| Qué | V8 | V9 | Por qué |
|---|---|---|---|
| Signo de `linear.y` | invertido: `+y` movía a Pepper a su **derecha** | `+y` = izquierda | Convención de ROS (REP 103). Bug del plugin original |
| Integración de la odometría | fórmula de arco que suma `vx + vy` | integración con el punto medio del giro | La fórmula original falla al moverse en lateral mientras gira |
| Fricción de la base (`Tibia`) | por defecto | `mu1 = mu2 = 0` | La base no rueda: se fija su velocidad, como en el V8. Con fricción, el roce con el suelo reduce el giro ~35% |
| Aplicación del comando | en cada actualización (50 Hz) | en cada paso de física (1 kHz), con el limitador a 50 Hz | Gazebo Harmonic descarta el comando de velocidad tras cada paso |

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
