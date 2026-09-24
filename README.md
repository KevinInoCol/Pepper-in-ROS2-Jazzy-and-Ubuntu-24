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
| 1 | Pepper en RViz2 (URDF + mallas, sin Gazebo) | ⏳ siguiente |
| 2 | Articulaciones en Gazebo Harmonic (`gz_ros2_control`) | pendiente |
| 3 | Base holonómica + odometría (`/pepper/cmd_vel`, `/pepper/odom`) | pendiente |
| 4 | Sensores: cámaras, profundidad, láseres, sonares, bumpers | pendiente |
| 5 | Mundos: oficina, museo, museo con personas | pendiente |
| 6 | SLAM con `slam_toolbox` (reemplaza gmapping) | pendiente |
| 7 | Navegación con Nav2 (reemplaza amcl + move_base) | pendiente |
| 8 | Percepción con `yolo_ros` (reemplaza darknet_ros) | pendiente |
| 9 | Opcionales: MoveIt 2, gente dinámica, Pepper real | opcional |

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

## Referencias

- **Tutorial V8** (ROS 1): este repositorio, `Pepper Tutorial V8.docx.pdf`.
- [`awesomebytes/pepper_virtual`](https://github.com/awesomebytes/pepper_virtual) (rama
  `simulation_that_works`) y [`marco-quiroz/pepper_robot`](https://github.com/marco-quiroz/pepper_robot):
  la simulación ROS 1 del V8, **referencia de la nomenclatura**.
- [`HibaSekkat/pepper_ign_moveit2`](https://github.com/HibaSekkat/pepper_ign_moveit2):
  Pepper en ROS 2 + Ignition con MoveIt 2, **base del código** (URDF, mallas, ros2_control).
  Sekkat et al., *"Beyond simulation: ... Pepper open-source digital twin"*, Heliyon 10(14), 2024.
- [`tuncismail/pepper-robot-ros2-gazebo-simulation`](https://github.com/tuncismail/pepper-robot-ros2-gazebo-simulation):
  referencia de parámetros de sensores y base.

## Licencias

El código de las referencias es BSD-3 (Sekkat) y MIT (tuncismail). Las **mallas de Pepper
son de Aldebaran/SoftBank bajo CC-BY-NC-ND**: sólo uso no comercial y con atribución.
