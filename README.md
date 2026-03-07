# Battle Bot

A differential-drive battle bot powered by ROS 2 Jazzy and an ESP32. The ESP32 controls SCServo motors via [Pico-ROS](https://github.com/MarqRazz/Pico-ROS-software) (lightweight Zenoh client) and communicates over WiFi with a ROS 2 stack running in Docker.

```
ESP32 (bbot_esp)   <--WiFi/Zenoh-->  Docker (zenoh_host)
  zenoh-pico client                    rmw_zenoh_cpp router
  SCServo motors                       diff_drive_base_controller
  WiFiManager portal                   joint_state_broadcaster
```

## Prerequisites

- [Docker Compose](https://docs.docker.com/compose/install/linux/#install-using-the-repository)
- [VS Code](https://code.visualstudio.com/download) with the [PlatformIO](https://platformio.org/install/ide?install=vscode) extension (for ESP32 firmware)

## ROS 2 Host Setup (Docker)

### Build

```bash
cd zenoh_host
docker compose build
```

The default `docker-compose.yaml` assumes an NVIDIA GPU with the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed.

<details>
<summary><b>No NVIDIA GPU?</b></summary>

Use the no-GPU compose override:

```bash
docker compose -f docker-compose.yaml -f docker-compose.no-gpu.yaml build
docker compose -f docker-compose.yaml -f docker-compose.no-gpu.yaml up -d
```

To avoid repeating the flags, export `COMPOSE_FILE`:

```bash
export COMPOSE_FILE=docker-compose.yaml:docker-compose.no-gpu.yaml
docker compose up -d
```

</details>

### Run

```bash
# Allow GUI apps (once per boot)
xhost +

# Start the container
cd zenoh_host
docker compose up -d

# Shell into the container
docker exec -it bbot bash
```

### Launch the Robot

Inside the container:

```bash
# Mock hardware (no ESP32 needed):
ros2 launch bbot_bringup bbot.launch.xml mock_hardware:=true

# Real hardware (start Zenoh router first):
ros2 run rmw_zenoh_cpp rmw_zenohd
ros2 launch bbot_bringup bbot.launch.xml mock_hardware:=false

# Joystick teleop (Logitech F710):
ros2 launch bbot_bringup teleop.launch.py

# URDF visualization only:
ros2 launch bbot_description show_bbot.launch.xml
```

### Development

Source files are bind-mounted into the container. URDF, launch, and config changes take effect immediately (symlink install). Changes to `CMakeLists.txt` or `package.xml` require a rebuild:

```bash
# Inside the container:
symlink_build       # alias for colcon build --symlink-install
source install/setup.bash
```

## ESP32 Firmware

See [bbot_esp/README.md](bbot_esp/README.md) for build, flash, and WiFi configuration instructions.

## Pre-commit

```bash
pip install pre-commit
pre-commit install         # Run on every commit
pre-commit run -a          # Run all hooks manually
```
