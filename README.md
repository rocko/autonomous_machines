# autonomous_machines

## IDEA

Develop a simulation using ROS.

Research and gradually extend the simulation with advanced topics such as remote control, mission planning, tactical behaviour, evasive maneuvering, target tracking, target identification or telemetry.

Apply ML and DL algorithms.

## GOAL

TBD, as we expore


## SCOPE
TBD, as we explore


## TECHNOLOGIES
- ROS
- ...


## INSTALLATION

### Prerequisites
- Docker Engine and Docker Compose
- Git
- SSH access to development machine
- An SSH client and web browser for remote visualization


### BUILD and START the environment
```bash
# Build the Docker images.
docker compose -f infra/compose.yaml build

# Start the ROS 2 environment and Gazebo simulation server.
docker compose -f infra/compose.yaml up -d

# Verify container status.
docker compose -f infra/compose.yaml ps
```
### SmokeTEST

```bash
./scripts/smoke_test.sh
# Expected: The ROS 2 publisher and subscriber successfully exchange messages.
```

```bash
docker compose -f infra/compose.yaml exec ros gz sim --versions
# Expected: Gazebo Sim 8.x, corresponding to Gazebo Harmonic
```

```bash
docker compose -f infra/compose.yaml exec ros gz service -l

docker compose -f infra/compose.yaml exec ros gz topic -l
# Expected services and topics include:
# /gazebo/worlds
# /world/shapes/control
# /world/shapes/clock
# /world/shapes/pose/info
#
```
A successful result confirms that Gazebo is running and its transport interfaces are available.

### Remote VISUALIZATION
Gazebo runs headlessly by default. Its graphical interface can be accessed remotely through noVNC.

Start the optional GUI service
```bash
docker compose -f infra/compose.yaml --profile gui up -d
```

From the Windows development machine, establish an SSH tunnel:
```bash
ssh -L 6080:127.0.0.1:6080 user@ai-server
```

Open `http://localhost:6080/vnc.html` in a web browser:


The Gazebo graphical interface should display the running simulation.

The GUI currently uses software rendering with the Ogre rendering engine for compatibility with the virtual display.

### STOP 
```bash
# Stop only the graphical interface.
docker compose -f infra/compose.yaml stop gazebo-gui

# Stop the complete environment.
docker compose -f infra/compose.yaml --profile gui down
```


## QUICKreference

Rebuild the Docker images
```bash
docker compose -f infra/compose.yaml build
```
Force a clean rebuild
```bash
docker compose -f infra/compose.yaml build --no-cache
```
Recreate containers
```bash
docker compose -f infra/compose.yaml --profile gui up -d --force-recreate
```
Inspect container status
```bash
docker compose -f infra/compose.yaml --profile gui ps
```
Inspect simulation logs
```bash
docker compose -f infra/compose.yaml logs --tail=50 ros
```
Inspect GUI logs
```bash
docker compose -f infra/compose.yaml logs --tail=50 gazebo-gui
```
Open a shell inside the ROS container
```bash
docker compose -f infra/compose.yaml exec ros bash
```




## PROJECT HISTORY

## MS01 - Setup
- Prepare Docker for ROS, gazebo
- Hands on experience, tutorials
- Test 
- Document setup for reproducibility
- Learn something