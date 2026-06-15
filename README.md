# Duckietown Maze Navigation

Deze repository bevat een complete ROS-stack voor maze-navigatie op een Duckiebot:

1. Path planning (Dijkstra)
2. Localisatie (odometrie + AprilTag)
3. Navigatie/maneuvers
4. PID lane controller

De standaard launcher start automatisch de volledige stack.

## Snel starten via dts terminal

Werk vanuit de root van deze repository.

1. Build de module image.
2. Run de module op de Duckiebot.

Gebruik in de praktijk je normale dts build/run workflow (zoals in de les gebruikt).
De launcher `default.sh` start automatisch:

- `maze_path_planner/launch/maze_navigation.launch`
- met `start_node` uit `START_NODE` (default `S`)
- met `goal_node` uit `GOAL_NODE` (default `T`)
- met `veh` uit `VEHICLE_NAME` of fallback naar `ROBOT_NAME`/`HOSTNAME`

## Omgevingsvariabelen

- `VEHICLE_NAME`: naam van de robot (bijv. `csc22999`)
- `START_NODE`: startknoop in de map (default `S`)
- `GOAL_NODE`: doelknoop in de map (default `T`)
- `DUCKIE_MAZE_MAP`: pad naar JSON/YAML mapbestand (default `${DT_REPO_PATH}/assets/maze_map.yaml`)

## Belangrijke dependencies

Deze zijn al toegevoegd in de repository:

- Python: `numpy`, `opencv-python-headless`
- APT: `ros-noetic-cv-bridge`

## Assets

Plaats het ONNX model op de verwachte locatie in de container:

- `/data/assets/best.onnx`

Of overschrijf met:

- `DUCKIE_MODEL_PATH=/data/assets/<jouw_model>.onnx`

## Externe doolhofkaart inladen

Als jullie 1 week voor de les een nieuwe grafische maze-layout krijgen, vervang dan:

- `assets/maze_map.yaml`

en build/run opnieuw.

Wil je zonder rebuild wisselen, zet het bestand op de robot en verwijs ernaar met:

- `DUCKIE_MAZE_MAP=/data/assets/maze_map.yaml`

Je kunt starten vanuit:

- `assets/maze_map.example.yaml`

Verwacht formaat:

- `node_positions`: node naar `[x, y]`
- `graph`: node naar lijst van `[buur_node, kost]`

Bij het opstarten leest `maze_path_planner_node` automatisch dit bestand in.
Als het bestand ontbreekt of ongeldig is, gebruikt de planner de ingebouwde default map.