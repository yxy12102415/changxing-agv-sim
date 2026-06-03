# AGV Simulation Workspace

This workspace contains the first AGV simulation assets:

- `src/agv_maps/map/changxing_v1.osm`: Lanelet2-style vector map.
- `src/agv_vehicle_model`: two-axis steering model sources intended for Autoware `simple_planning_simulator` integration.

## Build The Current Workspace

```bash
source /opt/ros/humble/setup.bash
cd /data/projects/AGV_sim_ws
colcon build --symlink-install
source install/setup.bash
```

## Next Integration Step

The vehicle model depends on Autoware's `simple_planning_simulator` API. This workspace preserves the source layout, but the model still needs to be wired into an Autoware source workspace before it can be selected by Autoware at runtime.
