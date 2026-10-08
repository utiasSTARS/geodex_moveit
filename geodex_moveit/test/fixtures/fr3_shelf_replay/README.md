# FR3 shelf smoother replay fixtures

Each fixture holds, for one metric and one pair of compartments of the demo's shelf, the planner's
waypoints (`raw.csv`), the planning scene (`scene.bin`, a serialized `moveit_msgs/PlanningScene`),
and the settings (`meta.yaml`). `attached_spheres.csv` is a cover of the attached box by 112
spheres, one sphere per row as x, y, z and radius in `2f85_tcp`. The spheres contain the box and
reach at most 0.01 m outside it. The robot files are copies of
`geodex_moveit_demos/robots/fr3_arm_gripper/`.

The replay reads `collision_check_resolution`, `output_spacing`, `round_corners`,
`obstacle_padding` and the attached-object settings from `meta.yaml`. `path.csv` and `result.yaml`
hold the smoother's recorded output, with the `collision_free` flag, the stage the smoother
returned and the metric length.
