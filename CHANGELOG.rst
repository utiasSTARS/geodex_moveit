^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package geodex_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-10-08)
------------------
First public release, for geodex 1.0.0 on MoveIt 2.12 (Jazzy) and MoveIt 2.15 (Lyrical).

* Planning

  * MoveIt 2 planner plugin ``geodex_moveit/GeodexPlanner`` with G-RRT*.
  * Euclidean and kinetic-energy metrics on the configuration space.
  * Joint goals, and pose and position goals through MoveIt's constraint samplers.
  * Path smoothing with geodex's metric-aware smoother, which rounds the corners into C² curves
    (``smoothing.round_corners``).
  * Fixed-base arms only. Plan mobile manipulators with geodex directly.

* Collision checking

  * Collision checking with geodex's sphere models of the robots or with the MoveIt planning scene.
  * Attached objects as one enclosing sphere, or as a sphere cover from the attached sphere
    primitives or from ``attached_object.spheres``, and an exact test of an attached box.
  * ``attached_object_spheres.py`` turns the spheres foam fits to an object mesh into
    ``attached_object.spheres``.
  * ``obstacle_padding`` sets the clearance of planned configurations from every obstacle.

* Configuration and demos

  * One generate_parameter_library schema declares and validates every parameter.
  * Panda demo and FR3 shelf demo on mock hardware with RViz, in ``geodex_moveit_demos``.
