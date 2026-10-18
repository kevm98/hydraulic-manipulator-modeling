# Setup and compatibility

## Required products

The integrated model was saved in MATLAB R2023b. Use MATLAB, Simulink, Simscape, Simscape Multibody, and Simscape Fluids. Its active hydraulic blocks use the isothermal-liquid domain. Robotics System Toolbox is required for `importrobot`, rigidBodyTree, inverse kinematics, and related geometry scripts. A newer MATLAB release may request model upgrades; review them separately from this file migration.

## Open and simulate

From the repository root in MATLAB:

```matlab
open_project
out = run_simulation;
```

`setup_project` adds active source, model, URDF, mesh, and reference-data folders to the path. It changes the working directory to the project root because model filename fields use project-relative paths. Cache and code-generation products go into `build/`.

Open `models/Mulag_FME_500_with_cylinder.slx`, not an `.slxc` cache. Its Signal Editor files are the original position, velocity, acceleration, and end-effector datasets under `data/references/`. The setup does not load arbitrary MAT files into the base workspace or overwrite their contents.

No MATLAB simulation was run during migration. Static checks verify the SLX archive and its referenced files; they cannot verify licensed block availability, algebraic-loop initialization, or numerical convergence.

## Archived model settings

| Setting | Stored value |
| --- | --- |
| Model version | R2023b |
| Main configuration solver | `ode15s` |
| Stop time | 30 s |
| Relative tolerance | `1e-3` |
| Maximum solver step | 1.2 s |
| Embedded controller integration constant | 0.005 s |
| Embedded error-accumulation helper constant | 0.01 s |
| Position/velocity/EE Signal Editor sample time | 0.005 s |
| Acceleration Signal Editor sample time | 0.01 s |

These values were preserved, not tuned or verified. The model includes multiple configuration sets; confirm the active set when opening it. Check actual execution rates, dataset time vectors, logging, and solver convergence before reporting new results. The offline error metric uses recorded timestamps, not a hardcoded sample interval.

## Controller

The controller embedded in the integrated SLX file uses backstepping-style error feedback on joints 1, 3, 4, and 5. Joint 2 uses PI position control, velocity feedback, configuration-dependent gravity bias, and a soft joint-limit term. Its pressure inputs are unused. This differs from the earlier report's simplified description of the same backstepping controller on all axes.

The control function and physical parameters were not changed during organization. The standalone `default_cfg.m` and `makeHydroParams.m` contain other controller designs and helper parameters; their presence does not mean the integrated model uses every value they define.

## Metrics after a new run

For a `logsout` dataset containing `cmd1..cmd4`, `q1..q4`, `Act1_Ext..Act4_Ext`, corresponding `Act*_PA`/`Act*_PB`, and `e_norm`:

```matlab
[DelayTable, EE_Metrics] = Delay_Metrics(out.logsout, ...
    fullfile('build', 'new_run_metrics.mat'));
```

The analysis defines command onset using 5% of command span with an absolute minimum threshold of 0.002. Response intervals use first crossings of 50% of the maximum post-onset deviation, with linear interpolation. This is a rise-response measure, not a directly identified pure transport delay. A sub-sample interpolated crossing is not independent sub-sample measurement resolution.

The saved original `data/results/myDelays.mat` contains MATLAB table objects. Do not assume Python's generic MAT reader will deserialize those objects. Several reference datasets use MATLAB v7.3/HDF5 with MATLAB-native objects.

## Earlier variants and prototypes

`models/variants/` retains kinematic, dynamic, actuation, and actuator-test models. `models/experimental/` retains the custom component library and an earlier unnamed model. `src/+custom_component/` contains legacy hydraulic-domain prototypes; they are distinct from the main model's built-in isothermal-liquid blocks. Rebuilding a component library may require a compatible MATLAB release and compiler.

`archive/scripts/` is not added to the default MATLAB path. It includes exploratory scripts, a duplicate IK variant, a reset helper whose function name originally did not match its filename, and a tuner example referring to functions not supplied in this archive. The legacy kinematic export helper also uses an unverified output extension. These are retained for provenance, not advertised as validated entry points.
