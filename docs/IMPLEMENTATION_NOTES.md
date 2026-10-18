# Implementation notes

## Portable model references

Geometry filenames in the SLX models resolve under `robot/meshes/stl/`. Signal Editor references resolve under `data/references/`. Only these filename fields were changed; controller code, physical parameters, model topology, and solver configuration were retained.

The earlier model variants use two legacy `Messerkopf-Shwenk` spellings. Their geometry aliases are backed by the corresponding `Schwenk` files. The file map under `original/` records these assignments and the hashes of individual SLX members.

## Controller and timing

The integrated model's embedded controller uses backstepping-style feedback on joints 1, 3, 4, and 5, and PI/velocity feedback with gravity bias on joint 2. Its pressure inputs are unused. The controller integrates at a hardcoded 0.005 s; the separate error-accumulation helper contains 0.01 s. Confirm actual block rates and logged timestamps when running a new experiment.

The stored model version is R2023b. The main configuration contains `ode15s`, a relative tolerance of `1e-3`, a maximum step of 1.2 s, and a 30 s stop time. These settings need a convergence study for new results.

## Result interpretation

The joint-3 values used in the report agree with the analysis script and the first delay table: 186.3, 168.8, 149.9, and 152.4 ms. A second table in the earlier report contains a different series. The mean-error values rise overall while RMS error falls; the discussion follows those numerical results.

Task-space path figures use the original MATLAB FIG arrays. The synthetic second-order curves in `archive/scripts/delay_plots.m` remain explanatory examples and are excluded from result figures. See [data and figure notes](DATA_AND_FIGURES.md).

## File integrity

`original/source-manifest.json` records original file hashes, and `original/migration-map.json` maps retained files and portable path edits. The original documents, CAD files, and recordings remain available with the project materials. Repository checks verify these artifacts without executing MATLAB or validating the plant against hardware.
