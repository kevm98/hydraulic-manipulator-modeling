# Data and figure provenance

## Numerical path traces

`archive/figures/plot.fig` stores actual `XData`, `YData`, and `ZData` arrays for curves labeled `Reference` and `Measured`. The figure script exports them directly to:

- `data/figure-traces/task_path_reference.csv`: 3,991 rows.
- `data/figure-traces/task_path_measured.csv`: 3,550 rows.

These are historical simulated coordinates, not new measurements and not image digitizations. They have no shared time vector in the extracted representation. The regenerated figure shows horizontal and vertical projections, with all axes in metres. Different sampling prevents indexwise error calculation; no RMS metric is recomputed from these spatial CSVs.

## Bulk-modulus summary

`data/figure-traces/bulk_modulus_summary.csv` records the source numerical summary. The joint-1, joint-3, joint-4 delays and RMS errors agree with `archive/scripts/delaysvserror.m`, the original presentation, and Table I of the earlier written report. Mean errors and the reported aggregate delay index are transcribed from the report/presentation tables.

The original written report's Table II gives conflicting joint-3 values: **185.0, 166.5, 155.3, 173.7 ms**. The revised report consistently uses the series confirmed by Table I and the source script: **186.3, 168.8, 149.9, 152.4 ms**. Both original documents are preserved so the discrepancy remains reviewable.

Joint-2 intervals are not tabulated in the summary script. The earlier report states they remain below 0.3 ms. The displayed four-joint aggregate is retained as originally reported; it is not recomputed by pretending a missing channel is a measured zero.

Mean error increases overall from 211.3 to 228.8 mm, while RMS error decreases from 827.9 to 739.4 mm. The revised text corrects the earlier claim that both improve together. Joint-4 delay increases from 202.1 to 221.6 ms.

## Illustrative curves excluded from results

`archive/scripts/delay_plots.m` generates second-order synthetic responses with an assumed damping ratio of 0.85 and selects natural frequencies to match reported 50% crossing times. `Best_and_worst_cases.fig` contains those explanatory reconstructions. They are preserved in the archive and are not presented as physical-simulation or experimental traces in the revised report.

`inverse_kinematics_plots.fig` contains a separate mowing-sweep reference example. It is not merged with the arc-and-chord task-space tracking curves.

## Rebuild

```bash
python3 -m pip install -r scripts/requirements-figures.txt
python3 scripts/build_figures.py
```

The default uses the versioned CSVs; add `--extract-fig` to repeat the original FIG extraction with SciPy. The script produces vector PDF/SVG and 300 dpi PNG previews for the sensitivity, path, and functional-architecture figures. There is no smoothing, new simulation, or invented sample interpolation in the exported path curves. Connecting lines in the four-case sensitivity figure guide the eye.

## Construction and network figures

The report's construction diagrams are separate from result data:

- `mechanical-topology.png` and `actuator-subsystem-original.png` are the embedded images from page 3 of the original written report, extracted without changing their content.
- `integrated-model-original.png` is the integrated-system image embedded on slide 10 of the original presentation. It documents an earlier design view, including an accumulator that is disabled in the saved model.
- `cylinder-construction` explains chamber geometry, CAD bodies, alignment joints, and the translational force/velocity interface. It is a schematic, not a dimensioned manufacturing drawing.
- `hydraulic-network` reconstructs the saved top-level connection groups. N1/N2 denote the reservoir-connected and sensor/fluid-connected rails. Serialized endpoints are retained; flow direction and named port associations require confirmation in the loaded model.
- `valve-routing` expands the active valve's configured positive, neutral, and negative metering paths. Its arrows are schematic valve-relative routing, not measured network flow.

Rebuild the three vector schematics with:

```bash
python3 scripts/build_construction_figures.py
```

`scripts/extract_model_specification.py` writes `docs/model-specification.json` directly from the SLX XML. It records block settings and units, active/commented states, source members, and connection groups. Parameters in disabled alternatives are not substituted for active values. This inspection does not evaluate model-workspace expressions or run MATLAB.
