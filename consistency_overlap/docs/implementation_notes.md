# Claude implementation: overlap-ratio constraint for GO-VMP

Implementation of the registration-degree constraint (g) on top of the GO-VMP ILP, tested with the random dummy data of `int main()` in `views_voxels_motion.cpp`.

## Files

| File | Status | Purpose |
|---|---|---|
| `claude_implementation.cpp` | new | The implementation: original class plus constraint (g), and a `main()` that solves the dummy data with and without the constraint and checks the results. |
| `claude_implementation.md` | new | This description. |
| `test_overlap_formulation.py` | unchanged, from the earlier step | Python reference: same model solved with HiGHS and compared against brute force. Source of the expected values below. |
| `views_voxels_motion_overlap.cpp` | unchanged, superseded | First draft of the constraint. `claude_implementation.cpp` replaces it; it can be deleted. |
| `views_voxels_motion.cpp` | not touched | Original file. |
| everything else in the folder | not touched | Including `GO-VMP/`, `overlap_analysis.py`, `overlap_formulation.md`, `overlap_results.json`, `output/`. |

Every change relative to `views_voxels_motion.cpp` is marked with `[overlap]` in the code.

## What the formulation does

The original model selects views `v_i` and a path `p_ij` with minimal motion cost such that every target voxel is seen at least once (constraints a–f of the GO-VMP paper). It has no notion of whether the selected views share any data, so a selected view can be completely unconnected to the others. Such a view cannot be registered against the rest, and a pose error in it goes uncorrected.

The extension adds one family of constraints. The objective is unchanged.

```
V_i   = set of voxels seen by view i                      (row i of connection_graph)
o_ij  = |V_i ∩ V_j| / |V_i ∪ V_j|                         overlap ratio (Jaccard), constant
a_ij  = 1 if o_ij >= tau, else 0                          views i and j are registrable, constant

(g)   sum over j != i of  a_ij * v_j   >=   k * v_i       for every view i except start and virtual end
```

- If view i is selected (`v_i = 1`), at least `k` other selected views must overlap it with ratio at least `tau`.
- If view i is not selected (`v_i = 0`), the constraint reads `... >= 0` and has no effect.
- `tau <= 0` or `k = 0` switches the extension off and gives exactly the original model.
- `k = 1`: no selected view is isolated. `k = 2`: every selected view has two registration partners, which forces at least one loop in the registration graph.

The start view and the virtual end view are left out of (g): the start view has no voxels in the dummy data and the virtual end is route bookkeeping.

## How it is implemented

All in `claude_implementation.cpp`:

1. **Parameters.** The constructor takes two more arguments with defaults, `_overlap_threshold` (tau, default -1 = off) and `_min_overlap_views` (k, default 0). Existing calls keep working.
2. **Overlap matrix.** After the coverage constraints, `overlap_ratio[i][j]` and `registrable[i][j]` are computed for all view pairs from `connection_graph`, which already holds each view's voxels without duplicates.
3. **Constraint (g).** For every view except start and virtual end, a `GRBLinExpr` sums `view_selection[j]` over its registrable partners and `model->addConstr(sum >= k * view_selection[i])` is added. No new variables.
4. **Infeasibility.** `solve()` now returns `bool`. It checks `SolCount` before reading the solution, because (g) can make the problem infeasible and the original code would throw in that case.
5. **Independent check.** `evaluate(tau)` rebuilds the registration graph of the solution from the selected views and reports: number of views, edges, minimum degree, connected components, independent loops (`edges − views + components`) and bridges. It also re-checks that every voxel is covered. It does not use the solver's constraints, only the solution.
6. **Solver log.** `OutputFlag` is set to 0 unless `print_log` is true, so the result table is readable.

## How the formulation is used in the test

`main()` generates the dummy data exactly as the original (`srand(1)`, 10 views, 10 voxels, start view 0, same order of `rand()` calls). It then solves that one instance five times:

| Row | tau | k | Purpose |
|---|---|---|---|
| GO-VMP baseline | off | 0 | Reference: original model |
| (g) tau=0.3 k=1 | 0.3 | 1 | No isolated view |
| (g) tau=0.3 k=2 | 0.3 | 2 | Redundancy at a low threshold |
| (g) tau=0.4 k=2 | 0.4 | 2 | Higher threshold |
| (g) tau=0.5 k=2 | 0.5 | 2 | Higher threshold |

For each row it prints motion cost, path and the registration-graph numbers from `evaluate()`. The baseline's graph is reported at tau = 0.3 so it can be compared with the constrained rows.

Three checks decide the `check` column and the exit code (0 = all passed):

1. Every voxel is covered by a selected view.
2. Every selected view has at least `k` registrable partners among the selected views.
3. No constrained row is cheaper than the baseline (adding a constraint can only raise the minimum).

## Expected output

`rand()` differs between Linux (glibc) and Windows (MSVC), so the dummy data differs by platform. These values come from the Python reference and from an exact dynamic program, both independent of Gurobi.

Linux / glibc (the devcontainer):

| Row | Cost | Path | Views | Edges | Min degree | Components | Loops | Bridges |
|---|---|---|---|---|---|---|---|---|
| GO-VMP baseline | 55 | 0-3-8-1 | 3 | 1 | 0 | 2 | 0 | 1 |
| (g) tau=0.3 k=1 | 61 | 0-6-9-2 | 3 | 2 | 1 | 1 | 0 | 2 |
| (g) tau=0.3 k=2 | 67 | 0-6-5-1 | 3 | 3 | 2 | 1 | 1 | 0 |
| (g) tau=0.4 k=2 | 120 | 0-6-2-8-1 | 4 | 5 | 2 | 1 | 2 | 0 |
| (g) tau=0.5 k=2 | 150 | 0-1-8-2 | 3 | 3 | 2 | 1 | 1 | 0 |

Windows / MSVC:

| Row | Cost | Path |
|---|---|---|
| GO-VMP baseline | 26 | 0-3-2 |
| (g) tau=0.3 k=1 | 26 | 0-3-2 |
| (g) tau=0.3 k=2 | 56 | 0-3-6-9-4 |
| (g) tau=0.4 k=2 | 89 | 0-3-2-4 |
| (g) tau=0.5 k=2 | infeasible | |

The cost must match. The path and the graph numbers can differ if several solutions have the same cost.

Reading the Linux table: the baseline selects view 3, which overlaps no other selected view. With k = 1 the selected views form a chain (no loop, two bridges). With k = 2 they form a loop with no bridge, at a motion cost of 67 instead of 55.

## How to build and run

In the environment that has Gurobi and OctoMap (the GO-VMP devcontainer):

```
cd <path-to-GO-VMP>/consistency_overlap
g++ -std=c++14 claude_implementation.cpp -o ~/thesis_build/claude_implementation \
    -I$GUROBI_HOME/include -L$GUROBI_HOME/lib -lgurobi_c++ -lgurobi110 -loctomap -loctomath
~/thesis_build/claude_implementation
```

`<version>` depends on the installed Gurobi, for example `gurobi110`. Gurobi may print a licence line each time a model is created; those lines appear between the table rows.

## Verification status

- The math of (g) is verified: the Python reference equals brute-force enumeration on five random instances.
- `claude_implementation.cpp` compiles without warnings under C++11, C++14 and C++17 against stand-in headers that declare the Gurobi and OctoMap calls it uses.
- **Executed with real Gurobi** on 2026-11-13: WSL Ubuntu 20.04, g++ 9, Gurobi 11.0.1 (academic licence), OctoMap from `liboctomap-dev`, built with the command above (`-lgurobi110`). All five rows match the Linux table above exactly (cost, path and every registration-graph number), `All checks passed: yes`, exit code 0.
- Gurobi prints `Set parameter Username`, the licence line and `Set parameter LazyConstraints` before each row. These are printed when the model is created and are not errors.

## Known limits

- (g) is a hard constraint and can be infeasible (last Windows row). A soft version with a slack variable and a penalty in the objective is described in `overlap_formulation_claude.pdf`, Section 4; it is not implemented.
- (g) is local. It does not guarantee that the registration graph is connected or free of bridges, although it was in all feasible test rows. The flow-based connectivity constraint and 2-edge-connectivity from Knechtel et al. are not in this file.
- Overlap is computed on the same voxels as coverage. In the real planner those are partly unknown voxels; registration needs overlap on observed surface.
