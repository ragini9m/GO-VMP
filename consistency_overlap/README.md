# Consistency-aware view selection: overlap constraint (g)

Thesis work on top of GO-VMP. The GO-VMP ILP (paper Sec. IV-B.2, `views_voxels_motion`) is extended by a constraint that makes the selected views overlap each other, so that their scans can be registered and pose errors become detectable. This reduces the risk of ghosting (one fruit reconstructed as two clusters).

Everything in this folder is standalone. It does not change the ROS planner in `src/`.

## The formulation

The original model chooses views `v_i` and a path `p_ij` with minimal motion cost such that every target voxel is seen at least once (constraints a–f). The extension adds one constraint family and leaves the objective unchanged:

```
V_i   = voxels seen by view i
o_ij  = |V_i ∩ V_j| / |V_i ∪ V_j|          overlap ratio (Jaccard), computed before solving
a_ij  = 1 if o_ij >= tau, else 0           views i and j are registrable

(g)   sum_{j != i} a_ij * v_j  >=  k * v_i       for every view i except start and virtual end
```

Every selected view must overlap at least `k` other selected views by at least `tau`. With `k = 2` the registration network contains a loop, so every registration can be checked by a second route. `tau <= 0` or `k = 0` gives the original GO-VMP model.

For comparison, the scripts also evaluate the pairwise rule `v_i + v_j <= 1` for every pair with `o_ij < tau` (every pair of selected views must overlap). See `figures/overlap_constraints_presentation.pdf` for why (g) is preferred.

## Contents

| File | What it is |
|---|---|
| `claude_implementation.cpp` | C++/Gurobi implementation: GO-VMP class plus constraint (g). `main()` solves the `int main` dummy data for five settings and checks each result. Changes against the original are marked `[overlap]`. |
| `test_overlap_formulation.py` | Same model in Python (HiGHS via scipy), checked against brute-force enumeration. Also contains the pairwise rule. |
| `visualize_g.py` | Figure for constraint (g). |
| `visualize_pairwise.py` | Figure for the pairwise rule. |
| `viz_common.py` | Drawing code shared by the two figure scripts. |
| `make_slides.py` | Presentation: GO-VMP → pairwise rule → (g). |
| `setup_wsl_gurobi.sh` | One-time setup on Ubuntu 20.04 / WSL: Gurobi 11.0.1, OctoMap, licence, build and run. |
| `gurobi_run.sh` | Runs a Gurobi program in WSL despite WSL's changing network address (see below). |
| `figures/` | Generated figures and slides. |
| `docs/overlap_formulation.pdf` | Formulation write-up with rationale. |
| `docs/implementation_notes.md` | Implementation details and expected output. |

## 1. C++ implementation with Gurobi

### One-time setup

Ubuntu 20.04 (native or WSL). The academic Gurobi licence must be activated **on the university network or full-tunnel VPN**; afterwards it works offline.

```
cd <path-to>/GO-VMP
bash consistency_overlap/setup_wsl_gurobi.sh <gurobi-licence-key>
```

This installs `build-essential` and `liboctomap-dev`, Gurobi 11.0.1 to `/opt/gurobi1101/linux64` (same path as `src/view_motion_planner/CMakeLists.txt`), adds `GUROBI_HOME`, `PATH` and `LD_LIBRARY_PATH` to `~/.bashrc`, rebuilds Gurobi's C++ library, activates the licence, and builds and runs the program.

Each licence key can be activated only once, on one machine. A key already used on Windows does not work in WSL (different host ID); request a second key on gurobi.com.

### Build

```
cd <path-to>/GO-VMP/consistency_overlap
mkdir -p ~/thesis_build
g++ -std=c++14 -O2 claude_implementation.cpp -o ~/thesis_build/claude_implementation \
    -I$GUROBI_HOME/include -L$GUROBI_HOME/lib -lgurobi_c++ -lgurobi110 -loctomap -loctomath
```

The binary goes to the Linux home folder: it runs faster there than on `/mnt/c`.

### Run

Native Linux:

```
~/thesis_build/claude_implementation
```

WSL: the named-user licence is tied to a host ID that Gurobi derives from the network card address, and WSL assigns a new random address after every restart. `gurobi_run.sh` sets the licensed address, runs the program, and restores the previous address. There is no internet in WSL while the program runs.

```
bash gurobi_run.sh ~/thesis_build/claude_implementation
```

It asks for the Linux `sudo` password. The licensed address is read from `GUROBI_LICENSED_MAC` (default: the author's machine). For your own licence, take `HOSTID` from `~/gurobi.lic`, for example `HOSTID=5d50d6ca` becomes `00:15:5d:50:d6:ca`:

```
GUROBI_LICENSED_MAC=00:15:5d:xx:xx:xx bash gurobi_run.sh ~/thesis_build/claude_implementation
```

A Gurobi WLS academic licence avoids this workaround entirely.

### Expected output

```
model             cost    path            views  edges  mindeg  comps  loops  bridges  check
GO-VMP baseline   55      0-3-8-1         3      1      0       2      0      1        ok
(g) tau=0.3 k=1   61      0-6-9-2         3      2      1       1      0      2        ok
(g) tau=0.3 k=2   67      0-6-5-1         3      3      2       1      1      0        ok
(g) tau=0.4 k=2   120     0-6-2-8-1       4      5      2       1      2      0        ok
(g) tau=0.5 k=2   150     0-1-8-2         3      3      2       1      1      0        ok
All checks passed: yes
```

Gurobi prints `Set parameter Username`, the licence line and `Set parameter LazyConstraints` before each row; these are not errors. The values above are for Linux, whose `rand()` produces this dummy data. On Windows `rand()` differs and so do the numbers.

To try other settings, edit the `test_cases` list in `main()` and rebuild.

## 2. Python: verification, figures, slides

Python 3.9 or newer on Windows or Linux. No Gurobi needed.

```
cd <path-to>/GO-VMP/consistency_overlap
pip install numpy scipy matplotlib
```

| Command | Output |
|---|---|
| `python test_overlap_formulation.py` | Table of all settings; last line must be `ILP == brute force for all configurations: True` |
| `python test_overlap_formulation.py 13 15 2 5` | Same for another random instance (`views voxels start seed`) |
| `python visualize_g.py` | `figures/g_constraint.png` and `.pdf` |
| `python visualize_pairwise.py` | `figures/pairwise_constraint.png` and `.pdf` |
| `python make_slides.py` | `figures/overlap_constraints_presentation.pdf` |

The figure scripts stop with an error if the solver result differs from brute force. Settings are in the `CONFIGS` list at the top of each script.

## Status and limits

- (g) is verified three ways on the dummy data: Python ILP, brute force, and C++ with Gurobi 11.0.1 give identical results.
- The data is random (10 views). Ghosting itself is not measured yet.
- Overlap is computed on target voxels; registration needs overlap on observed surface.
- (g) is a hard constraint and can make the problem infeasible. A soft version (slack plus penalty in the objective) is described in `docs/overlap_formulation.pdf` but not implemented.
- The constraint is not yet integrated into the ROS planner (`src/view_motion_planner/src/views_voxels_motion.cpp`).
