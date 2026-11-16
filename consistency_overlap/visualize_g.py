"""
Visualization of the registration-degree constraint (g) on the int main() dummy data.

    o_ij = |V_i ∩ V_j| / |V_i ∪ V_j|,   a_ij = 1 if o_ij >= tau
    (g)  sum_{j != i} a_ij * v_j  >=  k * v_i        for every view i except start and virtual end

Every selected view must overlap at least k other selected views by at least tau.

Usage:   python visualize_g.py
Output:  figures/g_constraint.png and figures/g_constraint.pdf
Needs:   numpy, scipy, matplotlib
"""
import os

import viz_common as V
from test_overlap_formulation import solve_ilp, brute_force

CONFIGS = [  # (panel title, bar label, tau, k)
    ("GO-VMP baseline", "baseline", None, 0),
    ("(g)  τ = 0.3,  k = 1", "τ=0.3, k=1", 0.3, 1),
    ("(g)  τ = 0.3,  k = 2", "τ=0.3, k=2", 0.3, 2),
    ("(g)  τ = 0.4,  k = 2", "τ=0.4, k=2", 0.4, 2),
    ("(g)  τ = 0.5,  k = 2", "τ=0.5, k=2", 0.5, 2),
]


def main():
    m, R, o = V.instance()
    results = []
    for title, _, tau, k in CONFIGS:
        A = None if tau is None else (o >= tau).astype(int)
        res = solve_ilp(m, R, V.START, A, k)
        ref = brute_force(m, R, V.START, A, k)          # independent check
        assert (res is None and ref is None) or abs(res[0] - ref) < 1e-6, f"{title}: ILP {res} != brute force {ref}"
        results.append(res)
        print(f"{title:24s} {'infeasible' if res is None else f'cost {res[0]:.0f}  path {res[1]}'}")

    os.makedirs("figures", exist_ok=True)
    V.make_figure(
        "Registration-degree constraint (g)",
        "Every selected view must overlap at least k other selected views with overlap ratio ≥ τ. "
        "Objective unchanged: minimize motion cost.",
        [(t, l, tau) for t, l, tau, _ in CONFIGS], results, o, "figures/g_constraint")


if __name__ == "__main__":
    main()
