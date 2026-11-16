"""
Visualization of the pairwise constraint (supervisor's proposal) on the int main() dummy data.

    o_ij = |V_i ∩ V_j| / |V_i ∪ V_j|
    v_i + v_j <= 1        for every pair of views i, j with o_ij < tau

Two views that do not overlap enough may not both be selected, i.e. every pair of
selected views overlaps by at least tau. Start view and virtual end are excluded.

Usage:   python visualize_pairwise.py
Output:  figures/pairwise_constraint.png and figures/pairwise_constraint.pdf
Needs:   numpy, scipy, matplotlib
"""
import itertools
import os

import viz_common as V
from test_overlap_formulation import solve_ilp

CONFIGS = [  # (panel title, bar label, tau)
    ("GO-VMP baseline", "baseline", None),
    ("pairwise  τ = 0.3", "τ=0.3", 0.3),
    ("pairwise  τ = 0.4", "τ=0.4", 0.4),
    ("pairwise  τ = 0.5", "τ=0.5", 0.5),
]


def forbidden_pairs(o, tau):
    real = [i for i in range(o.shape[0]) if i != V.START]
    return [(i, j) for i, j in itertools.combinations(real, 2) if o[i, j] < tau]


def brute_force_pairwise(m, R, o, tau):
    """Independent check: cheapest covering path whose views pairwise overlap by >= tau."""
    real = [i for i in range(m.shape[0]) if i != V.START]
    best = None
    for r in range(1, len(real) + 1):
        for sel in itertools.combinations(real, r):
            if not (R[list(sel) + [V.START]].sum(axis=0) >= 1).all():
                continue
            if tau is not None and any(o[i, j] < tau for i, j in itertools.combinations(sel, 2)):
                continue
            for perm in itertools.permutations(sel):
                c = m[V.START, perm[0]] + sum(m[perm[t], perm[t + 1]] for t in range(r - 1))
                if best is None or c < best:
                    best = c
    return best


def main():
    m, R, o = V.instance()
    results = []
    for title, _, tau in CONFIGS:
        forbid = None if tau is None else forbidden_pairs(o, tau)
        res = solve_ilp(m, R, V.START, forbid=forbid)
        ref = brute_force_pairwise(m, R, o, tau)           # independent check
        assert (res is None and ref is None) or abs(res[0] - ref) < 1e-6, f"{title}: ILP {res} != brute force {ref}"
        results.append(res)
        n = 0 if forbid is None else len(forbid)
        print(f"{title:24s} {n:2d} pair constraints  {'infeasible' if res is None else f'cost {res[0]:.0f}  path {res[1]}'}")

    os.makedirs("figures", exist_ok=True)
    V.make_figure(
        "Pairwise overlap constraint  v_i + v_j ≤ 1",
        "Two views with overlap ratio < τ may not both be selected, so every pair of selected views overlaps by ≥ τ. "
        "Objective unchanged: minimize motion cost.",
        CONFIGS, results, o, "figures/pairwise_constraint")


if __name__ == "__main__":
    main()
