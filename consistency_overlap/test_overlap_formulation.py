"""
Test of the overlap-ratio extension of the GO-VMP ILP (Sec. IV-B.2 of the paper).

Reproduces the random instance of int main() in views_voxels_motion.cpp
(srand(1), glibc rand()), solves
    L0  baseline GO-VMP                         (constraints a-f)
    L1  + registration degree constraint (g)    (overlap ratio tau, degree k)
    L2  + connected registration graph (h)      (single-commodity flow)
with HiGHS (scipy.optimize.milp) and checks every result against brute force
(Held-Karp over all view subsets), which is independent of the ILP.

Requires: numpy, scipy >= 1.9
Usage:    python test_overlap_formulation.py [num_view num_voxel start seed]
"""
import itertools
import sys

import numpy as np
from scipy.optimize import Bounds, LinearConstraint, milp


# ----------------------------------------------------------------------------
# glibc rand() (TYPE_3 additive feedback generator), so the data equals what
# the C++ main() produces on Linux. MSVC rand() gives different numbers.
# ----------------------------------------------------------------------------
class GlibcRand:
    def __init__(self, seed):
        r = [0] * 34
        r[0] = seed if seed != 0 else 1
        for i in range(1, 31):
            hi, lo = divmod(r[i - 1], 127773)
            word = 16807 * lo - 2836 * hi
            if word < 0:
                word += 2147483647
            r[i] = word
        for i in range(31, 34):
            r[i] = r[i - 31]
        for i in range(34, 344):
            r.append((r[i - 31] + r[i - 3]) & 0xFFFFFFFF)
        self.r = r

    def __call__(self):
        r = self.r
        val = (r[-31] + r[-3]) & 0xFFFFFFFF
        r.append(val)
        del r[0]
        return val >> 1


def generate_instance(num_view, num_voxel, start, seed):
    """Same call order as int main()."""
    rand = GlibcRand(seed)
    m = np.zeros((num_view, num_view))
    for i in range(num_view):
        for j in range(i):
            m[i, j] = m[j, i] = rand() % 100 + 1
    voxels = [(rand() % 30000, rand() % 30000, rand() % 30000) for _ in range(num_voxel)]
    views_voxels = []
    for i in range(num_view):
        sub = rand() % num_voxel + 1
        vv = [voxels[rand() % num_voxel] for _ in range(sub)]
        views_voxels.append([] if i == start else vv)
    # connection graph r_ic (view x voxel), voxel ids by first appearance
    ids = {}
    for vv in views_voxels:
        for key in vv:
            ids.setdefault(key, len(ids))
    R = np.zeros((num_view, len(ids)), dtype=int)
    for i, vv in enumerate(views_voxels):
        for key in vv:
            R[i, ids[key]] = 1
    return m, R


def overlap_ratio(R):
    """o_ij = |V_i n V_j| / |V_i u V_j| (Jaccard), o_ii = 0."""
    inter = R @ R.T
    size = R.sum(axis=1)
    union = size[:, None] + size[None, :] - inter
    o = np.divide(inter, union, out=np.zeros(inter.shape), where=union > 0)
    np.fill_diagonal(o, 0.0)
    return o


# ----------------------------------------------------------------------------
# ILP
# ----------------------------------------------------------------------------
def solve_ilp(m, R, start, A=None, k=0, connected=False, forbid=None):
    """A: 0/1 registrability matrix a_ij (None -> baseline).
    forbid: optional list of view pairs (i, j); adds v_i + v_j <= 1 for each (pairwise variant)."""
    n = m.shape[0]            # real views 0..n-1
    N = n + 1                 # + virtual end (index n)
    end = n
    M = np.zeros((N, N))
    M[:n, :n] = m
    real = [i for i in range(n) if i != start]          # views carrying data
    arcs = [(i, j) for i in real for j in real if i != j and A is not None and A[i, j]]

    # variable layout: v[N] | p[N*N] | w[n] | f[len(arcs)]
    iv = lambda i: i
    ip = lambda i, j: N + i * N + j
    iw = lambda i: N + N * N + i
    if_ = {a: N + N * N + n + t for t, a in enumerate(arcs)}
    use_flow = connected and A is not None
    nvar = N + N * N + (n + len(arcs) if use_flow else 0)

    c = np.zeros(nvar)
    for i in range(N):
        for j in range(N):
            c[ip(i, j)] = M[i, j]

    lb, ub = np.zeros(nvar), np.ones(nvar)
    integrality = np.ones(nvar)
    for i in range(N):
        ub[ip(i, i)] = 0                                   # no self loops
    ub[ip(start, end)] = 0                                 # p_0n = 0
    lb[ip(end, start)] = 1                                 # p_n0 = 1
    lb[iv(start)] = lb[iv(end)] = 1                        # v_0 = v_n = 1
    if use_flow:
        ub[iw(start)] = 0
        for a in arcs:
            ub[if_[a]] = n
            integrality[if_[a]] = 0

    rows, lo, hi = [], [], []

    def add(coefs, l, h):
        row = np.zeros(nvar)
        for idx, val in coefs:
            row[idx] += val
        rows.append(row); lo.append(l); hi.append(h)

    for i in range(N):
        inn = [(ip(j, i), 1) for j in range(N)]
        out = [(ip(i, j), 1) for j in range(N)]
        add(inn + [(iv(i), -1)], -np.inf, 0)               # (f) in  <= v_i
        add(out + [(iv(i), -1)], -np.inf, 0)               # (f) out <= v_i
        add(inn + out + [(iv(i), -1)], 0, np.inf)          # (f) in + out >= v_i
        add(inn, -np.inf, 1)                               # (b)
        add(out, -np.inf, 1)                               # (b)
        add(inn + [(idx, -val) for idx, val in out], 0, 0) # (b) in == out
    for cc in range(R.shape[1]):                           # (e) coverage
        add([(iv(i), 1) for i in range(n) if R[i, cc]], 1, np.inf)

    if A is not None and k > 0:                            # (g) registration degree
        for i in real:
            add([(iv(j), 1) for j in real if j != i and A[i, j]] + [(iv(i), -k)], 0, np.inf)

    for i, j in (forbid or []):                            # pairwise variant: v_i + v_j <= 1
        add([(iv(i), 1), (iv(j), 1)], -np.inf, 1)

    if use_flow:                                           # (h) connectivity, Shirabe flow
        add([(iw(i), 1) for i in real], 1, 1)              # one sink
        for i in real:
            add([(iw(i), 1), (iv(i), -1)], -np.inf, 0)     # sink must be selected
            out = [(if_[(i, j)], 1) for j in real if (i, j) in if_]
            inn = [(if_[(j, i)], -1) for j in real if (j, i) in if_]
            add(out + [(iv(i), -(n - 1))], -np.inf, 0)     # no outflow if unselected
            add(out + inn + [(iv(i), -1), (iw(i), n)], 0, np.inf)  # out - in >= v_i - n*w_i

    while True:                                            # (c) lazy subtour elimination
        res = milp(c, constraints=LinearConstraint(np.array(rows), lo, hi),
                   integrality=integrality, bounds=Bounds(lb, ub))
        if res.status != 0:
            return None
        x = np.round(res.x)
        succ = {i: j for i in range(N) for j in range(N) if x[ip(i, j)] > 0.5}
        tour, cur = [start], succ[start]
        while cur != start:
            tour.append(cur); cur = succ[cur]
        rest = set(succ) - set(tour)
        if not rest:
            path = tour[:-1] if tour[-1] == end else tour
            return float(res.fun), path
        while rest:                                        # cut every subtour
            s = rest.pop(); cyc = [s]; cur = succ[s]
            while cur != s:
                cyc.append(cur); rest.discard(cur); cur = succ[cur]
            add([(ip(i, j), 1) for i in cyc for j in cyc if i != j], -np.inf, len(cyc) - 1)


# ----------------------------------------------------------------------------
# Brute force reference (independent of the ILP)
# ----------------------------------------------------------------------------
def reg_stats(sel, A):
    """edges, min degree, connected?, cycle rank, bridges of the induced registration graph."""
    sel = list(sel)
    adj = {i: [j for j in sel if j != i and A[i, j]] for i in sel}
    edges = [(i, j) for i in sel for j in adj[i] if i < j]

    def components(edge_list):
        parent = {i: i for i in sel}
        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]; a = parent[a]
            return a
        for a, b in edge_list:
            parent[find(a)] = find(b)
        return len({find(i) for i in sel})

    ncomp = components(edges) if sel else 0
    bridges = sum(components([e for e in edges if e != b]) > ncomp for b in edges)
    return dict(edges=len(edges), min_deg=min((len(adj[i]) for i in sel), default=0),
                connected=ncomp <= 1, cycle_rank=len(edges) - len(sel) + ncomp, bridges=bridges)


def brute_force(m, R, start, A=None, k=0, connected=False):
    n = m.shape[0]
    real = [i for i in range(n) if i != start]
    q = len(real)
    INF = float("inf")
    dp = np.full((1 << q, q), INF)                         # Held-Karp: shortest path start -> ... -> last
    for a in range(q):
        dp[1 << a, a] = m[start, real[a]]
    for mask in range(1 << q):
        for last in range(q):
            d = dp[mask, last]
            if d == INF:
                continue
            for nxt in range(q):
                if not mask & (1 << nxt):
                    nm = mask | (1 << nxt)
                    dp[nm, nxt] = min(dp[nm, nxt], d + m[real[last], real[nxt]])
    best = INF
    for mask in range(1, 1 << q):
        sel = [real[a] for a in range(q) if mask & (1 << a)]
        if not (R[sel + [start]].sum(axis=0) >= 1).all():
            continue
        if A is not None and (k > 0 or connected):
            st = reg_stats(sel, A)
            if st["min_deg"] < k or (connected and not st["connected"]):
                continue
        best = min(best, dp[mask].min())
    return None if best == INF else float(best)


# ----------------------------------------------------------------------------
def main():
    args = [int(a) for a in sys.argv[1:]]
    num_view, num_voxel, start, seed = (args + [10, 10, 0, 1][len(args):])[:4]
    m, R = generate_instance(num_view, num_voxel, start, seed)
    o = overlap_ratio(R)

    print(f"instance: seed {seed}, {num_view} views, {R.shape[1]} distinct voxels, start {start}")
    print("voxels per view  :", R.sum(axis=1).tolist())
    print("views per voxel  :", R.sum(axis=0).tolist())
    print("overlap ratio o_ij (Jaccard):")
    print(np.array2string(o, precision=2, suppress_small=True, max_line_width=200))

    configs = [("L0 baseline", None, 0, False)]
    for tau in (0.3, 0.4, 0.5):
        configs += [(f"L1 tau={tau} k=1", tau, 1, False),
                    (f"L1 tau={tau} k=2", tau, 2, False),
                    (f"L2 tau={tau} k=1 +conn", tau, 1, True),
                    (f"L2 tau={tau} k=2 +conn", tau, 2, True)]

    do_bf = num_view <= 16
    print(f"\n{'model':24s} {'cost':>6s} {'brute':>6s}  ok  {'path':28s} |E| mindeg conn rank bridges")
    all_ok = True
    for name, tau, k, conn in configs:
        A = None if tau is None else (o >= tau).astype(int)
        sol = solve_ilp(m, R, start, A, k, conn)
        bf = brute_force(m, R, start, A, k, conn) if do_bf else None
        if sol is None:
            ok = (bf is None) if do_bf else True
            all_ok &= ok
            print(f"{name:24s} {'infeasible':>13s}  {'ok' if ok else 'XX'}")
            continue
        cost, path = sol
        ok = (bf is not None and abs(bf - cost) < 1e-6) if do_bf else True
        all_ok &= ok
        Astat = (o >= (tau if tau is not None else 0.3)).astype(int)
        st = reg_stats([i for i in path if i != start], Astat)
        print(f"{name:24s} {cost:6.0f} {bf if bf is not None else float('nan'):6.0f}  {'ok' if ok else 'XX'}  "
              f"{'-'.join(map(str, path)):28s} {st['edges']:3d} {st['min_deg']:6d} "
              f"{str(st['connected']):>4s} {st['cycle_rank']:4d} {st['bridges']:7d}")
    print("\nILP == brute force for all configurations:", all_ok if do_bf else "skipped (too many views)")
    print("(registration-graph statistics of the L0 row are evaluated at tau = 0.3)")


if __name__ == "__main__":
    main()
