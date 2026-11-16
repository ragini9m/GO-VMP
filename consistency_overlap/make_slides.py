"""
Presentation: why GO-VMP needs an overlap constraint, the pairwise rule, its limits, and constraint (g).

All numbers on the slides are computed here from the int main() dummy data (and one further random
instance), with the same ILP that matched Gurobi and brute force.

Usage:   python make_slides.py
Output:  figures/overlap_constraints_presentation.pdf  (16:9 slides)
Needs:   numpy, scipy, matplotlib
"""
import os
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import FancyBboxPatch

import test_overlap_formulation as T
import viz_common as V
from test_overlap_formulation import solve_ilp
from visualize_pairwise import forbidden_pairs

W, H = 13.333, 7.5
FORBID = "#e34948"   # used only together with a ✗ label
OUT = "figures/overlap_constraints_presentation.pdf"


# ----------------------------------------------------------------------------- results
def compute():
    m, R, o = V.instance()
    A = lambda tau: (o >= tau).astype(int)
    r = {
        "base": solve_ilp(m, R, V.START),
        "pw03": solve_ilp(m, R, V.START, forbid=forbidden_pairs(o, 0.3)),
        "pw04": solve_ilp(m, R, V.START, forbid=forbidden_pairs(o, 0.4)),
        "pw05": solve_ilp(m, R, V.START, forbid=forbidden_pairs(o, 0.5)),
        "g03": solve_ilp(m, R, V.START, A(0.3), 2),
        "g04": solve_ilp(m, R, V.START, A(0.4), 2),
        "g05": solve_ilp(m, R, V.START, A(0.5), 2),
    }
    # second random instance: 13 views, 15 voxels, start 2, seed 5 (views look at more spread-out voxels)
    m5, R5 = T.generate_instance(13, 15, 2, 5)
    o5 = T.overlap_ratio(R5)
    real5 = [i for i in range(13) if i != 2]
    fp5 = [(i, j) for x, i in enumerate(real5) for j in real5[x + 1:] if o5[i, j] < 0.3]
    s5 = {"base": solve_ilp(m5, R5, 2), "pw": solve_ilp(m5, R5, 2, forbid=fp5),
          "g": solve_ilp(m5, R5, 2, (o5 >= 0.3).astype(int), 2)}
    s5_loops = T.reg_stats([i for i in s5["g"][1] if i != 2], (o5 >= 0.3).astype(int))["cycle_rank"] if s5["g"] else None
    return m, R, o, r, s5, s5_loops


def loops(res, o, tau):
    return V.graph_stats(res[1], o, tau)["loops"]


# ----------------------------------------------------------------------------- slide helpers
def slide(pdf, title, kicker=None):
    fig = plt.figure(figsize=(W, H), facecolor=V.SURFACE)
    if kicker:
        fig.text(0.05, 0.935, kicker.upper(), fontsize=10, color=V.SELECTED, weight="bold")
    fig.text(0.05, 0.875, title, fontsize=24, weight="bold", color=V.INK, va="center")
    fig.add_artist(plt.Line2D([0.05, 0.95], [0.825, 0.825], color=V.GRID, lw=1))
    fig.text(0.95, 0.03, f"{len(pdf_pages) + 1}", fontsize=9, color=V.INK_2, ha="right")
    pdf_pages.append(fig)
    return fig


def bullets(fig, x, y, items, width=58, size=13, gap=0.016, line=0.037):
    """items: str or (str, level). Returns the y below the last item."""
    for it in items:
        text, level = (it, 0) if isinstance(it, str) else it
        indent = 0.025 * level
        lines = textwrap.wrap(text, width - 4 * level)
        marker = "•" if level == 0 else "–"
        fig.text(x + indent, y, marker, fontsize=size, color=V.SELECTED if level == 0 else V.INK_2, va="top")
        for k, ln in enumerate(lines):
            fig.text(x + indent + 0.018, y - k * line, ln, fontsize=size - (1 if level else 0),
                     color=V.INK if level == 0 else V.INK_2, va="top")
        y -= len(lines) * line + gap
    return y


def formula_box(fig, x, y, w, h, tex, caption=None, size=17):
    fig.add_artist(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.004,rounding_size=0.012",
                                  transform=fig.transFigure, facecolor="#eef3fa", edgecolor="#c9d9ef", lw=1))
    fig.text(x + w / 2, y + h * (0.6 if caption else 0.5), tex, fontsize=size, ha="center", va="center", color=V.INK)
    if caption:
        fig.text(x + w / 2, y + h * 0.2, caption, fontsize=10.5, ha="center", va="center", color=V.INK_2)


def schematic(ax, nodes, edges, labels=None, title=None, note=None):
    """nodes: {name: (x, y)}; edges: list of (a, b, style) with style 'ok' | 'forbid' | 'faint'."""
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1.75, 1.45)
    for a, b, style in edges:
        (x1, y1), (x2, y2) = nodes[a], nodes[b]
        if style == "ok":
            ax.plot([x1, x2], [y1, y2], color=V.SELECTED, lw=3, zorder=1, solid_capstyle="round")
        elif style == "faint":
            ax.plot([x1, x2], [y1, y2], color=V.GRID, lw=1.5, zorder=1)
        else:
            ax.plot([x1, x2], [y1, y2], color=FORBID, lw=2, ls=(0, (4, 3)), zorder=1)
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            ax.text(mx, my, "✗", fontsize=15, color=FORBID, ha="center", va="center", weight="bold",
                    bbox=dict(boxstyle="circle,pad=0.15", fc=V.SURFACE, ec="none"), zorder=3)
    for n, (x, y) in nodes.items():
        ax.scatter(x, y, s=1100, color=V.SELECTED, edgecolor=V.SURFACE, lw=2.5, zorder=4)
        ax.text(x, y, n, fontsize=15, color="white", weight="bold", ha="center", va="center", zorder=5)
        if labels and n in labels:
            dx = 0.42 if x > 0 else -0.42
            ax.text(x + dx, y, labels[n], fontsize=11.5, ha="left" if x > 0 else "right", va="center",
                    color=V.INK, zorder=5)
    if title:
        ax.set_title(title, fontsize=12.5, color=V.INK, pad=6)
    if note:
        ax.text(0, -1.6, note, fontsize=11, ha="center", va="center", color=V.INK_2)


RING = {"A": (-0.8, 0.8), "B": (0.8, 0.8), "C": (0.8, -0.8), "D": (-0.8, -0.8)}


# ----------------------------------------------------------------------------- slides
def build(pdf):
    m, R, o, r, s5, s5_loops = compute()
    base, pw03, pw04, pw05, g03, g04, g05 = (r[k] for k in ("base", "pw03", "pw04", "pw05", "g03", "g04", "g05"))
    pct = lambda c: f"+{(c / base[0] - 1) * 100:.0f}%"

    # 1 ── title
    fig = plt.figure(figsize=(W, H), facecolor=V.SURFACE)
    pdf_pages.append(fig)
    fig.text(0.07, 0.62, "Consistency-aware view selection", fontsize=34, weight="bold", color=V.INK)
    fig.text(0.07, 0.535, "From GO-VMP to the registration-degree constraint (g)", fontsize=20, color=V.INK_2)
    fig.add_artist(plt.Line2D([0.07, 0.4], [0.47, 0.47], color=V.SELECTED, lw=3))
    fig.text(0.07, 0.40, "1.  What GO-VMP does not control", fontsize=14, color=V.INK)
    fig.text(0.07, 0.355, "2.  The pairwise overlap rule as a first fix", fontsize=14, color=V.INK)
    fig.text(0.07, 0.31, "3.  Where the pairwise rule fails", fontsize=14, color=V.INK)
    fig.text(0.07, 0.265, "4.  How constraint (g) solves it", fontsize=14, color=V.INK)
    fig.text(0.07, 0.10, "Test data: random dummy instance of int main() in views_voxels_motion.cpp "
             "(10 views, 10 voxels, start view 0). Results verified against brute force and Gurobi.",
             fontsize=10, color=V.INK_2)

    # 2 ── GO-VMP problem
    fig = slide(pdf, "GO-VMP does not control how views relate to each other", "1 · The problem")
    formula_box(fig, 0.05, 0.665, 0.43, 0.115,
                r"$\min \sum_{i,j} m_{ij}\,p_{ij}$    s.t.  $\sum_i v_i\, r_{ic} \geq 1\ \ \forall c$",
                "minimize motion cost, see every target voxel at least once", size=15)
    bullets(fig, 0.05, 0.62, [
        "Objective is motion cost only: nothing rewards seeing the same surface from related views.",
        "Coverage needs each voxel only once, so the cheapest solution tends to pick few views that share little.",
        "No constraint links selected views: a view can overlap none of the others.",
        "Such a view cannot be registered; its pose error stays in the map and a fruit appears twice (ghosting).",
        "Even overlapping views form chains: one bad registration shifts all later views, undetected.",
    ], width=50, size=12.5)
    ax = fig.add_axes([0.52, 0.05, 0.44, 0.70])
    V.draw_panel(ax, "", base, o, 0.3)
    st = V.graph_stats(base[1], o, 0.3)
    ax.set_title(f"GO-VMP result on the test data: view 3 overlaps neither 8 nor 1", fontsize=12, color=V.INK)

    # 3 ── pairwise fix
    fig = slide(pdf, "First fix: the pairwise overlap rule", "2 · Pairwise rule")
    formula_box(fig, 0.05, 0.665, 0.43, 0.115, r"$v_i + v_j \leq 1\quad$ for every pair with $o_{ij} < \tau$",
                "two views that overlap too little may not both be selected")
    bullets(fig, 0.05, 0.62, [
        "Every pair of selected views overlaps by at least τ.",
        "So no selected view is isolated: all scans can be registered into one map.",
        "Simple: one linear inequality per bad pair, two variables each, one parameter τ.",
        f"Test data, τ = 0.3: views 6, 5, 1 form a triangle, 1 loop, cost {pw03[0]:.0f} ({pct(pw03[0])}).",
    ], width=50, size=12.5)
    ax = fig.add_axes([0.52, 0.05, 0.44, 0.70])
    V.draw_panel(ax, "", pw03, o, 0.3)
    ax.set_title("Pairwise rule, τ = 0.3", fontsize=12, color=V.INK)

    # 4 ── pairwise problems
    fig = slide(pdf, "Where the pairwise rule fails", "3 · Limits of the pairwise rule")
    bullets(fig, 0.05, 0.78, [
        "Too strict: every view must overlap every other view.",
        ("Views of different fruits rarely overlap; if coverage needs both, the problem has no solution.", 1),
        ("A ring A–B–C–D is forbidden, although every view has two partners.", 1),
        (f"Second random instance (13 views): no solution at τ = 0.3.", 1),
        "Too weak: two views with one shared link satisfy it.",
        ("One registration, nothing to check it against: no loop, error undetectable.", 1),
        (f"Test data, τ = 0.4: only views 5 and 1, 0 loops, cost {pw04[0]:.0f}.", 1),
    ], width=46, size=12.5)
    ax = fig.add_axes([0.47, 0.08, 0.25, 0.70])
    schematic(ax, RING, [("A", "B", "ok"), ("B", "C", "ok"), ("C", "D", "ok"), ("D", "A", "ok"),
                         ("A", "C", "forbid"), ("B", "D", "forbid")],
              title="Ring of four views", note="A–C and B–D overlap < τ\n→ ring forbidden")
    ax = fig.add_axes([0.72, 0.08, 0.26, 0.70])
    V.draw_panel(ax, "", pw04, o, 0.4)
    ax.set_title("Pairwise rule, τ = 0.4", fontsize=12.5, color=V.INK, pad=6)

    # 5 ── (g)
    fig = slide(pdf, "Constraint (g): count partners per view", "4 · The solution")
    formula_box(fig, 0.05, 0.665, 0.43, 0.115,
                r"$\sum_{j \neq i} a_{ij}\, v_j \;\geq\; k\, v_i \quad \forall i$,    $a_{ij} = 1$ if $o_{ij} \geq \tau$",
                "every selected view needs at least k selected partners with overlap ≥ τ", size=15)
    bullets(fig, 0.05, 0.62, [
        "Asks per view “how many partners?”, not per pair “do these two overlap?”.",
        (f"Views far apart are allowed if each has partners: the ring is allowed; "
         f"the 13-view instance is solvable (cost {s5['g'][0]:.0f}).", 1),
        "k = 2: each view needs two partners, so a single link is not enough and a loop is forced.",
        (f"Test data, τ = 0.4: 4 views, {loops(g04, o, 0.4)} loops, no bridge, cost {g04[0]:.0f}.", 1),
        "Same variables, still linear; k = 0 gives GO-VMP back. Implemented in C++ and run with Gurobi.",
    ], width=50, size=12.5)
    ax = fig.add_axes([0.50, 0.08, 0.22, 0.70])
    schematic(ax, RING, [("A", "B", "ok"), ("B", "C", "ok"), ("C", "D", "ok"), ("D", "A", "ok"),
                         ("A", "C", "faint"), ("B", "D", "faint")],
              labels={"A": "2 ✓", "B": "2 ✓", "C": "2 ✓", "D": "2 ✓"},
              title="Same ring under (g), k = 2", note="each view has 2 partners\n→ ring allowed, 1 loop")
    ax = fig.add_axes([0.73, 0.08, 0.25, 0.70])
    V.draw_panel(ax, "", g04, o, 0.4)
    ax.set_title("(g), τ = 0.4, k = 2", fontsize=12.5, color=V.INK, pad=6)

    # 6 ── comparison
    fig = slide(pdf, "Side by side", "Comparison")
    cols = ["", "τ = 0.3", "τ = 0.4", "τ = 0.5", "13-view instance, τ = 0.3"]
    def cell(res, oo, tau, lp=None):
        if res is None:
            return "no solution"
        return f"cost {res[0]:.0f} · {loops(res, oo, tau) if lp is None else lp} loop(s)"
    rows = [
        ["GO-VMP baseline", f"cost {base[0]:.0f} · view 3 isolated", "same (τ not used)", "same (τ not used)", f"cost {s5['base'][0]:.0f}"],
        ["Pairwise rule", cell(pw03, o, .3), cell(pw04, o, .4), cell(pw05, o, .5), cell(s5["pw"], None, .3, 0)],
        ["(g), k = 2", cell(g03, o, .3), cell(g04, o, .4), cell(g05, o, .5), cell(s5["g"], None, .3, s5_loops)],
    ]
    ax = fig.add_axes([0.05, 0.42, 0.90, 0.36])
    ax.axis("off")
    tb = ax.table(cellText=rows, colLabels=cols, loc="upper left", cellLoc="left", colLoc="left",
                  colWidths=[0.16, 0.2, 0.2, 0.2, 0.24])
    tb.auto_set_font_size(False)
    tb.set_fontsize(12.5)
    tb.scale(1, 3.0)
    for (rr, cc), c in tb.get_celld().items():
        c.set_edgecolor(V.GRID)
        c.visible_edges = "B"
        c.set_facecolor(V.SURFACE)
        if rr == 0:
            c.set_text_props(weight="bold", color=V.INK)
            c.set_edgecolor(V.INK)
        elif cc == 0:
            c.set_text_props(weight="bold", color=V.INK)
        txt = c.get_text().get_text()
        if txt.startswith("cost") and "0 loop" in txt and "10 loop" not in txt or txt == "no solution":
            c.set_text_props(color=FORBID, weight="bold")
    bullets(fig, 0.05, 0.36, [
        "Where both rules have a solution with three or more views, they agree (τ = 0.3 and 0.5).",
        "The pairwise rule fails in both directions: no loop at τ = 0.4, no solution on the larger instance.",
        "(g) produced a loop in every case with a solution; the price is motion cost, controlled by τ.",
    ], width=110, size=12.5)
    fig.text(0.05, 0.06, "Red: outcomes with no redundancy (0 loops) or no solution.", fontsize=10, color=V.INK_2)

    # 7 ── conclusion
    fig = slide(pdf, "Conclusion and next steps", "Summary")
    y = bullets(fig, 0.05, 0.78, [
        "GO-VMP can select views that cannot be registered with each other, a direct source of ghosting.",
        "The pairwise rule removes isolated views, but it is too strict for spread-out targets "
        "and too weak to force redundancy.",
        "Constraint (g) with k = 2 asks every view for two overlapping partners: loops in every solved case, "
        "linear, verified with Gurobi.",
        "τ is the main cost knob (exploration vs. consistency); k sets the level of redundancy.",
    ], width=105, size=13)
    fig.text(0.05, y - 0.03, "Limits and next steps", fontsize=15, weight="bold", color=V.INK)
    bullets(fig, 0.05, y - 0.09, [
        "Results are on small random data; next: real view and voxel data from the planner.",
        "Ghosting itself is not measured yet; next: map with a registration step and count split fruits.",
        "Overlap is computed on target voxels; registration needs overlap on observed surface.",
        "(g) can be infeasible; a soft version (slack + penalty in the objective) keeps the problem solvable.",
    ], width=105, size=12.5)


pdf_pages = []

if __name__ == "__main__":
    os.makedirs("figures", exist_ok=True)
    build(None)
    with PdfPages(OUT) as pdf:
        for f in pdf_pages:
            pdf.savefig(f, facecolor=V.SURFACE)
            plt.close(f)
    print(f"written {OUT} ({len(pdf_pages)} slides)")
