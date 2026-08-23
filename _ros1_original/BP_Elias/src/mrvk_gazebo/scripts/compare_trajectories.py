#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vykresli viacero trajektorii (z CSV od trajectory_recorder.py) do jedneho
obrazka pre vizualne porovnanie.

Pouzitie:
  ./compare_trajectories.py teb.csv dwa.csv tpros.csv -o crossing.png
  ./compare_trajectories.py teb.csv dwa.csv -l TEB DWA -o out.png \
      --title "Scenár Crossing" --xlim -4 4 --ylim -2 5

CSV format ocakavany: hlavicka t,x,y,yaw; oddelovac ',' (default csv).
"""
import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")          # headless: zapis do suboru bez X servera
import matplotlib.pyplot as plt


def load_csv(path):
    xs, ys = [], []
    with open(path, "r") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
            except (KeyError, ValueError):
                continue
    return xs, ys


def default_label(path):
    return os.path.splitext(os.path.basename(path))[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csvs", nargs="+", help="CSV subory s trajektoriami")
    ap.add_argument("-l", "--labels", nargs="+",
                    help="popisky pre legendu (default: nazov suboru)")
    ap.add_argument("-o", "--output", default="trajectories.png",
                    help="vystupny obrazok (PNG/PDF/SVG)")
    ap.add_argument("--title", default=None, help="nadpis grafu")
    ap.add_argument("--xlim", nargs=2, type=float, default=[-7.5, 7.5],
                    metavar=("XMIN", "XMAX"))
    ap.add_argument("--ylim", nargs=2, type=float, default=[-7.5, 7.5],
                    metavar=("YMIN", "YMAX"))
    ap.add_argument("--linewidth", type=float, default=1.8)
    ap.add_argument("--dpi", type=int, default=200)
    ap.add_argument("--no-legend", action="store_true",
                    help="skry legendu")
    ap.add_argument("--start-marker", action="store_true",
                    help="kresli marker startu (kruh)")
    ap.add_argument("--end-marker", action="store_true",
                    help="kresli marker konca (krizik)")
    args = ap.parse_args()

    if args.labels and len(args.labels) != len(args.csvs):
        print("[compare] pocet --labels ({}) != pocet CSV ({})".format(
            len(args.labels), len(args.csvs)), file=sys.stderr)
        sys.exit(1)

    labels = args.labels if args.labels \
        else [default_label(p) for p in args.csvs]

    # Konzistentna farebna paleta pre TEB / DWA / TR (ak v nazve).
    color_map = {
        "teb":   "#1f77b4",
        "dwa":   "#d62728",
        "tpros": "#2ca02c",
        "tr":    "#2ca02c",
    }
    cycle = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    fig, ax = plt.subplots(figsize=(7, 6))

    for i, (path, label) in enumerate(zip(args.csvs, labels)):
        xs, ys = load_csv(path)
        if not xs:
            print("[compare] {} prazdne, preskakujem".format(path),
                  file=sys.stderr)
            continue
        key = label.lower().split("_")[0]
        color = color_map.get(key, cycle[i % len(cycle)])
        ax.plot(xs, ys, "-", color=color, linewidth=args.linewidth,
                label=label)
        if args.start_marker:
            ax.plot(xs[0], ys[0], "o", color=color, markersize=8,
                    markeredgecolor="black", markeredgewidth=0.8)
        if args.end_marker:
            ax.plot(xs[-1], ys[-1], "X", color=color, markersize=10,
                    markeredgecolor="black", markeredgewidth=0.8)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", alpha=0.6)
    if not args.no_legend:
        ax.legend(loc="best", framealpha=0.9)
    if args.title:
        ax.set_title(args.title)
    if args.xlim:
        ax.set_xlim(args.xlim)
    if args.ylim:
        ax.set_ylim(args.ylim)

    fig.tight_layout()
    fig.savefig(args.output, dpi=args.dpi)
    print("[compare] ulozene -> {}".format(args.output))


if __name__ == "__main__":
    main()
