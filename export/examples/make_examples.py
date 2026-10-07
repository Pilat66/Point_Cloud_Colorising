#!/usr/bin/env python3
"""Собирает мини-пример для кубовой раскраски из данных репозитория-источника.

Что делает (детерминированно, по возрастанию времени):

1. читает ``trajectory.csv`` и берёт интервал поз;
2. выбирает первый кластер из ``--photos`` снимков подряд (внутри интервала
   поз, разброс не больше ``--max-cluster-span`` секунд);
3. копирует эти снимки в ``photos/``, вырезает окно траектории вокруг них;
4. выгружает из карты (``all_raw_points.las``) только точки с ``gps_time``
   внутри окна, сохраняя формат LAS 1.4 point format 7, исходные scale/offset,
   координаты int32, intensity и время — тем самым пример воспроизводит
   реальный вход кубового режима;
5. копирует ``calib.json``.

Требуется ``laspy`` (для генерации; самому режиму laspy не нужен).

Пример запуска (из каталога export/):

    python3 examples/make_examples.py --margin 0.25 --photos 3

Пути по умолчанию считаются от каталога репозитория-источника
(``<repo>/export/examples`` -> ``<repo>/data``), поэтому скрипт не содержит
абсолютных путей.
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

try:
    import laspy
    import numpy as np
except Exception as exc:  # pragma: no cover - сообщение вместо трассировки
    sys.exit("нужны laspy и numpy: {}".format(exc))

CHUNK = 2_000_000


def load_trajectory(path):
    """Возвращает (times, rows) для CSV вида time;x;y;z;roll;pitch;yaw."""
    times, rows = [], []
    with open(path) as f:
        header = f.readline()
        if header.split(";")[0].strip().lower() not in ("time", "stamp", "t", "timestamp"):
            f.seek(0)
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split(";") if ";" in line else line.split(",")
            try:
                times.append(float(v[0]))
            except ValueError:
                continue
            rows.append(line)
    order = sorted(range(len(times)), key=lambda i: times[i])
    return [times[i] for i in order], [rows[i] for i in order]


def photo_times(directory):
    """(time_seconds, filename) по именам файлов с расширением png/jpg."""
    out = []
    for name in sorted(os.listdir(directory)):
        stem, _, ext = name.rpartition(".")
        if ext.lower() not in ("png", "jpg", "jpeg", "bmp"):
            continue
        digits = "".join(ch if ch.isdigit() else " " for ch in stem).split()
        if not digits:
            continue
        raw = float(max(digits, key=len))
        if raw >= 1e15:
            raw /= 1e9
        elif raw >= 1e11:
            raw /= 1e3
        out.append((raw, name))
    out.sort()
    return out


def pick_cluster(photos, t_min, t_max, count, max_span):
    inside = [(t, n) for t, n in photos if t_min <= t <= t_max]
    for i in range(len(inside) - count + 1):
        if inside[i + count - 1][0] - inside[i][0] <= max_span:
            return inside[i : i + count]
    raise SystemExit("не нашлось {} снимков внутри интервала траектории".format(count))


def write_map(src_path, dst_path, t_lo, t_hi):
    """Потоково копирует точки с gps_time в [t_lo, t_hi] в LAS 1.4 fmt 7."""
    kept = 0
    seen = 0
    with laspy.open(src_path) as src:
        src_hdr = src.header
        out_hdr = laspy.LasHeader(point_format=7, version="1.4")
        out_hdr.scales = src_hdr.scales
        out_hdr.offsets = src_hdr.offsets
        out_hdr.system_identifier = "PointCloudColorising"
        out_hdr.generating_software = "make_examples.py"
        with laspy.open(dst_path, mode="w", header=out_hdr) as dst:
            for chunk in src.chunk_iterator(CHUNK):
                seen += len(chunk)
                t = np.asarray(chunk.gps_time, np.float64)
                mask = (t >= t_lo) & (t <= t_hi)
                if not mask.any():
                    continue
                idx = np.nonzero(mask)[0]
                rec = laspy.ScaleAwarePointRecord.zeros(idx.size, header=out_hdr)
                rec.X = np.asarray(chunk.X, np.int32)[idx]
                rec.Y = np.asarray(chunk.Y, np.int32)[idx]
                rec.Z = np.asarray(chunk.Z, np.int32)[idx]
                rec.intensity = np.asarray(chunk.intensity, np.uint16)[idx]
                rec.gps_time = t[idx]
                zeros = np.zeros(idx.size, np.uint16)
                rec.red = zeros
                rec.green = zeros
                rec.blue = zeros
                dst.write_points(rec)
                kept += idx.size
    return kept, seen


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=os.path.join(root, "data"),
                    help="каталог данных репозитория-источника")
    ap.add_argument("--out", default=here, help="куда писать пример")
    ap.add_argument("--photos-dir", default=None,
                    help="каталог снимков (по умолчанию <data>/img-uniq)")
    ap.add_argument("--photos", type=int, default=3, help="сколько снимков взять")
    ap.add_argument("--max-cluster-span", type=float, default=2.0,
                    help="максимальный разброс времени снимков кластера, с")
    ap.add_argument("--margin", type=float, default=0.25,
                    help="запас окна вокруг снимков, с")
    args = ap.parse_args(argv)

    photos_dir = args.photos_dir
    if photos_dir is None:
        for cand in ("img-uniq", "img-uniq-filtered"):
            if os.path.isdir(os.path.join(args.data, cand)):
                photos_dir = os.path.join(args.data, cand)
                break
    if photos_dir is None:
        sys.exit("не найден каталог снимков в " + args.data)

    traj_path = os.path.join(args.data, "trajectory.csv")
    map_path = os.path.join(args.data, "all_raw_points.las")
    calib_path = os.path.join(args.data, "calib.json")
    for p in (traj_path, map_path, calib_path):
        if not os.path.exists(p):
            sys.exit("нет файла: " + p)

    times, rows = load_trajectory(traj_path)
    cluster = pick_cluster(photo_times(photos_dir), times[0], times[-1],
                           args.photos, args.max_cluster_span)
    t_lo = cluster[0][0] - args.margin
    t_hi = cluster[-1][0] + args.margin

    out_photos = os.path.join(args.out, "photos")
    os.makedirs(out_photos, exist_ok=True)
    os.makedirs(os.path.join(args.out, "golden"), exist_ok=True)
    for _, name in cluster:
        shutil.copy2(os.path.join(photos_dir, name),
                     os.path.join(out_photos, name))

    with open(os.path.join(args.out, "trajectory.csv"), "w") as f:
        f.write("time;x;y;z;roll;pitch;yaw\n")
        for t, row in zip(times, rows):
            if t_lo <= t <= t_hi:
                f.write(row + "\n")

    kept, seen = write_map(map_path, os.path.join(args.out, "map.las"), t_lo, t_hi)
    shutil.copy2(calib_path, os.path.join(args.out, "calib.json"))

    print("источник снимков : {}".format(photos_dir))
    print("кластер          : {} снимков {} .. {}".format(
        len(cluster), cluster[0][1], cluster[-1][1]))
    print("окно             : {:.3f} .. {:.3f} ({:.2f} с)".format(
        t_lo, t_hi, t_hi - t_lo))
    print("карта            : {} точек из {} (окно по gps_time)".format(kept, seen))
    print("записано         : photos/, trajectory.csv, map.las, calib.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

