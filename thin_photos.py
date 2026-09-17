#!/usr/bin/env python3
"""Прореживание серии фотографий: копирует снимки с заданным интервалом.

Из каталога-источника в каталог назначения копируются фотографии, отстоящие
от предыдущего скопированного снимка не менее чем на --interval секунд.
Время снимка берётся из имени файла (эпоха в нс/мс/с, как в colorise_offline).
Имена файлов сохраняются.

Использование:
    python3 thin_photos.py <источник> <назначение> [--interval 1.0]
"""
import argparse
import os
import shutil
import sys

import colorise_offline as co

EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help="каталог с фотографиями")
    ap.add_argument("dest", help="каталог назначения (создаётся при отсутствии)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="минимальный интервал между копируемыми снимками, с [1.0]")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.interval <= 0:
        raise SystemExit("--interval должен быть > 0")
    if not os.path.isdir(args.source):
        raise SystemExit("нет такого каталога: " + args.source)
    os.makedirs(args.dest, exist_ok=True)
    if os.path.abspath(args.source) == os.path.abspath(args.dest):
        raise SystemExit("каталоги источника и назначения совпадают")

    photos = []
    for name in sorted(os.listdir(args.source)):
        path = os.path.join(args.source, name)
        if not os.path.isfile(path) or os.path.splitext(name)[1].lower() not in EXTS:
            continue
        t = co.photo_timestamp_from_name(os.path.splitext(name)[0])
        if t is None:
            print("[skip] в имени нет времени: {}".format(name))
            continue
        photos.append((t, name))
    photos.sort()
    if not photos:
        raise SystemExit("в {} нет фотографий с временем в имени".format(args.source))

    picked = []
    last = None
    for t, name in photos:
        if last is None or t - last >= args.interval:
            shutil.copy2(os.path.join(args.source, name), os.path.join(args.dest, name))
            picked.append((t, name))
            last = t

    span = photos[-1][0] - photos[0][0]
    print("[photos] в источнике: {}, интервал {:.3g} с -> скопировано: {}".format(
        len(photos), args.interval, len(picked)))
    print("[span] {:.3f} .. {:.3f} ({:.1f} с)".format(photos[0][0], photos[-1][0], span))
    if picked:
        gaps = [b[0] - a[0] for a, b in zip(picked, picked[1:])]
        print("[gaps] min={:.3f} с max={:.3f} с".format(min(gaps), max(gaps)))
        for t, name in picked:
            print("  {:18.6f} {}".format(t, name))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
