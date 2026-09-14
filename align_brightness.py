#!/usr/bin/env python3
"""
Выравнивание яркости серии фотографий.

Эталон — кадр с максимальным значением выбранной статистики яркости
(--stat). Остальные кадры приводятся к эталону либо масштабированием
канала L (--method gain, по умолчанию), либо сопоставлением гистограмм
канала L (--method hist).

Статистики яркости (--stat), канал L пространства LAB (0-255):
    mean    средняя яркость кадра (поведение старой версии скрипта)
    median  медиана яркости кадра
    top50   среднее самых ярких 50% пикселей
    top25   среднее самых ярких 25% пикселей (по умолчанию)
    max     робастный максимум: 99-й перцентиль (сырой максимум пикселя на
            реальных кадрах упирается в 255 из-за неба и не различает кадры)

Использование:
    python align_brightness.py /path/to/photos
    python align_brightness.py /path/to/photos --stat top50
    python align_brightness.py /path/to/photos --stat median --method hist
    python align_brightness.py /path/to/photos --dry-run

Результат:
    В папке /path/to/photos создаётся подпапка (--out-dir, по умолчанию
    "aligned"), куда сохраняются выровненные копии всех фото (включая эталон).

Зависимости: numpy, opencv-python (scikit-image не требуется).
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

DEFAULT_EXTENSIONS = ("jpg", "jpeg", "png", "bmp", "tif", "tiff")
STATS = ("max", "top25", "top50", "median", "mean")


def parse_extensions(value):
    """Строка "png, .JPG" -> ("png", "jpg")."""
    exts = []
    for part in value.replace(";", ",").split(","):
        part = part.strip().lstrip("*.").lower()
        if part:
            exts.append(part)
    return tuple(dict.fromkeys(exts)) or DEFAULT_EXTENSIONS


def find_images(folder, extensions):
    """Все изображения с заданными расширениями в папке (без рекурсии)."""
    files = []
    for ext in extensions:
        files.extend(glob.glob(os.path.join(folder, "*." + ext)))
        files.extend(glob.glob(os.path.join(folder, "*." + ext.upper())))
    return sorted(set(files))


def lab_l(bgr):
    """Канал L пространства LAB (0-255, uint8): и измеряем, и правим только его."""
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)[:, :, 0]


def brightness_score(bgr, stat):
    """
    Яркость кадра по выбранной статистике канала L.

    mean   — средняя яркость
    median — медиана яркости
    top50  — среднее самых ярких 50% пикселей
    top25  — среднее самых ярких 25% пикселей
    max    — робастный максимум (99-й перцентиль)
    """
    l = lab_l(bgr).astype(np.float32).ravel()
    if stat == "mean":
        return float(l.mean())
    if stat == "median":
        return float(np.median(l))
    if stat == "max":
        return float(np.percentile(l, 99.0))
    frac = 0.25 if stat == "top25" else 0.5
    n = l.size
    m = max(1, min(n, int(round(n * frac))))
    return float(np.partition(l, n - m)[n - m:].mean())


def apply_gain(source_bgr, gain):
    """
    Выравнивание яркости масштабированием канала L (LAB) на коэффициент gain.
    Каналы a/b не трогаются, поэтому цвет сохраняется; значения обрезаются в 0-255.
    """
    if abs(gain - 1.0) < 1e-3:
        return source_bgr.copy()
    src_lab = cv2.cvtColor(source_bgr, cv2.COLOR_BGR2LAB)
    lut = np.clip(np.rint(np.arange(256, dtype=np.float64) * gain), 0, 255).astype(np.uint8)
    src_lab[:, :, 0] = cv2.LUT(src_lab[:, :, 0], lut)
    return cv2.cvtColor(src_lab, cv2.COLOR_LAB2BGR)


def match_histograms_lab(source_bgr, reference_bgr):
    """
    Подгоняет яркость source под reference, сохраняя цвета.
    Сопоставление гистограмм только по каналу L пространства LAB —
    замена skimage.exposure.match_histograms на numpy/cv2 (scikit-image не нужен).
    """
    src_lab = cv2.cvtColor(source_bgr, cv2.COLOR_BGR2LAB)
    ref_lab = cv2.cvtColor(reference_bgr, cv2.COLOR_BGR2LAB)

    src_hist = np.bincount(src_lab[:, :, 0].ravel(), minlength=256).astype(np.float64)
    ref_hist = np.bincount(ref_lab[:, :, 0].ravel(), minlength=256).astype(np.float64)
    src_cdf = np.cumsum(src_hist) / max(src_hist.sum(), 1.0)
    ref_cdf = np.cumsum(ref_hist) / max(ref_hist.sum(), 1.0)
    lut = np.interp(src_cdf, ref_cdf, np.arange(256)).astype(np.uint8)

    src_lab[:, :, 0] = cv2.LUT(src_lab[:, :, 0], lut)
    return cv2.cvtColor(src_lab, cv2.COLOR_LAB2BGR)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Выравнивание яркости серии фотографий по самому яркому кадру.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "статистики яркости --stat (канал L пространства LAB, 0-255):\n"
            "  mean    средняя яркость кадра (как в старой версии скрипта)\n"
            "  median  медиана яркости кадра\n"
            "  top50   среднее самых ярких 50% пикселей\n"
            "  top25   среднее самых ярких 25% пикселей (по умолчанию)\n"
            "  max     робастный максимум (99-й перцентиль): сырой максимум\n"
            "          пикселя на реальных кадрах упирается в 255 из-за неба\n"
        ),
    )
    ap.add_argument("folder", help="папка с фотографиями")
    ap.add_argument("--stat", choices=STATS, default="top25",
                    help="статистика яркости: эталон = кадр с её максимумом, "
                         "к ней же приводится остальные (по умолчанию %(default)s)")
    ap.add_argument("--method", choices=("gain", "hist"), default="gain",
                    help="gain — масштабирование канала L до статистики эталона "
                         "(по умолчанию); hist — сопоставление гистограмм канала L")
    ap.add_argument("--out-dir", default="aligned",
                    help="имя подпапки результата, абсолютный путь тоже допустим "
                         "(по умолчанию %(default)s)")
    ap.add_argument("--max-gain", type=float, default=4.0,
                    help="ограничить гейн диапазоном [1/max-gain, max-gain], "
                         "0 — без ограничения (по умолчанию %(default)s)")
    ap.add_argument("--no-resize", action="store_true",
                    help="не приводить кадры к размеру эталона (такие кадры пропускаются)")
    ap.add_argument("--ext", default=",".join(DEFAULT_EXTENSIONS),
                    help="расширения изображений через запятую (по умолчанию %(default)s)")
    ap.add_argument("--dry-run", action="store_true",
                    help="только анализ и расчёт, файлы не записываются")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    folder = args.folder
    if not os.path.isdir(folder):
        print(f"Папка не найдена: {folder}")
        return 1

    image_files = find_images(folder, parse_extensions(args.ext))
    if len(image_files) < 2:
        print("В папке найдено меньше двух изображений — нечего выравнивать.")
        return 1

    # Папка результата внутри исходной (по умолчанию "aligned")
    out_dir = args.out_dir if os.path.isabs(args.out_dir) else os.path.join(folder, args.out_dir)
    if os.path.abspath(out_dir) == os.path.abspath(folder):
        print("Папка результата совпадает с исходной — исходники были бы перезаписаны.")
        return 1

    print(f"Найдено изображений: {len(image_files)}")
    print(f"Статистика яркости: {args.stat} (канал L, LAB)")

    # 1. Анализ яркости: эталон — кадр с максимальным значением статистики
    print("\nАнализ яркости...")
    scores = {}
    for path in image_files:
        img = cv2.imread(path)
        if img is None:
            scores[path] = None
            print(f"  {os.path.basename(path):40s} {args.stat}: n/a (не прочитан)")
            continue
        scores[path] = brightness_score(img, args.stat)
        print(f"  {os.path.basename(path):40s} {args.stat}: {scores[path]:6.2f}")

    valid = [p for p in image_files if scores[p] is not None]
    if not valid:
        print("Ни одного изображения прочитать не удалось.")
        return 1

    reference_path = max(valid, key=lambda p: scores[p])
    reference_score = scores[reference_path]
    print(f"\n>>> Эталон (максимум {args.stat}): {os.path.basename(reference_path)} "
          f"({args.stat} = {reference_score:.2f})")

    # 2. Загрузка эталона
    reference_img = cv2.imread(reference_path)
    if reference_img is None:
        print("Не удалось загрузить эталонное изображение.")
        return 1

    # 3. Создание выходной папки
    if args.dry_run:
        print("\n[--dry-run] выходная папка не создаётся: " + out_dir)
    else:
        os.makedirs(out_dir, exist_ok=True)

    # 4. Выравнивание всех фото (эталон копируется как есть)
    print(f"\nВыравнивание (метод: {args.method})...")
    aligned_scores = {}
    gain_limit = abs(args.max_gain) if args.max_gain else 0.0

    for path in image_files:
        filename = os.path.basename(path)
        out_path = os.path.join(out_dir, filename)

        if path == reference_path:
            aligned_scores[path] = reference_score
            if not args.dry_run:
                cv2.imwrite(out_path, reference_img)
            print(f"  [эталон]  {filename}  {args.stat} {reference_score:6.2f}")
            continue

        src_img = cv2.imread(path)
        if src_img is None:
            print(f"  [пропуск] {filename} — не удалось загрузить")
            continue

        src_score = scores[path]

        # Совпадение размеров обязательно и для LUT, и для сопоставления гистограмм
        if src_img.shape != reference_img.shape:
            if args.no_resize:
                print(f"  [пропуск] {filename} — размер не совпадает с эталоном (--no-resize)")
                continue
            src_img = cv2.resize(
                src_img,
                (reference_img.shape[1], reference_img.shape[0]),
                interpolation=cv2.INTER_AREA,
            )
            src_score = brightness_score(src_img, args.stat)
            print(f"  [resize]  {filename} — приведён к размеру эталона")

        note = ""
        if args.method == "gain":
            if src_score <= 1e-6:
                print(f"  [пропуск] {filename} — {args.stat} = 0, гейн не определён")
                continue
            gain = reference_score / src_score
            if gain_limit and abs(gain - 1.0) > 1e-9:
                clamped = min(max(gain, 1.0 / gain_limit), gain_limit)
                if abs(clamped - gain) > 1e-9:
                    gain = clamped
                    note = f"  (гейн ограничен --max-gain {args.max_gain:g})"
            aligned = apply_gain(src_img, gain)
            gain_text = f"gain {gain:6.3f}  "
            if not note and not (0.5 <= gain <= 2.0):
                note = "  (внимание: гейн вне [0.5, 2.0])"
        else:
            gain = None
            aligned = match_histograms_lab(src_img, reference_img)
            gain_text = ""

        after = brightness_score(aligned, args.stat)
        aligned_scores[path] = after
        status = "[dry-run]" if args.dry_run else "[готово] "
        if not args.dry_run:
            cv2.imwrite(out_path, aligned)
        print(f"  {status} {filename}  {gain_text}{args.stat} {src_score:6.2f} -> {after:6.2f}{note}")

    # 5. Сводка: разброс выбранной статистики до и после выравнивания
    before = [scores[p] for p in image_files if scores[p] is not None]
    after_scores = [aligned_scores[p] for p in image_files if aligned_scores.get(p) is not None]
    if before:
        print(f"\nРазброс {args.stat} до выравнивания:    {max(before) - min(before):6.2f} "
              f"({min(before):.2f} .. {max(before):.2f})")
    if after_scores:
        print(f"Разброс {args.stat} после выравнивания: {max(after_scores) - min(after_scores):6.2f} "
              f"({min(after_scores):.2f} .. {max(after_scores):.2f})")

    if args.dry_run:
        print("\n[--dry-run] файлы не записаны.")
    else:
        print(f"\nВсе изображения сохранены в: {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
