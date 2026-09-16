#!/usr/bin/env python3
"""
Тестовое облако, параллельное плоскости фотографии.

Назначение
    Создать детерминированную фикстуру для проверки алгоритмов раскраски:
    облако точек в плоскости, параллельной плоскости изображения выбранного
    снимка, на заданном расстоянии от камеры. Скрипт НИЧЕГО не раскрашивает —
    он готовит только облако.

    Одно и то же облако прогоняется через разные алгоритмы
    (colorise_offline C++ и Python, ColoriseMap, будущие оптимизированные
    версии); результаты сравниваются между собой и с эталоном.

    Точки считаются порциями (--chunk, по умолчанию 1000 штук) и сразу
    пишутся в файлы, поэтому память не зависит от размера фикстуры.

    Дополнительно в конец облака добавляются 100 точек (25 на каждую сторону) —
    рамка кадра снимка: она отмечает, где находится «виртуальная фотография»,
    в той же плоскости и на том же расстоянии, что и облако.

    Время снимка берётся из имени файла (эпоха в нс/мс/с), поза камеры — из
    траектории и калибровки, той же цепочкой, что и в colorise_offline.

Использование
    python3 make_photo_plane.py data/1788882799423559018.png \
        --trajectory data/trajectory.csv \
        --calib data/calib-pinhole-2026-09-16.json --distance 5 --ppm 100

Результат
    <имя фото>-plane-<d>m-<ppm>ppm.pcd   облако (x y z intensity rgb)

Зависимости: numpy, opencv-python; загрузчики переиспользуются из
colorise_offline.py (импорт безопасен — там всё под if __name__).
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

import colorise_offline as co


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Облако точек параллельно плоскости фотографии "
                    "(фикстура для тестов раскраски).",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("photo", help="файл снимка; время съёмки — в его имени (эпоха)")
    ap.add_argument("--trajectory", default="trajectory.csv", help="CSV с позами лидара")
    ap.add_argument("--calib", default="calib.json",
                    help="интринсики камеры (блок \"camera\") + экстраинсик камера<->лидар")
    ap.add_argument("--distance", type=float, required=True,
                    help="расстояние от камеры до плоскости, м (по оптической оси)")
    ap.add_argument("--ppm", type=float, required=True, help="точек на квадратный метр")
    ap.add_argument("--margin", type=float, default=1.05,
                    help="запас за границы кадра, доли (1.0 = ровно по углам) [1.05]")
    ap.add_argument("--output", default="", help="куда писать облако [<имя фото>-plane-...pcd]")
    ap.add_argument("--chunk", type=int, default=1000,
                    help="сколько точек считать и писать за один заход [1000]")
    ap.add_argument("--time-tolerance", type=float, default=1.0,
                    help="с; допуск выхода времени снимка за траекторию [1.0]")
    ap.add_argument("--time-shift", type=float, default=0.0,
                    help="с; сдвиг времени траектории [0]")
    ap.add_argument("--extrinsic-name", default=None, help="ключ экстраинсика в calib.json")
    ap.add_argument("--extrinsic-direction", default=None,
                    choices=[None, "camera_from_lidar", "lidar_from_camera"])
    ap.add_argument("--euler-order", default="xyz", choices=["xyz", "zyx"])
    ap.add_argument("--euler-units", default="auto", choices=["auto", "deg", "rad"])
    return ap.parse_args(argv)


def pcd_header(n):
    """Заголовок бинарного PCD — тот же, что пишет colorise_offline.save_cloud_pcd."""
    return ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            "FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\nTYPE F F F F U\n"
            "COUNT 1 1 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n" % (n, n))


def pixel_rays(cam, pixels):
    """Направления (x/z, y/z) лучей через заданные пиксели, с учётом модели."""
    pts = np.asarray(pixels, np.float64).reshape(-1, 1, 2)
    if cam["model"] == "fisheye":
        out = cv2.fisheye.undistortPoints(pts, cam["K"], cam["dist"])
    else:
        out = cv2.undistortPoints(pts, cam["K"], cam["dist"])
    out = np.asarray(out[0] if isinstance(out, tuple) else out, np.float64)
    return out.reshape(-1, 2)


def image_corner_rays(cam):
    """Лучи через четыре угла кадра (по часовой: ЛВ, ПВ, ПН, ЛН)."""
    w, h = cam["width"], cam["height"]
    return pixel_rays(cam, [[0.0, 0.0], [w - 1.0, 0.0], [w - 1.0, h - 1.0], [0.0, h - 1.0]])


def photo_frame_points(cam, distance, per_side=25):
    """
    Точки по периметру кадра снимка: per_side на сторону (25 -> ровно 100).
    Пиксели равномерно раскладываются по границе изображения, затем их лучи
    пересекаются с плоскостью z=distance, поэтому проекция каждой точки лежит
    точно на границе кадра — рамка отмечает именно место фотографии.
    """
    w, h = cam["width"], cam["height"]
    n = per_side * 4
    corners = np.array([[0.0, 0.0], [w - 1.0, 0.0], [w - 1.0, h - 1.0], [0.0, h - 1.0]],
                       np.float64)
    pix = np.empty((n, 2), np.float64)
    for i in range(n):
        seg = (i / n) * 4.0
        k = int(seg) % 4
        f = seg - int(seg)
        pix[i] = corners[k] * (1.0 - f) + corners[(k + 1) % 4] * f
    rays = pixel_rays(cam, pix)
    pts = np.empty((n, 3), np.float64)
    pts[:, 0] = distance * rays[:, 0]
    pts[:, 1] = distance * rays[:, 1]
    pts[:, 2] = distance
    return pts


def plane_rect(cam, distance, margin):
    """
    Прямоугольник плоскости z=distance в кадре камеры: (cx, cy, hx, hy), метры.
    Границы берутся по реальным углам кадра (через модель дисторсии), затем
    растягиваются на margin, чтобы часть точек вышла за кадр.
    """
    rays = image_corner_rays(cam)
    xs = distance * rays[:, 0]
    ys = distance * rays[:, 1]
    if not (np.all(np.isfinite(xs)) and np.all(np.isfinite(ys))):
        raise SystemExit("не удалось построить границы плоскости: углы кадра не "
                         "пересекают z=distance (проверьте модель камеры и --distance)")
    cx = 0.5 * (xs.min() + xs.max())
    cy = 0.5 * (ys.min() + ys.max())
    hx = 0.5 * (xs.max() - xs.min()) * margin
    hy = 0.5 * (ys.max() - ys.min()) * margin
    return cx, cy, hx, hy


def camera_pose_at(traj, t, tol):
    """(T_world_lidar, t_used) на время t; T=None, если t вне траектории сверх tol."""
    if t < traj.min_t - tol or t > traj.max_t + tol:
        return None, t
    tc = min(max(t, traj.min_t), traj.max_t)
    return traj.lidar_pose_world(tc), tc


def main(argv=None):
    args = parse_args(argv)
    if args.distance <= 0.0:
        raise SystemExit("--distance должен быть > 0")
    if args.ppm <= 0.0:
        raise SystemExit("--ppm должен быть > 0")
    if not os.path.isfile(args.photo):
        raise SystemExit("нет такого файла: " + args.photo)

    stem = os.path.splitext(os.path.basename(args.photo))[0]
    t_photo = co.photo_timestamp_from_name(stem)
    if t_photo is None:
        raise SystemExit("в имени файла нет времени (эпоха в нс/мс/с): " + args.photo)

    cam, cam_key = co.load_camera_from_calib(args.calib)
    T_cam_lidar, calib_key, calib_dir = co.load_calib(
        args.calib, args.extrinsic_name, args.extrinsic_direction)
    traj = co.load_trajectory(args.trajectory, args.euler_order, args.euler_units,
                              args.time_shift)

    print("[photo] {}  t={:.6f}".format(os.path.basename(args.photo), t_photo))
    print("[camera] {} {}x{} (calib: {}) fx={:.3f} fy={:.3f} cx={:.3f} cy={:.3f} dist={}".format(
        cam["model"], cam["width"], cam["height"], cam_key,
        cam["K"][0, 0], cam["K"][1, 1], cam["K"][0, 2], cam["K"][1, 2],
        np.round(cam["dist"], 6).tolist()))
    print("[extrinsic] {} ({})".format(calib_key, calib_dir))
    print("[trajectory] {} poses t=[{:.3f}, {:.3f}]".format(
        traj.size(), traj.min_t, traj.max_t))

    Tw, t_used = camera_pose_at(traj, t_photo, args.time_tolerance)
    if Tw is None:
        raise SystemExit("время снимка вне траектории более чем на {:.2f} с: t={:.3f}, "
                         "траектория [{:.3f}, {:.3f}]".format(
                             args.time_tolerance, t_photo, traj.min_t, traj.max_t))
    if abs(t_used - t_photo) > 1e-9:
        print("[WARN] время снимка {:.3f} вне траектории — поза зажата к {:.3f}".format(
            t_photo, t_used))

    T_world_cam = Tw @ co.invert_T(T_cam_lidar)
    print("[pose] T_world_cam: p=({:.3f}, {:.3f}, {:.3f})\n  R=\n{}".format(
        T_world_cam[0, 3], T_world_cam[1, 3], T_world_cam[2, 3],
        np.round(T_world_cam[:3, :3], 4)))

    cx, cy, hx, hy = plane_rect(cam, args.distance, args.margin)
    step = 1.0 / math.sqrt(args.ppm)
    nx = max(2, int(round(2.0 * hx / step)) + 1)
    ny = max(2, int(round(2.0 * hy / step)) + 1)
    xs = cx + (np.arange(nx) - 0.5 * (nx - 1)) * step
    ys = cy + (np.arange(ny) - 0.5 * (ny - 1)) * step
    w, h = cam["width"], cam["height"]
    base = "{}-plane-{:g}m-{:g}ppm".format(stem, args.distance, args.ppm)
    out_path = args.output or (base + ".pcd")

    # Потоковая генерация: точки считаются порциями по ~--chunk штук и сразу
    # пишутся в файл, поэтому память не зависит от размера фикстуры.
    n_total = nx * ny
    rows_per_chunk = max(1, int(args.chunk) // nx)
    R_wc, o_wc = T_world_cam[:3, :3], T_world_cam[:3, 3]
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("intensity", "<f4"), ("rgb", "<u4")])
    # Рамка кадра снимка: 100 точек по периметру (25 на сторону) — та же
    # плоскость и то же место, где находится «виртуальная фотография».
    frame_cam = photo_frame_points(cam, args.distance)
    frame_xyz = (R_wc @ frame_cam.T).T + o_wc
    written = 0
    with open(out_path, "wb") as fc:
        fc.write(pcd_header(n_total + frame_cam.shape[0]).encode("ascii"))
        for r0 in range(0, ny, rows_per_chunk):
            yy = ys[r0:r0 + rows_per_chunk]
            gx, gy = np.meshgrid(xs, yy)
            p_cam = np.stack([gx.ravel(), gy.ravel(),
                              np.full(gx.size, args.distance)], axis=1)
            xyz = (R_wc @ p_cam.T).T + o_wc
            rec = np.empty(p_cam.shape[0], dt)
            rec["x"], rec["y"], rec["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            rec["intensity"] = 0.0
            rec["rgb"] = 0
            fc.write(rec.tobytes())
            written += p_cam.shape[0]
        rec = np.empty(frame_cam.shape[0], dt)
        rec["x"], rec["y"], rec["z"] = frame_xyz[:, 0], frame_xyz[:, 1], frame_xyz[:, 2]
        rec["intensity"] = 0.0
        rec["rgb"] = 0
        fc.write(rec.tobytes())
        written += frame_cam.shape[0]
    print("[cloud] {} точек -> {}".format(written, out_path))
    print("[photo-frame] {} точек по периметру кадра ({} на сторону), z = {:g} м".format(
        frame_cam.shape[0], frame_cam.shape[0] // 4, args.distance))

    vis_area = (args.distance * w / cam["K"][0, 0]) * (args.distance * h / cam["K"][1, 1])
    print("[plane] {:.2f} x {:.2f} м, шаг сетки {:.4f} м, сетка {}x{}".format(
        2 * hx, 2 * hy, step, nx, ny))
    print("[stats] точек {}  (в кадре ожидается ~{:.0f} при площади кадра {:.1f} м²)".format(
        written, vis_area * args.ppm, vis_area))
    if args.margin < 1.0:
        print("[WARN] --margin {} < 1.0: края кадра не покрыты точками".format(args.margin))
    if args.distance <= 2.0:
        print("[WARN] --distance {:g} м <= 2.0 м: при стандартном --min-camera-dist 2 м "
              "эти точки не будут раскрашены".format(args.distance))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())