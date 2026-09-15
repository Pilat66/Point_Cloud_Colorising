#!/usr/bin/env python3
"""
Тестовое облако, параллельное плоскости фотографии.

Назначение
    Создать детерминированную фикстуру для проверки алгоритмов раскраски:
    облако точек в плоскости, параллельной плоскости изображения выбранного
    снимка, на заданном расстоянии от камеры. Скрипт НИЧЕГО не раскрашивает —
    он готовит только вход (и, при желании, эталонный список ожидаемых
    пикселей и цветов).

    Одно и то же облако прогоняется через разные алгоритмы
    (colorise_offline C++ и Python, ColoriseMap, будущие оптимизированные
    версии); результаты сравниваются между собой и с эталоном.

    Время снимка берётся из имени файла (эпоха в нс/мс/с), поза камеры — из
    траектории и калибровки, той же цепочкой, что и в colorise_offline.

Использование
    python3 make_photo_plane.py data/1788882799423559018.png \
        --trajectory data/trajectory.csv --camera data/camera.yaml \
        --calib data/calib.json --distance 5 --ppm 100

Результат
    <имя фото>-plane-<d>m-<ppm>ppm.pcd            облако (x y z intensity rgb)
    <имя фото>-plane-<d>m-<ppm>ppm-expected.csv   эталон:
        index;x;y;z;u;v;inside;R;G;B

Зависимости: numpy, opencv-python, pyyaml; загрузчики переиспользуются из
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
    ap.add_argument("--camera", default="camera.yaml", help="интринсики камеры")
    ap.add_argument("--calib", default="calib.json", help="экстраинсики камера<->лидар")
    ap.add_argument("--distance", type=float, required=True,
                    help="расстояние от камеры до плоскости, м (по оптической оси)")
    ap.add_argument("--ppm", type=float, required=True, help="точек на квадратный метр")
    ap.add_argument("--margin", type=float, default=1.05,
                    help="запас за границы кадра, доли (1.0 = ровно по углам) [1.05]")
    ap.add_argument("--output", default="", help="куда писать облако [<имя фото>-plane-...pcd]")
    ap.add_argument("--expected-csv", default="",
                    help="куда писать эталон [<облако без .pcd>-expected.csv]")
    ap.add_argument("--no-expected", action="store_true", help="не писать эталонный CSV")
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


def image_corner_rays(cam):
    """Направления (x/z, y/z) лучей через четыре угла кадра, с учётом модели."""
    w, h = cam["width"], cam["height"]
    pts = np.array([[0.0, 0.0], [w - 1.0, 0.0], [w - 1.0, h - 1.0], [0.0, h - 1.0]],
                   np.float64).reshape(-1, 1, 2)
    if cam["model"] == "fisheye":
        out = cv2.fisheye.undistortPoints(pts, cam["K"], cam["dist"])
    else:
        out = cv2.undistortPoints(pts, cam["K"], cam["dist"])
    out = np.asarray(out[0] if isinstance(out, tuple) else out, np.float64)
    return out.reshape(-1, 2)


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

    cam = co.load_camera(args.camera)
    T_cam_lidar, calib_key, calib_dir = co.load_calib(
        args.calib, args.extrinsic_name, args.extrinsic_direction)
    traj = co.load_trajectory(args.trajectory, args.euler_order, args.euler_units,
                              args.time_shift)

    print("[photo] {}  t={:.6f}".format(os.path.basename(args.photo), t_photo))
    print("[camera] {} {}x{} fx={:.3f} fy={:.3f} cx={:.3f} cy={:.3f} dist={}".format(
        cam["model"], cam["width"], cam["height"],
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
    gx, gy = np.meshgrid(xs, ys)
    p_cam = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, args.distance)], axis=1)

    rvec = np.zeros((3, 1), np.float64)
    tvec = np.zeros((3, 1), np.float64)
    if cam["model"] == "fisheye":
        res = cv2.fisheye.projectPoints(p_cam.reshape(-1, 1, 3), rvec, tvec,
                                        cam["K"], cam["dist"])
    else:
        res = cv2.projectPoints(p_cam.reshape(-1, 1, 3), rvec, tvec,
                                cam["K"], cam["dist"])
    uv = np.asarray(res[0] if isinstance(res, tuple) else res, np.float64).reshape(-1, 2)
    u = np.round(uv[:, 0]).astype(np.int64)
    v = np.round(uv[:, 1]).astype(np.int64)
    w, h = cam["width"], cam["height"]
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)

    xyz = (T_world_cam[:3, :3] @ p_cam.T).T + T_world_cam[:3, 3]

    img = cv2.imread(args.photo, cv2.IMREAD_COLOR)
    if img is None:
        raise SystemExit("не удалось прочитать изображение: " + args.photo)
    rgb = np.zeros((u.size, 3), np.int64)
    if inside.any():
        rgb[inside] = img[v[inside], u[inside]][:, ::-1]      # BGR -> RGB

    base = "{}-plane-{:g}m-{:g}ppm".format(stem, args.distance, args.ppm)
    out_path = args.output or (base + ".pcd")
    exp_path = args.expected_csv or (os.path.splitext(out_path)[0] + "-expected.csv")

    co.save_cloud_pcd(out_path, xyz, np.zeros(xyz.shape[0], np.uint32),
                      np.zeros(xyz.shape[0], np.float32))
    print("[cloud] {} точек -> {}".format(xyz.shape[0], out_path))

    if not args.no_expected:
        with open(exp_path, "w") as f:
            f.write("index;x;y;z;u;v;inside;R;G;B\n")
            for i in range(u.size):
                f.write("{};{:.6f};{:.6f};{:.6f};{:.3f};{:.3f};{};{};{};{}\n".format(
                    i, xyz[i, 0], xyz[i, 1], xyz[i, 2], uv[i, 0], uv[i, 1],
                    int(inside[i]), rgb[i, 0], rgb[i, 1], rgb[i, 2]))
        print("[expected] {} строк -> {}".format(u.size, exp_path))

    vis_area = (args.distance * w / cam["K"][0, 0]) * (args.distance * h / cam["K"][1, 1])
    print("[plane] {:.2f} x {:.2f} м, шаг сетки {:.4f} м, сетка {}x{}".format(
        2 * hx, 2 * hy, step, nx, ny))
    print("[stats] точек {}  внутри кадра {}  (оценка площади кадра {:.1f} м²: ~{:.0f} точек)".format(
        xyz.shape[0], int(inside.sum()), vis_area, vis_area * args.ppm))
    if args.margin < 1.0:
        print("[WARN] --margin {} < 1.0: края кадра не покрыты точками".format(args.margin))
    if args.distance <= 2.0:
        print("[WARN] --distance {:g} м <= 2.0 м: при стандартном --min-camera-dist 2 м "
              "эти точки не будут раскрашены".format(args.distance))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())