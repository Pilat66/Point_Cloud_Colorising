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
    рамка кадра снимка: она отмечает, где находится «виртуальная фотография».
    Рамка строится в плоскости, параллельной изображению, на --frame-distance
    от камеры (по умолчанию 1 м) — между камерой и облаком, отдельно от
    фикстуры.

    Время снимка берётся из имени файла (эпоха в нс/мс/с), поза камеры — из
    траектории и калибровки, той же цепочкой, что и в colorise_offline.

Наклон плоскости (--tilt-x, --tilt-y)
    Плоскость можно наклонить: углы задаются вокруг осей X и Y системы камеры
    (в градусах, правило правой руки), порядок поворотов R = Ry(--tilt-y) · Rx(
    --tilt-x). Точка привязки не меняется — плоскость проходит через прежний
    центр (cx, cy, distance) на оптической оси, наклоняется только её нормаль.
    Границы сетки пересчитываются так, чтобы наклонённая плоскость по-прежнему
    покрывала весь кадр: лучи через четыре угла изображения пересекаются с
    наклонённой плоскостью, и по ним берутся границы в собственных координатах
    плоскости (--margin действует как раньше — растягивает прямоугольник).

    При больших углах кадр «растягивается» по плоскости: её площадь и число
    точек растут как 1/cos угла, а дальние углы уезжают дальше по расстоянию
    (печатается в [plane-tilt], с предупреждением при выходе за --max-range
    раскраски, по умолчанию 20 м).

Использование
    python3 make_photo_plane.py data/1788882799423559018.png \
        --trajectory data/trajectory.csv \
        --calib data/calib-pinhole-2026-09-16.json --distance 5 --ppm 100

    # наклон: 30° вокруг X и 20° вокруг Y (оси камеры)
    python3 make_photo_plane.py data/1788882799423559018.png \
        --trajectory data/trajectory.csv \
        --calib data/calib.json --distance 5 --ppm 100 --tilt-x 30 --tilt-y 20

Результат
    <имя фото>-plane-<d>m-<ppm>ppm.pcd   облако (x y z intensity rgb)
    <имя фото>-plane-<d>m-<ppm>ppm-tilt<X>x<Y>deg.pcd   то же с наклоном

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


# Потолок на размер сетки: защита от около-касательных наклонов, где площадь
# плоскости растёт как 1/cos угла и число точек уходит в бесконечность.
MAX_GRID_POINTS = 100_000_000


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
    ap.add_argument("--tilt-x", type=float, default=0.0,
                    help="наклон плоскости вокруг оси X камеры, градусы [0]")
    ap.add_argument("--tilt-y", type=float, default=0.0,
                    help="наклон плоскости вокруг оси Y камеры, градусы [0]")
    ap.add_argument("--margin", type=float, default=1.05,
                    help="запас за границы кадра, доли (1.0 = ровно по углам) [1.05]")
    ap.add_argument("--frame-distance", type=float, default=1.0,
                    help="м; расстояние от камеры до рамки кадра (0 < d < --distance) [1.0]")
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


def photo_frame_points(cam, frame_distance, per_side=25):
    """
    Точки по периметру кадра снимка: per_side на сторону (25 -> ровно 100).
    Пиксели равномерно раскладываются по границе изображения, затем их лучи
    пересекаются с плоскостью z=frame_distance — так рамка лежит там, где
    находится реальная фотография (ближе к камере), отдельно от облака.
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
    pts[:, 0] = frame_distance * rays[:, 0]
    pts[:, 1] = frame_distance * rays[:, 1]
    pts[:, 2] = frame_distance
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


def tilt_rotation(tilt_x_deg, tilt_y_deg):
    """
    Матрица наклона плоскости в системе камеры: R = Ry(tilt_y) · Rx(tilt_x).
    Углы в градусах, правило правой руки. Столбцы — оси плоскости в кадре
    камеры: e1, e2 лежат в плоскости, третий столбец — её нормаль.
    """
    tx = math.radians(tilt_x_deg)
    ty = math.radians(tilt_y_deg)
    cx_, sx = math.cos(tx), math.sin(tx)
    cy_, sy = math.cos(ty), math.sin(ty)
    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx_, -sx], [0.0, sx, cx_]])
    rot_y = np.array([[cy_, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy_]])
    return rot_y @ rot_x


def tilted_plane_frame(cam, distance, margin, rot_tilt):
    """
    Наклонённая плоскость, проходящая через прежний центр (cx, cy, distance).

    Возвращает (pivot, e1, e2, s_mid, t_mid, hs, ht, corner_dist, quad_area):
    pivot — точка привязки, e1/e2 — орты плоскости, s_mid/t_mid и hs/ht — центр и
    полуразмеры сетки в этих координатах, corner_dist — расстояния до пересечений
    лучей через углы кадра с плоскостью, quad_area — площадь четырёхугольника,
    который кадр занимает на плоскости.

    Границы берутся именно по этим пересечениям, поэтому наклонённая плоскость
    покрывает кадр так же, как перпендикулярная при нулевом наклоне, а --margin
    растягивает прямоугольник в собственных координатах плоскости. При наклоне
    пятно кадра на плоскости — четырёхугольник, а сетка прямоугольная: её площадь
    (и число точек) больше пятна, см. [stats].
    """
    px, py, _, _ = plane_rect(cam, distance, margin)
    pivot = np.array([px, py, distance], np.float64)
    e1, e2, n = rot_tilt[:, 0], rot_tilt[:, 1], rot_tilt[:, 2]

    rays = image_corner_rays(cam)
    dirs = np.column_stack([rays[:, 0], rays[:, 1], np.ones(rays.shape[0])])
    nd = dirs @ n                                   # n·d по каждому лучу
    lam = float(n @ pivot) / nd                     # параметр вдоль луча
    if not np.all(np.isfinite(lam)) or np.any(np.abs(nd) < 1e-9) or np.any(lam <= 0.0):
        raise SystemExit(
            "наклон слишком велик для этого кадра: лучи через его углы не "
            "пересекают плоскость перед камерой (уменьшите --tilt-x/--tilt-y)")

    hits = dirs * lam[:, None]
    rel = hits - pivot
    s, t = rel @ e1, rel @ e2
    s_mid = 0.5 * (s.min() + s.max())
    t_mid = 0.5 * (t.min() + t.max())
    hs = 0.5 * (s.max() - s.min()) * margin
    ht = 0.5 * (t.max() - t.min()) * margin
    quad_area = 0.5 * abs(float(np.dot(s, np.roll(t, -1)) - np.dot(t, np.roll(s, -1))))
    return (pivot, e1, e2, s_mid, t_mid, hs, ht,
            np.linalg.norm(hits, axis=1), quad_area)


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
    if not (0.0 < args.frame_distance < args.distance):
        raise SystemExit("--frame-distance {} м должен быть в диапазоне "
                         "(0, --distance {:g}): рамка лежит между камерой и "
                         "облаком".format(args.frame_distance, args.distance))
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

    if abs(args.tilt_x) >= 90.0 or abs(args.tilt_y) >= 90.0:
        raise SystemExit("--tilt-x/--tilt-y должны быть по модулю < 90°: "
                         "наклоняется нормаль плоскости, а не камера")
    tilted = abs(args.tilt_x) > 0.0 or abs(args.tilt_y) > 0.0

    if tilted:
        rot_tilt = tilt_rotation(args.tilt_x, args.tilt_y)
        pivot, e1, e2, mid_s, mid_t, hx, hy, corner_dist, quad_area = tilted_plane_frame(
            cam, args.distance, args.margin, rot_tilt)
        print("[tilt] Rx={:g}° Ry={:g}° (R = Ry·Rx), нормаль плоскости в кадре "
              "камеры n=({:.4f}, {:.4f}, {:.4f})".format(
                  args.tilt_x, args.tilt_y,
                  rot_tilt[0, 2], rot_tilt[1, 2], rot_tilt[2, 2]))
        print("[plane-tilt] углы кадра на плоскости: {:.2f}..{:.2f} м по расстоянию"
              .format(corner_dist.min(), corner_dist.max()))
        if corner_dist.max() > 20.0:
            print("[WARN] дальний угол плоскости на {:.1f} м: при стандартном "
                  "--max-range 20 м в colorise_offline эти точки не будут раскрашены"
                  .format(corner_dist.max()))
    else:
        mid_s, mid_t, hx, hy = plane_rect(cam, args.distance, args.margin)

    step = 1.0 / math.sqrt(args.ppm)
    nx = max(2, int(round(2.0 * hx / step)) + 1)
    ny = max(2, int(round(2.0 * hy / step)) + 1)
    if nx * ny > MAX_GRID_POINTS:
        raise SystemExit("сетка {}x{} = {:,} точек, лимит {:,}: уменьшите --ppm "
                         "или наклон".format(nx, ny, nx * ny, MAX_GRID_POINTS))
    # Без наклона xs/ys — координаты в кадре камеры (как было); с наклоном — в
    # базисе плоскости (e1, e2) относительно точки привязки pivot.
    xs = mid_s + (np.arange(nx) - 0.5 * (nx - 1)) * step
    ys = mid_t + (np.arange(ny) - 0.5 * (ny - 1)) * step
    w, h = cam["width"], cam["height"]
    base = "{}-plane-{:g}m-{:g}ppm".format(stem, args.distance, args.ppm)
    if tilted:
        base += "-tilt{:g}x{:g}deg".format(args.tilt_x, args.tilt_y)
    out_path = args.output or (base + ".pcd")

    # Потоковая генерация: точки считаются порциями по ~--chunk штук и сразу
    # пишутся в файл, поэтому память не зависит от размера фикстуры.
    n_total = nx * ny
    rows_per_chunk = max(1, int(args.chunk) // nx)
    R_wc, o_wc = T_world_cam[:3, :3], T_world_cam[:3, 3]
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("intensity", "<f4"), ("rgb", "<u4")])
    # Рамка кадра снимка: 100 точек по периметру (25 на сторону) — в плоскости
    # у камеры на --frame-distance, где находится реальная фотография; облако —
    # дальше, на --distance.
    frame_cam = photo_frame_points(cam, args.frame_distance)
    frame_xyz = (R_wc @ frame_cam.T).T + o_wc
    written = 0
    with open(out_path, "wb") as fc:
        fc.write(pcd_header(n_total + frame_cam.shape[0]).encode("ascii"))
        for r0 in range(0, ny, rows_per_chunk):
            yy = ys[r0:r0 + rows_per_chunk]
            gx, gy = np.meshgrid(xs, yy)
            if tilted:
                p_cam = pivot + np.outer(gx.ravel(), e1) + np.outer(gy.ravel(), e2)
            else:
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
    print("[photo-frame] {} точек по периметру кадра ({} на сторону), z = {:g} м "
          "(облако на {:g} м, масштаб {:g}x)".format(
              frame_cam.shape[0], frame_cam.shape[0] // 4, args.frame_distance,
              args.distance, args.frame_distance / args.distance))
    if args.frame_distance < 2.0:
        print("[NOTE] рамка на {:.3g} м от камеры: при стандартном --min-camera-dist 2 м "
              "в colorise_offline эти точки не будут раскрашены (рамка не обязана "
              "окрашиваться; поднять можно --frame-distance)".format(args.frame_distance))

    vis_area = (args.distance * w / cam["K"][0, 0]) * (args.distance * h / cam["K"][1, 1])
    print("[plane] {:.2f} x {:.2f} м, шаг сетки {:.4f} м, сетка {}x{}".format(
        2 * hx, 2 * hy, step, nx, ny))
    if tilted:
        print("[stats] точек {}  (пятно кадра на плоскости {:.1f} м² -> в кадре ~{:.0f} "
              "точек; площадь сетки {:.1f} м² — прямоугольник вокруг пятна, {:.1f}x "
              "больше)".format(written, quad_area, quad_area * args.ppm, 4.0 * hx * hy,
                               (4.0 * hx * hy) / quad_area))
    else:
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