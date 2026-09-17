# colorise_map — офлайн-раскраска готовой карты (без ROS)

ROS-free C++ приложение, повторяющее логику узла `ColoriseMap`
(`src/ColoriseMap.cpp`): красит **готовое облако карты** (PCD/LAS в world-кадре)
по фотографиям и CSV-траектории лидара.

Карта статична, поэтому от кадра к кадру меняется только положение камеры:
поза лидара интерполируется на **время самого снимка**, облако отсекается по
радиусу вокруг камеры, цвет накапливается по точкам карты и усредняется, а в
конце результат сохраняется. Это тот же конвейер, что узел выполняет на живом
bag-е, только без ROS: снимки берутся из каталогов (время в имени файла), позы
из CSV, калибровка из файлов.

Проекционное ядро **не дублируется**: цель cmake компилирует уже проверенные
исходники порта `colorise/` (`io.cpp`, `trajectory.cpp`, `project.cpp`), поэтому
гейты, проекция (pinhole/fisheye), z-буфер окклюзии и упаковка `rgb` — тот же
код, что в `colorise_offline` и в Python-версии.

---

## 1. Зависимости

Только системные пакеты — те же, что у `colorise/`:

| Библиотека | Для чего |
|---|---|
| Eigen3 ≥ 3.3 | 4×4-трансформы, кватернионы, SLERP |
| OpenCV ≥ 4 | чтение фото, `cv::projectPoints` / `cv::fisheye::projectPoints` |
| nlohmann/json | vendored single-header в `../colorise/third_party/` |

```bash
sudo apt install libeigen3-dev libopencv-dev
```

## 2. Сборка

```bash
cmake -S ColoriseMap -B ColoriseMap/build -DCMAKE_BUILD_TYPE=Release
cmake --build ColoriseMap/build -j4
```

Получается бинарник `ColoriseMap/build/colorise_map` (каталог `build/`
исключён `.gitignore`).

## 3. Быстрый старт

Одна камера — ключи как у `colorise_offline`:

```bash
./ColoriseMap/build/colorise_map \
    --cloud data/all_raw_points.pcd \
    --trajectory data/trajectory.csv \
    --photos data --calib data/calib.json \
    --output coloured_map.pcd
```

Все пути по умолчанию совпадают с `colorise_offline`, поэтому команда работает
как есть: `./ColoriseMap/build/colorise_map --output coloured_map.pcd`.

Несколько камер: **каждый `--photos` открывает новую камеру**, а следующие за ним
`--calib` / `--name` относятся к ней. Пара повторяется на каждую
камеру:

```bash
./ColoriseMap/build/colorise_map --cloud map.pcd --trajectory traj.csv \
    --photos dirRight --calib right.json --name right \
    --photos dirLeft  --calib left.json  --name left \
    --output coloured_map.pcd
```

Кадры разных камер подбираются по времени: ведущей считается первая камера, для
остальных берётся ближайший снимок в пределах `--max-time-offset`.

---

## 4. Входные данные

| Аргумент | По умолчанию | Что это |
|---|---|---|
| `--cloud` | `data/all_raw_points.pcd` | карта: бинарный `.pcd` или `.las`, в world/map-кадре |
| `--trajectory` | `data/trajectory.csv` | позы **лидара** в том же кадре |
| `--output` | `coloured_map.pcd` | окрашенная карта: `.pcd` (по умолчанию бинарный `x y z intensity rgb`) или `.las` |
| `--photos` | — | каталог снимков, время съёмки в имени файла |
| `--calib` | — | интринсики камеры (блок `camera`) + экстраинсик камера↔лидар (`calib.json`) |

Форматы фото, траектории, камеры и экстраинсиков — ровно те же, что у
`colorise_offline` (см. `colorise_offline_README.md` §3 и `colorise/README.md` §4):
время в имени файла (эпоха в нс/мс/с, определяется автоматически), траектория
`time;x;y;z;roll;pitch;yaw` или `time,x,y,z,qx,qy,qz,qw`, направление
экстраинсиков по схеме `A_from_B`.

Данные этого репозитория, на которых всё проверено:

| Файл | Что |
|---|---|
| `data/all_raw_points.pcd` | карта (Livox MID360, world-кадр), 391 МБ |
| `data/trajectory.csv` | траектория ~200 Гц |
| `data/calib.json` | блок `camera` (интринсики, 1600×1300, pinhole) + экстраинсик камеры `right`; образец формата — `data/calib-pinhole-2026-09-16.json` |
| `data/*.png` | 82 кадра по 24 МБ облака (быстрый вариант проверки) |

---

## 5. Флаги

| Флаг | По умолчанию | Описание |
|---|---|---|
| `--map-max-range` | `20.0` | радиус отсечения вокруг камеры, м (`0` — отключить) |
| `--min-color-frames` | `1` | минимум кадров, увидевших точку (в `configs/config.yaml` узла для карты стоит `2`) |
| `--compensation` | `odom` | `odom` — интерполяция траектории на время кадра; `none` — ближайший отсчёт (аналог `CompMode::NONE`) |
| `--max-time-offset` | `0.05` | допуск подбора пар между камерами, с |
| `--occlusion` / `--no-occlusion` | вкл. | z-буфер окклюзии |
| `--occlusion-cell` | `4.0` | размер ячейки z-буфера, px |
| `--occlusion-depth-tol` | `0.3` | допустимый разброс глубины в ячейке, м |
| `--keep-uncolored` | выкл. | сохранить неокрашенные точки чёрными |
| `--nearest-wins` | выкл. | цвет точки — только от **ближайшей** камеры: среди кадров, где точка видима, выбирается наблюдение с минимальной дистанцией камера→точка (быстрое приближение sqrt, ошибка <0.2%); сравнение строгим `<` (не зависит от порядка кадров/потоков); `--min-color-frames` остаётся порогом на число наблюдений |
| `--extrinsic-name` | — | ключ экстраинсика в `calib.json` |
| `--extrinsic-direction` | авто | `camera_from_lidar` \| `lidar_from_camera` |
| `--euler-order` | `xyz` | порядок углов Эйлера в траектории |
| `--euler-units` | `auto` | `auto` \| `deg` \| `rad` |
| `--time-shift` | `0` | сдвиг времени траектории, с (экв. обратному сдвигу фото) |
| `--time-tolerance` | `1.0` | с: снимок вне диапазона траектории зажимается к концу, если не дальше этого |
| `--min-camera-dist` | `2.0` | ближний гейт, м |
| `--max-view-angle` | `75.0` | полуугол обзора от оптической оси, град. |
| `--edge-margin` | `0.0` | обрезка краёв изображения, px |
| `--jobs` | `0` | рабочих потоков (0 = все ядра) |
| `-h`, `--help` | — | справка |

Значения по умолчанию специально совпадают с `colorise_offline` — это даёт
проверку паритета при одной камере теми же командами.

---

## 6. Как это работает

1. Загружается карта, отбрасываются нефинитные точки, строится воксельная сетка
   (`VoxelGrid`) — замена `pcl::KdTreeFLANN` из узла, семантика радиуса `≤ r`.
2. Загружается траектория (`--time-shift`, `--euler-*`, автоопределение
   единиц/разделителя).
3. Для каждой камеры читаются интринсики, экстраинсик и список снимков
   (время из имени файла).
4. Кадры спариваются: ведущая камера — первая, остальные — ближайшим снимком в
   пределах `--max-time-offset`; неспаренные кадры отбрасываются (в конце
   печатается их число).
5. Для каждого кадра и каждой камеры: поза на время **этого** снимка →
   `T_cam_from_world = (T_world_lidar · T_lidar_cam)⁻¹` → радиусный отсев вокруг
   центра камеры → `cv::imread` → `projectAndSample` (гейты, проекция, z-буфер) →
   накопление сумм и счётчиков по индексам карты.
6. Кадры обрабатываются параллельно (`--jobs`), но фиксируются строго в порядке
   кадров — суммы и лог детерминированы (проверено: `--jobs 1` и `--jobs N` дают
   побайтово одинаковый выход).
7. Финал: точка сохраняется, если её увидели `≥ --min_color_frames` кадров;
   цвет — среднее наблюдений с округлением half-to-even (`std::nearbyint`);
   `--keep-uncolored` пишет остальные чёрными; результат сохраняется в конце
   прогона (в узле это делается по Ctrl-C).

### Соответствие узлу `src/ColoriseMap.cpp`

| Узел | Здесь |
|---|---|
| `loadMap()` (PCD XYZI → XYZ + intensity) | `loadCloud()` + `VoxelGrid` |
| `OdomTrajectory::loadCsv` | `loadTrajectory()` |
| `poseAt()` (`NONE`, `ODOM`) | `--compensation none` / `odom` |
| `kdtree_.radiusSearch(..., map_max_range)` | `VoxelGrid::radiusQuery` |
| `cv::imdecode(CompressedImage)` | `cv::imread` + `listPhotos()` |
| `projectAndSample()` | тот же код из `../colorise/src/project.cpp` |
| `colour_sum_/colour_cnt_`, `min_color_frames` | те же суммы/счётчики и порог |
| `keep_uncolored_points` | `--keep-uncolored` |
| `occlusion_check/_cell_px/_depth_tol` | те же ключи/флаги |
| `findClosest` + `max_time_offset` | подбор пар между камерами |
| `saveFinalMap()` (PCD через PCL) | `saveCloud()` (PCD `x y z intensity rgb` или LAS 1.2 fmt 3) |
| publish, timer, RViz, фрустумы, backlog-drop | нет (ROS-only) |

---

## 7. Отличия от узла

- **IMU-компенсации нет.** `CompMode::IMU` в узле делает SLERP по полю
  `orientation` из `sensor_msgs/Imu`, т.е. ему нужен AHRS. В данных этого
  репозитория такого нет: у `/livox/imu` в bag-файле
  (`2026-09-08-18-56-45_0.mcap`) кватернион всегда единичный
  (`orientation = (0,0,0,1)`, `orientation_covariance[0] = 0`), сырые только
  `angular_velocity`/`linear_acceleration`. Порт этого режима «как есть» выродился
  бы в тождественное преобразование. Поэтому `--compensation` принимает только
  `none | odom`; на этих данных интерполяция траектории и так точнее: CSV прогона
  идёт с шагом 0.005 с (200 Гц), а кадры — 5 Гц, т.е. поза на время кадра
  известна практически точно.
- **Сохранение в конце прогона**, а не по Ctrl-C (`--save-on-signal` возможен как
  отдельная задача).
- **Усреднение**: `std::nearbyint` по сумме double (как в `colorise_offline` и в
  Python-версии). Узел делит целочисленно (`colour_sum_ / colour_cnt_`), что
  может отличаться на 1 единицу на канал; здесь выбран вариант офлайн-ядра, чтобы
  сохранялся побайтовый паритет с `colorise_offline`.
- **`--min-color-frames` по умолчанию 1** (как в `colorise_offline`); в
  `configs/config.yaml` узла для карты стоит 2.
- Лог дополнен строками `[map]`, `[frames]` и `[photos] <имя>`; строки `[frame N]`
  и блок `==== RESULT ====` идентичны `colorise_offline`.
- Нет `map_publish_every_n` (публикация в RViz — ROS-only).

---

## 8. Проверено

Прогон на данных репозитория (`1788882799423559018.pcd`, 1 201 612 точек,
82 кадра, `data/`), одинаковые параметры у обоих инструментов:

```bash
./colorise/build/colorise_offline --cloud 1788882799423559018.pcd --photos data \
    --trajectory data/trajectory.csv --calib data/calib.json \
    --output /tmp/a.pcd
./ColoriseMap/build/colorise_map --cloud 1788882799423559018.pcd --photos data \
    --trajectory data/trajectory.csv --calib data/calib.json \
    --output /tmp/b.pcd
cmp /tmp/a.pcd /tmp/b.pcd          # побайтовое совпадение
```

| Проверка | Результат |
|---|---|
| PCD против `colorise_offline` (одна камера, те же флаги) | побайтовое совпадение |
| Строки `[frame N]` лога | совпадают все 82 |
| Блок `==== RESULT ====` | `total=1,201,612 coloured=1,196,386 (99.57%)` в обоих |
| Две камеры с одинаковыми фото | спарено 82 кадра, PCD побайтово равен однокамерному |
| `--jobs 1` против `--jobs` (авто) | побайтовое совпадение (детерминизм) |
| `--min-color-frames 2` | `coloured=1,189,870 (99.02%)` — меньше, как и ожидается |
| `--output out.las` | LAS 1.2, point format 3, 1 196 386 точек, RGB ×257, intensity сохранена |
| Негативные CLI-кейсы (нет `--calib` в группе камеры, нет камер, `--compensation imu`, старый флаг `--camera`) | внятная ошибка, exit 1 |
| Камера читается из `data/calib.json` (`camera` блок) вместо `camera.yaml` | PCD побайтово совпал с прогоном до перехода: `md5 9b03201e…`, `coloured=618 290` (набор `data/1789476925898092722.las` + `data/img-1789476925898092722/`) |

---

## 9. Типичные проблемы

| Симптом | Причина / решение |
|---|---|
| `coloured=0`, покрытие 0 % | Карта, траектория и фото из разных прогонов (сверить диапазоны `[trajectory]` и `[photos]`), либо `--min-color-frames` выше числа кадров (в этом случае печатается WARN) |
| «no points within … m» в каждом кадре | `--map-max-range` мал или карта и позы в разных кадрах (проверить `--extrinsic-direction`, знак `--time-shift`) |
| Цвет «плывёт» по краям | Сдвиг времени камеры относительно лидара — подобрать `--time-shift` (для набора `2026-09-08-18-56-45` конфиг прогона `mid360-long-city.yaml` задаёт `img_time_offset: 0.1`, т.е. ~0.1 с); также помогает уменьшить `--max-view-angle` и увеличить `--min-camera-dist` |
| Кадров меньше, чем снимков | Какая-то камера не имеет пары в пределах `--max-time-offset` — увеличить допуск или синхронизировать каталоги фото |
| Дырки на плоскостях, мало окрашенных точек в кадре | Уменьшить `--occlusion-cell` (4 → 2/1) или увеличить `--occlusion-depth-tol` |
| Цвет «протекает» через контуры | Увеличить `--occlusion-cell` (4 → 8/16) или уменьшить `--occlusion-depth-tol` |
| `--name must follow --photos` | Порядок аргументов: `--name` (как и `--calib`) относится к последней открытой группе `--photos` |
| Ошибка `no camera intrinsics block in …` или `has no image size` | В `--calib` нет блока `camera` либо в нём нет `width`/`height`: добавить их (образец — `data/calib-pinhole-2026-09-16.json`) |
| Ошибка `unknown option: '--camera'` | Флаг удалён: интринсики берутся из `--calib` (блок `camera`), `camera.yaml` больше не читается |

> Примечание про время: в отличие от узла, здесь поза берётся на **время самого
> снимка**, поэтому расхождение часов камеры и лидара лечится именно
> `--time-shift`; IMU для этого не нужен (см. §7).



