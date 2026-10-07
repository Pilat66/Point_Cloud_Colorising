# Экспорт: кубовая раскраска карты точек (`ColoriseMap --cube`)

Самодостаточный набор для переноса **кубового режима раскраски готовой карты**
в другой проект. ROS не нужен, внешние зависимости — только Eigen3, OpenCV 4 и
потоки (`Threads`). Поиск соседей, чтение LAS/PCD и запись LAS реализованы в
самом наборе, без PCL, scipy и laspy.

Источник: репозиторий `Point_Cloud_Colorising`, коммит **f6a9202**.
C++-исходники скопированы **побайтово** (см. `MANIFEST.md` — там же sha256 всех
файлов). Отличается только `src/CMakeLists.txt` (пути сборки под раскладку
экспорта) — код не менялся ни на строку.

## Что делает режим

Карта (LAS с временем каждой точки, `gps_time`) делится на пространственные кубы
(по умолчанию 1 м). Для каждого снимка берутся кубы в его конусе внимания, а
кадр-победитель для точки выбирается по **её собственному времени** и расстоянию
до камеры:

```
score = w_t · |t_photo − t_point| / T_ref + w_d · min(1, d / D_ref)
```

Цвет берётся от кадра с минимальным баллом, без усреднения по кадрам. Выход —
LAS 1.4 point format 7 (RGB + `gps_time`), координаты/intensity/время
переносятся из входа без изменений.

Зачем отдельный режим: кадровый конвейер держит в памяти массивы размером с кадр
и на карте 43–56 млн точек требует ~20 ГБ RSS, кубовый укладывается в ~2 ГБ и
работает в 3.3 раза быстрее (замеры — `docs/Полезные наблюдения.md`).

## Состав

```
README.md          этот файл: сборка, запуск, ограничения
ALGORITHM.md       разбор конвейера по этапам с формулами и ссылками файл:строка
DATA_FORMATS.md    форматы входа (LAS, trajectory.csv, calib.json, фото) и выхода
INTEGRATION.md     встраивание в другой проект (пример: FAST-LIVO2-LAS)
MANIFEST.md        коммит-источник и sha256 всех файлов
src/               C++ исходники + CMakeLists.txt (цель colorise_map)
docs/              оригинальные README и память-банк проекта-источника
examples/          мини-набор данных, генератор и эталонные выходы
tests/smoke.sh     автоматическая проверка на примерах
```

Кодовая раскладка:

| Файл | Роль |
|---|---|
| `src/ColoriseMap/cube_colourise.{hpp,cpp}` | сам кубовый режим (`runCubeColourise`) |
| `src/ColoriseMap/main.cpp` | CLI и диспатч: `--cube` включает режим, иначе кадровый конвейер |
| `src/colorise/src/project.cpp` | проекционное ядро: гейты, pinhole/fisheye, z-буфер, упаковка RGB |
| `src/colorise/src/io.cpp` | потоковое чтение LAS, чтение PCD, писатель LAS 1.4 fmt 7, `calib.json`, список фото |
| `src/colorise/src/trajectory.cpp` | CSV поз, SLERP-интерполяция, 4×4-трансформы |
| `src/colorise/include/{voxel_grid.hpp,parallel.h}` | радиусный поиск (для кадрового режима) и parallel-for |

## Сборка

```bash
sudo apt install libeigen3-dev libopencv-dev     # зависимости
cmake -S src -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
# -> build/colorise_map
```

`nlohmann/json` уже внутри (`src/colorise/third_party/nlohmann/json.hpp`),
системный не требуется.

## Быстрый старт на примерах

```bash
./build/colorise_map --cube 1.0 \
    --cloud      examples/map.las \
    --photos     examples/photos \
    --trajectory examples/trajectory.csv \
    --calib      examples/calib.json \
    --min-camera-dist 0.5 --map-max-range 0 \
    --occlusion-cell 1 --occlusion-depth-tol 0.03 \
    --keep-uncolored --debug1 out-debug1.csv \
    --output out.las
```

Ожидаемые строки лога: `[camera]`, `[extrinsic]`, `[trajectory]`, `[photos]`,
`[cloud] ... gps_time present`, `[frames] N usable of M photos`, далее прогресс
`[frame i/N] ... cubes=... updates=... uniq=...`, затем `==== RESULT ====`.

`--debug1 <csv>` пишет, какой кадр дал цвет какой точке: легенда
`# photo N = файл` и строки `point,gps_time,photo,dt,dist,score`. Флаг работает
только вместе с `--cube`.

## Проверка

```bash
tests/smoke.sh            # собрать (если нужно) и проверить
tests/smoke.sh --rebuild  # то же, но с чистого build/
PYTHON=/path/to/python3 tests/smoke.sh   # если laspy не в системном python3
```

Проверяется: сборка из нуля, побитовое равенство результата при `--jobs 1` и
`--jobs auto`, совпадение `--debug1` с `examples/golden/debug1.csv.gz`,
sha256 выходного LAS против `examples/golden/expected.txt`, перенос
`gps_time`/координат/intensity/scale-offset из входа в выход (через laspy,
если доступен) и доля окрашенных точек с допуском 1 %.

sha256 выходного LAS привязан к сборке: другой компилятор или версия Eigen
могут дать расхождение в младших битах. В этом случае запустите с
`SMOKE_ALLOW_LAS_MISMATCH=1` — проверка станет предупреждением, остальные
останутся жёсткими.

## Ограничения

- только модель камеры **pinhole** (fisheye в кубовом режиме не поддерживается);
- **одна камера** на прогон;
- вход обязан быть **LAS с `gps_time`** (форматы 1/3/4/5/6…10), PCD не подойдёт —
  режиму нужно время каждой точки;
- выход — LAS 1.4 fmt 7; PCD-выход у кубового режима отсутствует;
- куб задаётся размером в метрах и разбиением с началом в нуле координат
  (`floor(x / cube)`), поэтому смена `--cube` меняет и порядок точек в выходе.

## Документы

- как устроен алгоритм и почему так — `ALGORITHM.md`;
- что на входе и выходе, включая подводные камни смещений в LAS — `DATA_FORMATS.md`;
- как перенести в свой проект по шагам — `INTEGRATION.md`.
