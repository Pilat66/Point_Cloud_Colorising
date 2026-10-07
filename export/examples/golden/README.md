# Эталонные выходы примера

Сгенерировано прогоном на файлах этого каталога (`map.las`, `photos/`,
`trajectory.csv`, `calib.json`) сборкой из `export/src` (коммит-источник
`f6a9202`), на машине, где создавался экспорт:

```bash
cmake -S src -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4
./build/colorise_map --cube 1.0 \
    --cloud      examples/map.las \
    --photos     examples/photos \
    --trajectory examples/trajectory.csv \
    --calib      examples/calib.json \
    --min-camera-dist 0.5 --map-max-range 0 \
    --occlusion-cell 1 --occlusion-depth-tol 0.03 \
    --keep-uncolored --debug1 golden-debug1.csv \
    --output golden-out.las
```

| Файл | Что это |
|---|---|
| `debug1.csv.gz` | `--debug1` этого прогона (gzip): 66 986 строк данных + легенда из 3 кадров и заголовок (66 990 строк всего) |
| `expected.txt` | ожидаемые счётчики и sha256 выходного LAS |

`tests/smoke.sh` сравнивает:

- `--debug1` нового прогона с `debug1.csv.gz` — построчно (жёсткая проверка);
- sha256 нового LAS с `expected.txt` — жёсткая, но привязана к сборке: другой
  компилятор/версия Eigen могут дать расхождение в младших битах. Поэтому
  предусмотрен обход `SMOKE_ALLOW_LAS_MISMATCH=1` (проверка становится
  предупреждением), а доля окрашенных точек всё равно сверяется с допуском 1 %.

Данные примера (`map.las` и снимки) — реальный фрагмент набора репозитория:
окно 1.5 с вокруг трёх подряд идущих кадров (1789476757.688…1789476758.688),
269 487 точек с `gps_time` в этом окне. Как пересобрать — `../README.md`.
