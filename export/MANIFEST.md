# MANIFEST — происхождение и контрольные суммы

| | |
|---|---|
| Репозиторий-источник | `Point_Cloud_Colorising` |
| Коммит | `f6a9202` (`[ITER-20260917-183550] feat: --debug1 — CSV «какой кадр дал цвет какой точке»`) |
| Дата экспорта | 2026-09-19 |
| Итерация экспорта | `ITER-20260919-204519` |
| Объём | 30 файлов, ~22 МБ (из них 21 МБ — данные примера) |

## Что скопировано verbatim

Десять файлов C++, включая vendored `nlohmann/json`, скопированы **побайтово**.
Проверка (из корня репозитория-источника):

```bash
cmp ColoriseMap/src/cube_colourise.cpp export/src/ColoriseMap/cube_colourise.cpp
cmp ColoriseMap/src/cube_colourise.hpp export/src/ColoriseMap/cube_colourise.hpp
cmp ColoriseMap/src/main.cpp           export/src/ColoriseMap/main.cpp
cmp colorise/include/colorise.h        export/src/colorise/include/colorise.h
cmp colorise/include/parallel.h        export/src/colorise/include/parallel.h
cmp colorise/include/voxel_grid.hpp    export/src/colorise/include/voxel_grid.hpp
cmp colorise/src/io.cpp                export/src/colorise/src/io.cpp
cmp colorise/src/trajectory.cpp        export/src/colorise/src/trajectory.cpp
cmp colorise/src/project.cpp           export/src/colorise/src/project.cpp
cmp colorise/third_party/nlohmann/json.hpp export/src/colorise/third_party/nlohmann/json.hpp
```

Все десять сравнений при подготовке экспорта совпали.

## Что отличается от репозитория

| Файл | Отличие |
|---|---|
| `src/CMakeLists.txt` | единственный изменённый: копия `ColoriseMap/CMakeLists.txt` с путями под раскладку экспорта (`ColoriseMap/src/*` → `ColoriseMap/*`, `colorise/*` → `colorise/*`); сами исходники не тронуты |
| `README.md`, `ALGORITHM.md`, `DATA_FORMATS.md`, `INTEGRATION.md`, `MANIFEST.md` | новые документы, в репозитории их нет |
| `examples/*` | новые данные примера (фрагмент реального набора), генерируются `examples/make_examples.py` |
| `tests/smoke.sh` | новый скрипт проверки |

Не попали в экспорт (сознательно): `colorise/CMakeLists.txt` (сборка кадрового
инструмента; в экспорте её заменяет единый `src/CMakeLists.txt`),
`colorise/build/`, `data/`, `2026-09-08-18-56-45/`, `venv/` и прочие артефакты и
данные репозитория — см. `.gitignore` источника.

## Контрольные суммы

Файлы отсортированы по пути; размеры — в байтах.

| Файл | Размер | sha256 |
|---|---|---|
| `README.md` | 7491 | `702cb6f8c38cdb7b521904885e694871be0ba7b6e09e0662048e4942dfc8292a` |
| `ALGORITHM.md` | 12137 | `e60e009be9a4ec9c89751da417ce69bc2fe0f20c71ad851c35893d9d542922db` |
| `DATA_FORMATS.md` | 10273 | `23718aaa5dd9d9483588542394c0da97dd217300ecdb46e5229cc16bbcea58ed` |
| `INTEGRATION.md` | 12130 | `9907af312ccf05267e1faad4b01ba976ddece25ae27cf8081ac4a4c39c3326c5` |
| `MANIFEST.md` | — | этот файл (сам себя не хеширует) |
| `docs/ColoriseMap_README.md` | 25465 | `706c63fec440abeb42fdb929631c8c79b03b72cce51fc9c7cecf803c640317b5` |
| `docs/Полезные наблюдения.md` | 5257 | `d0af1b9b5f100590b18e36709838337b06c2bab0e79bbfc79c846be85f209fca` |
| `src/CMakeLists.txt` | 1701 | `30ca18c2f9571d9645025798eeb505441045a6491bd05d498a7287c2359a2e2a` |
| `src/ColoriseMap/cube_colourise.hpp` | 2925 | `c079959e19707ecfb0ae164103f6571fedeb404d0ccd5ed3a1c13e5f47f8c663` |
| `src/ColoriseMap/cube_colourise.cpp` | 26824 | `7332988c94cda358fe991e72d3855441633f48e441cec56a7cfb80d67b8d467f` |
| `src/ColoriseMap/main.cpp` | 39193 | `0c0b4db343ec3aa3fd43aba79e6ce6c73ffec251db8f8a72d71a2934f085880f` |
| `src/colorise/include/colorise.h` | 10582 | `7729aae500bf7e46b42011991fdb0e253865ffbb4d1164dc8c5875197da76925` |
| `src/colorise/include/parallel.h` | 968 | `07204afb042bac0e2fe960e85df7f34174b1d46dae7666bb510ff51aeb7dcb2a` |
| `src/colorise/include/voxel_grid.hpp` | 6701 | `4b43b96b5b8ac14850ab49d7deb4235b3aea306de35aeaa68d1d52a2d72a768d` |
| `src/colorise/src/io.cpp` | 50392 | `152ab7625b41bccb6b8a1b0134d577932a9299b9beea5d827961c601f9b3cd07` |
| `src/colorise/src/trajectory.cpp` | 10892 | `e709ed90fa40cf6e9ebd84b98e6f779aef6781aef7b5690b1dcc17193d8d3d92` |
| `src/colorise/src/project.cpp` | 10067 | `95b08797c4636d3a79543dee5e46fb24b3d89e1e7afa8e3ec439988fa4be00f1` |
| `src/colorise/third_party/nlohmann/json.hpp` | 1142695 | `75f9c53eca2978c3738da2a4296742037d46393cf06f913fcae34cf0cf528704` |
| `examples/README.md` | 2468 | `c49701e5ec0a1d3c54a31f686dc590b4170262218c79e91c5ada5872b73da333` |
| `examples/make_examples.py` | 8521 | `db94dfefc94c912507e7529538cadc8c559ae5177b3c8c3232e3905a194a2c68` |
| `examples/calib.json` | 1184 | `834c5fab2b973dd27ca5f9a85c2b1db31134ccf8a901f5137dfb4f427dfac881` |
| `examples/trajectory.csv` | 29210 | `38ec12dca89869c71ec6e280886f6029318d552971193f8fe0f1e07426c13e75` |
| `examples/map.las` | 9701907 | `e42d274f26f3fc1155ccce1b2c5fb91431956c6f60988589399c542cfd6da6fd` |
| `examples/photos/1789476757688393926.png` | 3625787 | `f81be2cedb378cf782ef614a231a58ccec839ee4c9e436649550ea9bed9f2fd3` |
| `examples/photos/1789476758288428704.png` | 3666070 | `0d6f0e6b0fd5f1c32c47154b87e831547b5f85b2f1e558378c4e25b628faa081` |
| `examples/photos/1789476758688450592.png` | 3666860 | `6b2913081e1423bc1b10ed00ffae9922cd893303460e6d8300ce3a4b644c857f` |
| `examples/golden/README.md` | 2107 | `7e5caf18b4840b8582787f80592754f0f7c9335d2efd947a71b44ed9bdd567d6` |
| `examples/golden/debug1.csv.gz` | 770943 | `241509580df4c11b258f9fc7b2bb7dc6f9a7c436a14a38e652d7cf1a1f3c04cc` |
| `examples/golden/expected.txt` | 470 | `d801af936b371b4fce09b0e098652bfcf7491d455415786c5174345d6128de6f` |
| `tests/smoke.sh` | 8441 | `a774bef0b9b30ab604c6d70a10bb2f512bef6f35ef2ed7d32bdd9c8394042ffc` |

Проверить целостность целиком (сравнить вывод с таблицей выше):

```bash
cd export && find . -type f ! -name MANIFEST.md -print0 | sort -z \
    | xargs -0 sha256sum
```

## Как получены данные примера

`examples/make_examples.py` (laspy) из `data/` репозитория-источника: окно
1.5 с вокруг трёх подряд идущих кадров `data/img-uniq`, вырезка точек с
`gps_time` в этом окне (269 487 из 56 154 407) с сохранением формата 7, scale,
offset, int32-координат, intensity и времени. Подробности и команда —
`examples/README.md`.

## Как проверялся экспорт при подготовке

1. Сборка `export/src` с нуля в пустом каталоге — успешно.
2. Сборка копии экспорта вне репозитория (`/tmp/export-standalone`) — успешно:
   набор самодостаточен, абсолютных путей репозитория внутри нет
   (`grep -rn '/home/reestr' export` — пусто).
3. `tests/smoke.sh --rebuild` — 9 проверок PASS (детерминизм `--jobs 1` vs
   auto, `--debug1` == эталон, sha256 LAS == эталон, bit-exact перенос
   `gps_time`/координат/intensity/scale-offset).
4. Все десять `cmp` копий исходников против репозитория — совпали.

