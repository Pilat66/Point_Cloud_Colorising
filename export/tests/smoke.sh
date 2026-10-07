#!/usr/bin/env bash
# tests/smoke.sh — проверка собранного экспорта на примерах из examples/.
#
# Что проверяется:
#   1. сборка src/ (цель colorise_map) во временный каталог;
#   2. детерминизм: --jobs 1 и --jobs auto дают побайтово одинаковый LAS
#      и одинаковый --debug1 CSV;
#   3. совпадение --debug1 с эталоном examples/golden/debug1.csv.gz;
#   4. sha256 выходного LAS против examples/golden/expected.txt
#      (жёстко; обход — SMOKE_ALLOW_LAS_MISMATCH=1);
#   5. перенос полей: gps_time / int32-координаты / intensity / scale-offset
#      выхода равны входным (нужен python3 с laspy и numpy — иначе пропуск);
#   6. ключевые строки лога и доля окрашенных точек (допуск 1 %).
#
# Использование:
#   tests/smoke.sh              # собрать (если нужно) и проверить
#   tests/smoke.sh --rebuild    # удалить build/ и собрать заново
#   BUILD=/tmp/b WORK=/tmp/w tests/smoke.sh
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXPORT="$(cd "$HERE/.." && pwd)"
SRC="$EXPORT/src"
EXAMPLES="$EXPORT/examples"
GOLDEN="$EXAMPLES/golden"
BUILD="${BUILD:-/tmp/export-build}"
WORK="${WORK:-/tmp/export-smoke}"
PYTHON="${PYTHON:-python3}"

REBUILD=0
[[ "${1:-}" == "--rebuild" ]] && REBUILD=1

checks=0
failures=0
ok()   { checks=$((checks + 1)); printf '  ok   %s\n' "$1"; }
bad()  { checks=$((checks + 1)); failures=$((failures + 1)); printf '  FAIL %s\n' "$1"; }
note() { printf '  ..   %s\n' "$1"; }

echo "== 1. Сборка =="
if [[ $REBUILD == 1 ]]; then
    rm -rf "$BUILD"
fi
if [[ ! -x "$BUILD/colorise_map" ]]; then
    cmake -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release > "$WORK-cmake.log" 2>&1 \
        || { echo "cmake не отработал, лог: $WORK-cmake.log"; exit 1; }
    cmake --build "$BUILD" -j"$(nproc)" >> "$WORK-cmake.log" 2>&1 \
        || { echo "сборка упала, лог: $WORK-cmake.log"; exit 1; }
fi
if [[ -x "$BUILD/colorise_map" ]]; then
    ok "бинарник $BUILD/colorise_map"
else
    bad "бинарник не собран"
    exit 1
fi

echo "== 2. Прогоны на примерах =="
rm -rf "$WORK"
mkdir -p "$WORK"

run() {  # run <jobs> <tag>
    "$BUILD/colorise_map" --cube 1.0 \
        --cloud      "$EXAMPLES/map.las" \
        --photos     "$EXAMPLES/photos" \
        --trajectory "$EXAMPLES/trajectory.csv" \
        --calib      "$EXAMPLES/calib.json" \
        --min-camera-dist 0.5 --map-max-range 0 \
        --occlusion-cell 1 --occlusion-depth-tol 0.03 \
        --keep-uncolored --debug1 "$WORK/$2-debug1.csv" \
        --output "$WORK/$2.las" --jobs "$1" > "$WORK/$2.log" 2>&1
}

if run 1 single && run 0 auto; then
    ok "оба прогона завершились (rc=0)"
else
    bad "прогон завершился с ошибкой, логи: $WORK/single.log, $WORK/auto.log"
    tail -5 "$WORK/auto.log" 2>/dev/null | sed 's/^/       /'
fi

echo "== 3. Детерминизм (jobs 1 == auto) =="
if cmp -s "$WORK/single.las" "$WORK/auto.las"; then
    ok "LAS побайтово совпал"
else
    bad "LAS различается между --jobs 1 и --jobs auto"
fi
echo "== 4. Сверка с эталоном =="
GOLD_CSV="$WORK/golden-debug1.csv"
if gunzip -c "$GOLDEN/debug1.csv.gz" > "$GOLD_CSV" 2>/dev/null; then
    if diff -q "$GOLD_CSV" "$WORK/auto-debug1.csv" > /dev/null; then
        ok "debug1 CSV совпал с эталоном ($(wc -l < "$GOLD_CSV") строк)"
    else
        bad "debug1 CSV отличается от эталона, первые расхождения:"
        diff "$GOLD_CSV" "$WORK/auto-debug1.csv" | head -5 | sed 's/^/       /'
    fi
else
    bad "не удалось распаковать $GOLDEN/debug1.csv.gz (нужен gunzip)"
fi

EXP_LAS_SHA="$(sed -n 's/^sha256_las=//p' "$GOLDEN/expected.txt")"
GOT_LAS_SHA="$(sha256sum "$WORK/auto.las" | awk '{print $1}')"
if [[ "$GOT_LAS_SHA" == "$EXP_LAS_SHA" ]]; then
    ok "sha256 LAS совпал с эталоном"
elif [[ "${SMOKE_ALLOW_LAS_MISMATCH:-0}" == "1" ]]; then
    note "sha256 LAS отличается (разрешено SMOKE_ALLOW_LAS_MISMATCH=1): $GOT_LAS_SHA"
else
    bad "sha256 LAS отличается: получено $GOT_LAS_SHA, ожидалось $EXP_LAS_SHA"
    note "если сборка не та, что создавала эталон — сначала проверьте остальное,"
    note "затем при необходимости повторите с SMOKE_ALLOW_LAS_MISMATCH=1"
fi

echo "== 5. Перенос полей вход -> выход (laspy) =="
if "$PYTHON" -c 'import laspy, numpy' > /dev/null 2>&1; then
    if "$PYTHON" - "$EXAMPLES/map.las" "$WORK/auto.las" <<'PY'
import sys
import numpy as np
import laspy

src_path, out_path = sys.argv[1], sys.argv[2]
with laspy.open(src_path) as f:
    src = f.read()
with laspy.open(out_path) as f:
    out = f.read()

problems = []
if out.header.point_format.id != 7:
    problems.append("point format выхода = {}, ожидался 7".format(out.header.point_format.id))
if len(out.points) != len(src.points):
    problems.append("точек в выходе {} против {} во входе".format(len(out.points), len(src.points)))
if not np.array_equal(np.asarray(out.header.scales), np.asarray(src.header.scales)):
    problems.append("scale изменился: {} != {}".format(out.header.scales, src.header.scales))
if not np.array_equal(np.asarray(out.header.offsets), np.asarray(src.header.offsets)):
    problems.append("offset изменился")
for name in ("X", "Y", "Z", "intensity"):
    a = np.sort(np.asarray(getattr(src, name), np.int64))
    b = np.sort(np.asarray(getattr(out, name), np.int64))
    if a.shape != b.shape or not np.array_equal(a, b):
        problems.append("{} выхода != {} входа".format(name, name))
t_src = np.sort(np.asarray(src.gps_time, np.float64))
t_out = np.sort(np.asarray(out.gps_time, np.float64))
if not np.array_equal(t_src, t_out):
    problems.append("множество gps_time изменилось")
if len(np.unique(t_out)) and np.any(np.isnan(t_out)):
    problems.append("в gps_time выхода есть NaN")

if problems:
    print("; ".join(problems))
    sys.exit(1)
print("gps_time/XYZ/intensity/scale-offset перенесены без изменений ({} точек)".format(len(out.points)))
PY
    then
        ok "поля перенесены bit-exact"
    else
        bad "поля выхода отличаются от входа (см. строку выше)"
    fi
else
    note "$PYTHON без laspy/numpy — проверка переноса полей пропущена"
fi

echo "== 6. Лог и доля окрашенных =="
if grep -q "gps_time present" "$WORK/auto.log"; then
    ok "карта прочитана как LAS с gps_time"
else
    bad "в логе нет строки про gps_time"
fi
if grep -q "\[frames\] 3 usable of 3 photos" "$WORK/auto.log"; then
    ok "все 3 снимка получили позу"
else
    bad "кадры не совпали со снимками (ожидалось 3 из 3)"
    grep -E '\[trajectory\]|\[photos\]|\[frames\]' "$WORK/auto.log" | sed 's/^/       /'
fi

EXP_COL="$(sed -n 's/^coloured=//p' "$GOLDEN/expected.txt")"
EXP_TOTAL="$(sed -n 's/^total=//p' "$GOLDEN/expected.txt")"
GOT_LINE="$(grep -E '^  total=' "$WORK/auto.log" | tail -1)"
GOT_COL="$(grep -oE 'coloured=[0-9,]+' "$WORK/auto.log" | tail -1 | tr -d 'coloured=,')"
if [[ -z "$GOT_COL" ]]; then
    bad "не удалось разобрать строку RESULT: '$GOT_LINE'"
else
    tol=$(( EXP_COL / 100 + 1 ))
    if (( GOT_COL > EXP_COL - tol && GOT_COL < EXP_COL + tol )); then
        ok "окрашено $GOT_COL из $EXP_TOTAL (эталон $EXP_COL, допуск ±1 %)"
    else
        bad "окрашено $GOT_COL, эталон $EXP_COL (допуск ±1 %) — алгоритм изменился?"
    fi
fi

echo
if (( failures == 0 )); then
    echo "ИТОГ: PASS ($checks проверок)"
    exit 0
fi
echo "ИТОГ: FAIL ($failures из $checks проверок не прошли)"
exit 1

