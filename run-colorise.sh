#!/usr/bin/env bash
# ===========================================================================
# run-colorise.sh — серия прогонов `colorise` по ПОЛНОЙ матрице параметров.
#
# Шаблон вызова (N — номер варианта, 1..192):
#
#   colorise --cloud test1.las --photos img1 --trajectory trajectory.csv \
#            --camera camera.yaml --calib calib.json \
#            --min-camera-dist 1 --min-color-frames 1 \
#            --output test1-colored-<N>.las [параметры варианта N]
#
# Матрица: порядок уровней — от внешнего к внутреннему, он же порядок N.
#
#   --edge-margin            2 значения:  0.0            100
#   --keep-uncolored         2 значения:  yes            no
#   --first-wins             2 значения:  no (усредн.)   yes
#   --occlusion              2 значения:  on             off
#   --occlusion-depth-tol    3 значения:  0.1            0.2  0.3
#   --occlusion-cell         4 значения:  1.0            2.0  3.0  4.0
#
#   2 * 2 * 2 * 2 * 3 * 4 = 192 варианта.
#
# Фиксированные параметры из шаблона: --min-camera-dist 1, --min-color-frames 1.
# N присваивается один раз и не зависит от ключей запуска (см. манифест
# test1-colored-variants.tsv и вывод --list).
# ===========================================================================
set -euo pipefail

PROG="${0##*/}"

usage() {
    cat <<EOF
Использование: $PROG [опции] [N ...]

Серия прогонов colorise по полной матрице параметров (192 варианта).
Входные файлы берутся по именам из шаблона: test1.las, img1/, trajectory.csv,
camera.yaml, calib.json. Результат варианта N: test1-colored-<N>.las,
лог: test1-colored-<N>.log, манифест: test1-colored-variants.tsv.

Позиционные аргументы N ...  — запустить только эти варианты
                               (без аргументов: все 192 по порядку).

Опции:
  -d, --dir DIR        каталог с входными данными            [. — текущий]
  -o, --out-dir DIR    каталог для .las и .log (создаётся)   [= --dir]
  -j, --jobs N         проброс --jobs в colorise (потоки)    [по умолчанию colorise]
  -f, --force          перезаписывать уже готовые .las       [пропускать]
      --distinct-only  не запускать варианты, различающиеся только
                       --occlusion-cell/--occlusion-depth-tol при
                       --no-occlusion (эти параметры там не влияют)
  -n, --dry-run        только напечатать команды, ничего не запускать
  -v, --verbose        дублировать в консоль вывод colorise и все строки
                       пропусков (по умолчанию пропуски — только счётчиками)
  -l, --list           напечатать таблицу N -> параметры и выйти
  -h, --help           эта справка

Примеры:
  $PROG                       # все 192 варианта в текущем каталоге
  $PROG -d ../run1 -o out 3 5 12
  $PROG --list | head
  $PROG -n 1                  # показать команду первого варианта
EOF
}

# ── Значения по умолчанию (имена файлов — из шаблона вызова) ────────────────
DATA_DIR="."
OUT_DIR="."
JOBS=""
FORCE=0
DRY_RUN=0
DISTINCT_ONLY=0
VERBOSE=0
DO_LIST=0
SELECTED=()

declare -r CLOUD_FILE="test1.las"
declare -r PHOTOS_SUBDIR="img1"
declare -r TRAJECTORY_FILE="trajectory.csv"
declare -r CAMERA_FILE="camera.yaml"
declare -r CALIB_FILE="calib.json"
declare -r OUT_PREFIX="test1-colored"

declare -r MIN_CAMERA_DIST="1"
declare -r MIN_COLOR_FRAMES="1"

# Матрица значений. Порядок уровней = порядок разрядов в номере варианта N.
declare -ar EDGE_MARGIN_VALUES=(0.0 100)
declare -ar KEEP_UNCOLORED_VALUES=(yes no)
declare -ar FIRST_WINS_VALUES=(no yes)
declare -ar OCCLUSION_VALUES=(on off)
declare -ar OCCLUSION_DEPTH_TOL_VALUES=(0.1 0.2 0.3)
declare -ar OCCLUSION_CELL_VALUES=(1.0 2.0 3.0 4.0)

# ── Разбор аргументов ───────────────────────────────────────────────────────
while (($#)); do
    case "$1" in
        -d|--dir)         DATA_DIR="${2:?$1 требует значение}"; shift 2 ;;
        -o|--out-dir)     OUT_DIR="${2:?$1 требует значение}";  shift 2 ;;
        -j|--jobs)        JOBS="${2:?$1 требует значение}";     shift 2 ;;
        -f|--force)       FORCE=1; shift ;;
        -n|--dry-run)     DRY_RUN=1; shift ;;
        -v|--verbose)     VERBOSE=1; shift ;;
        -l|--list)        DO_LIST=1; shift ;;
        --distinct-only)  DISTINCT_ONLY=1; shift ;;
        -h|--help)        usage; exit 0 ;;
        --)               shift; while (($#)); do SELECTED+=("$1"); shift; done ;;
        -*)               printf '%s: неизвестная опция: %s\n\n' "$PROG" "$1" >&2; usage >&2; exit 2 ;;
        *)                SELECTED+=("$1"); shift ;;
    esac
done

die() { printf '%s: ошибка: %s\n' "$PROG" "$*" >&2; exit 1; }

if [[ -n "$JOBS" && ! "$JOBS" =~ ^[0-9]+$ ]]; then
    die "--jobs ожидает целое число >= 0, получено '$JOBS'"
fi

if [[ ! -d "$DATA_DIR" ]]; then
    die "нет каталога с данными: $DATA_DIR"
fi
DATA_DIR="$(cd -- "$DATA_DIR" && pwd)"
cd -- "$DATA_DIR"
mkdir -p -- "$OUT_DIR"
OUT_DIR="${OUT_DIR%/}"
if [[ -z "$OUT_DIR" ]]; then OUT_DIR="/"; fi

# ── Вспомогательные функции ────────────────────────────────────────────────
is_selected() {  # $1 = N; 0 — если вариант выбран (пустой список = все)
    local n="$1" s
    ((${#SELECTED[@]} == 0)) && return 0
    for s in "${SELECTED[@]}"; do
        if [[ "$s" == "$n" ]]; then return 0; fi
    done
    return 1
}

out_path() {  # $1 = имя файла -> путь с учётом --out-dir (по умолчанию — как в шаблоне)
    if [[ "$OUT_DIR" == "." ]]; then
        printf '%s\n' "$1"
    elif [[ "$OUT_DIR" == "/" ]]; then
        printf '/%s\n' "$1"
    else
        printf '%s/%s\n' "${OUT_DIR%/}" "$1"
    fi
}

# build_args <edge-margin> <keep-uncolored> <first-wins> <occlusion> <tol> <cell> <output-name>
# Заполняет глобальный массив ARGS — ровно то, что передаётся в colorise.
build_args() {
    ARGS=(
        --cloud "$CLOUD_FILE"
        --photos "$PHOTOS_SUBDIR"
        --trajectory "$TRAJECTORY_FILE"
        --camera "$CAMERA_FILE"
        --calib "$CALIB_FILE"
        --min-camera-dist "$MIN_CAMERA_DIST"
        --min-color-frames "$MIN_COLOR_FRAMES"
        --edge-margin "$1"
        --output "$(out_path "$7")"
    )
    if [[ "$4" == on ]]; then
        ARGS+=( --occlusion --occlusion-cell "$6" --occlusion-depth-tol "$5" )
    elif (( DISTINCT_ONLY )); then
        # При --no-occlusion ячейка/допуск не влияют на результат — не дублируем.
        ARGS+=( --no-occlusion )
    else
        ARGS+=( --no-occlusion --occlusion-cell "$6" --occlusion-depth-tol "$5" )
    fi
    if [[ "$2" == yes ]]; then ARGS+=( --keep-uncolored ); fi
    if [[ "$3" == yes ]]; then ARGS+=( --first-wins ); fi
    if [[ -n "$JOBS" ]]; then ARGS+=( --jobs "$JOBS" ); fi
}

args_to_cmd() {  # печатает командную строку с корректным квотированием
    local str="colorise" a q
    for a in "$@"; do
        printf -v q ' %q' "$a"
        str+="$q"
    done
    printf '%s\n' "$str"
}

# ── Построение полной матрицы ──────────────────────────────────────────────
declare -a ROW_EM ROW_KU ROW_FW ROW_OC ROW_TOL ROW_CELL ROW_OUT ROW_DESC ROW_CMD ROW_PLAN
declare -a ARGS
row_count=0

for em in "${EDGE_MARGIN_VALUES[@]}"; do
    for ku in "${KEEP_UNCOLORED_VALUES[@]}"; do
        for fw in "${FIRST_WINS_VALUES[@]}"; do
            for oc in "${OCCLUSION_VALUES[@]}"; do
                for tol in "${OCCLUSION_DEPTH_TOL_VALUES[@]}"; do
                    for cell in "${OCCLUSION_CELL_VALUES[@]}"; do
                        n=$((row_count + 1))
                        name="$OUT_PREFIX-$n.las"
                        plan="run"

                        # При выключенной окклюзии cell/tol на результат не влияют.
                        if (( DISTINCT_ONLY )) && [[ "$oc" == off ]] &&
                           { [[ "$tol" != "${OCCLUSION_DEPTH_TOL_VALUES[0]}" ]] ||
                             [[ "$cell" != "${OCCLUSION_CELL_VALUES[0]}" ]]; }; then
                            plan="skip-redundant"
                        fi
                        if ! is_selected "$n"; then plan="skip-not-selected"; fi

                        ROW_EM[$row_count]="$em"
                        ROW_KU[$row_count]="$ku"
                        ROW_FW[$row_count]="$fw"
                        ROW_OC[$row_count]="$oc"
                        ROW_TOL[$row_count]="$tol"
                        ROW_CELL[$row_count]="$cell"
                        ROW_OUT[$row_count]="$name"
                        ROW_DESC[$row_count]="edge-margin=$em occlusion=$oc"
                        ROW_DESC[$row_count]+=" occlusion-cell=$cell occlusion-depth-tol=$tol"
                        ROW_DESC[$row_count]+=" keep-uncolored=$ku first-wins=$fw"
                        ROW_PLAN[$row_count]="$plan"

                        build_args "$em" "$ku" "$fw" "$oc" "$tol" "$cell" "$name"
                        ROW_CMD[$row_count]="$(args_to_cmd "${ARGS[@]}")"

                        row_count=$n
                    done
                done
            done
        done
    done
done

# Проверка выбранных номеров вариантов
for s in "${SELECTED[@]}"; do
    if [[ ! "$s" =~ ^[0-9]+$ ]] || (( s < 1 || s > row_count )); then
        die "неверный номер варианта: '$s' (допустимо 1..$row_count)"
    fi
done

# ── Имя манифеста ──────────────────────────────────────────────────────────
MANIFEST="$(out_path "$OUT_PREFIX-variants.tsv")"

# ── --list: таблица N -> параметры ─────────────────────────────────────────
if (( DO_LIST )); then
    printf '%s: %d вариантов (полная матрица), фиксировано: --min-camera-dist %s --min-color-frames %s\n' \
        "$PROG" "$row_count" "$MIN_CAMERA_DIST" "$MIN_COLOR_FRAMES"
    for ((i = 0; i < row_count; i++)); do
        printf '%4d  %-15s  %s\n' "$((i + 1))" "${ROW_PLAN[i]}" "${ROW_DESC[i]}"
    done
    exit 0
fi

# ── --dry-run: только команды ──────────────────────────────────────────────
if (( DRY_RUN )); then
    for ((i = 0; i < row_count; i++)); do
        n=$((i + 1))
        if ! is_selected "$n"; then continue; fi
        if [[ "${ROW_PLAN[i]}" != "run" ]]; then
            printf '# %d  %s  [%s — команда не выполняется]\n' \
                "$n" "${ROW_OUT[i]}" "${ROW_PLAN[i]}"
            continue
        fi
        printf '# %d  %s  [%s]\n%s\n' "$n" "${ROW_OUT[i]}" "${ROW_PLAN[i]}" "${ROW_CMD[i]}"
    done
    exit 0
fi

# ── Проверки входных данных и бинарника ────────────────────────────────────
for f in "$CLOUD_FILE" "$TRAJECTORY_FILE" "$CAMERA_FILE" "$CALIB_FILE"; do
    if [[ ! -f "$f" ]]; then
        die "нет входного файла: $DATA_DIR/$f (ожидается имя из шаблона вызова)"
    fi
done
if [[ ! -d "$PHOTOS_SUBDIR" ]]; then
    die "нет каталога с фотографиями: $DATA_DIR/$PHOTOS_SUBDIR"
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" 2>/dev/null && pwd || true)"
COLORISE_BIN=""
if command -v colorise >/dev/null 2>&1; then
    COLORISE_BIN="$(command -v colorise)"
elif [[ -n "$SCRIPT_DIR" && -x "$SCRIPT_DIR/colorise/build/colorise_offline" ]]; then
    COLORISE_BIN="$SCRIPT_DIR/colorise/build/colorise_offline"
elif [[ -n "$SCRIPT_DIR" && -x "$SCRIPT_DIR/colorise_offline" ]]; then
    COLORISE_BIN="$SCRIPT_DIR/colorise_offline"
else
    die "не найден бинарник colorise (ни в PATH, ни рядом со скриптом)"
fi

trap 'printf "\n%s: прервано; повторный запуск продолжит серию (готовые .las пропускаются, -f перезаписывает)\n" "$PROG" >&2; exit 130' INT TERM

# ── Манифест: N -> параметры -> команда ────────────────────────────────────
{
    printf '# %s: %d вариантов (полная матрица), создано %s\n' \
        "$PROG" "$row_count" "$(date '+%Y-%m-%d %H:%M:%S')"
    printf '# фиксировано: --min-camera-dist %s --min-color-frames %s\n' \
        "$MIN_CAMERA_DIST" "$MIN_COLOR_FRAMES"
    printf 'N\tplan\toutput\tparameters\tcommand\n'
    for ((i = 0; i < row_count; i++)); do
        printf '%d\t%s\t%s\t%s\t%s\n' \
            "$((i + 1))" "${ROW_PLAN[i]}" "${ROW_OUT[i]}" "${ROW_DESC[i]}" "${ROW_CMD[i]}"
    done
} > "$MANIFEST"

# ── Запуск серии ───────────────────────────────────────────────────────────
ok=0
fail=0
skipped=0
sel_skip=0
red_skip=0
exist_skip=0
run_plan=0
max_out=0
declare -a SUMMARY=()

printf '%s: %d вариантов | данные: %s | бинарник: %s\n' \
    "$PROG" "$row_count" "$DATA_DIR" "$COLORISE_BIN"
if ((${#SELECTED[@]})); then
    printf '  выбраны варианты: %s\n' "${SELECTED[*]}"
fi
printf '  манифест: %s\n\n' "$MANIFEST"

for ((i = 0; i < row_count; i++)); do
    n=$((i + 1))
    plan="${ROW_PLAN[i]}"
    name="${ROW_OUT[i]}"
    out="$(out_path "$name")"
    log="$(out_path "$OUT_PREFIX-$n.log")"

    if [[ "$plan" != "run" ]]; then
        skipped=$((skipped + 1))
        case "$plan" in
            skip-not-selected) sel_skip=$((sel_skip + 1)) ;;
            skip-redundant)    red_skip=$((red_skip + 1)) ;;
        esac
        if (( VERBOSE )); then
            printf '[%3d/%3d] %-15s %s\n' "$n" "$row_count" "$plan" "${ROW_DESC[i]}"
        fi
        continue
    fi
    if [[ -f "$out" ]] && (( FORCE == 0 )); then
        skipped=$((skipped + 1))
        exist_skip=$((exist_skip + 1))
        printf '[%3d/%3d] %-15s %s -> %s\n' "$n" "$row_count" "skip:exists" "${ROW_DESC[i]}" "$out"
        continue
    fi

    # Защита от переполнения диска: ориентир — самый большой уже готовый .las.
    if (( max_out > 0 )); then
        avail_kb="$(df -Pk -- "$OUT_DIR" | awk 'NR == 2 {print $4}' || true)"
        avail_kb="${avail_kb:-0}"
        need_kb=$(( max_out / 1024 * 12 / 10 + 2048 ))
        if (( avail_kb < need_kb )); then
            die "мало места: свободно ${avail_kb} KiB, для следующего .las нужно ~${need_kb} KiB; освободите место и запустите скрипт снова (готовые .las пропускаются)"
        fi
    fi

    build_args "${ROW_EM[i]}" "${ROW_KU[i]}" "${ROW_FW[i]}" \
               "${ROW_OC[i]}" "${ROW_TOL[i]}" "${ROW_CELL[i]}" "$name"

    run_plan=$((run_plan + 1))

    printf '[%3d/%3d] %-15s %s\n' "$n" "$row_count" "run" "${ROW_DESC[i]}"
    t0=$SECONDS
    rc=0
    set +e
    if (( VERBOSE )); then
        "$COLORISE_BIN" "${ARGS[@]}" 2>&1 | tee "$log"
        rc=${PIPESTATUS[0]}
    else
        "$COLORISE_BIN" "${ARGS[@]}" > "$log" 2>&1
        rc=$?
    fi
    set -e
    dt=$((SECONDS - t0))

    if (( rc != 0 )); then
        fail=$((fail + 1))
        printf '        [!] colorise вернул код %d за %ds; лог: %s\n' "$rc" "$dt" "$log"
        tail -n 5 -- "$log" | sed 's/^/        | /'
        SUMMARY+=("$(printf '%4d  %-4s %6ds  rc=%d  %s' "$n" "FAIL" "$dt" "$rc" "${ROW_DESC[i]}")")
        continue
    fi

    ok=$((ok + 1))
    if [[ -f "$out" ]]; then
        size="$(stat -c '%s' -- "$out" || true)"
        size="${size:-0}"
        if (( size > max_out )); then max_out=$size; fi
    fi

    res_line="$(grep -m1 -A1 '^==== RESULT ====' -- "$log" | tail -n1 || true)"
    tot="$(sed -n 's/.*total=\([0-9][0-9,]*\).*/\1/p' <<<"$res_line" || true)"
    col="$(sed -n 's/.*coloured=\([0-9][0-9,]*\).*/\1/p' <<<"$res_line" || true)"
    pct="$(sed -n 's/.*(\([0-9.]*\)%).*/\1/p' <<<"$res_line" || true)"
    SUMMARY+=("$(printf '%4d  %-4s %6ds  coloured=%-12s total=%-12s (%s%%)  %s' \
        "$n" "OK" "$dt" "${col:-?}" "${tot:-?}" "${pct:-?}" "${ROW_DESC[i]}")")
done

# ── Итог ───────────────────────────────────────────────────────────────────
printf '\n==== ИТОГО ====\n'
printf 'вариантов всего: %d | к запуску: %d | выполнено: %d | с ошибкой: %d | пропущено: %d\n' \
    "$row_count" "$run_plan" "$ok" "$fail" "$skipped"
printf 'пропущено: не выбраны %d, дубли (--no-occlusion) %d, уже готовы %d\n' \
    "$sel_skip" "$red_skip" "$exist_skip"
printf 'манифест:  %s\n' "$MANIFEST"
printf 'логи:      %s\n' "$(out_path "$OUT_PREFIX-<N>.log")"
printf '.las файлы: %s\n' "$(out_path "$OUT_PREFIX-<N>.las")"

if ((${#SUMMARY[@]})); then
    printf '\n%4s  %-4s %6s  %s\n' "N" "стат" "время" "результат"
    for line in "${SUMMARY[@]}"; do printf '%s\n' "$line"; done
fi

if (( fail > 0 )); then
    printf '\n%s: %d вариант(ов) завершились с ошибкой\n' "$PROG" "$fail" >&2
    exit 1
fi
printf '\n%s: серия завершена успешно\n' "$PROG"
