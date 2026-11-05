
import sys
import csv
import time
import re
from pathlib import Path
from datetime import datetime

import serial
import matplotlib.pyplot as plt
from matplotlib import font_manager


# ==========================================================
# ESP32-S3 MOSFET I-V 자동 측정 및 그래프 생성
#
# 사용 예:
# python3 mosfet_graph.py /dev/cu.usbmodem101 2.0
#
# VGS는 명령어 마지막 숫자로 지정.
#
# Arduino 코드 변경 불필요.
# ==========================================================


# ==========================================================
# 1. 기본 설정
# ==========================================================

BAUD_RATE = 115200

DEFAULT_VGS = 2.0

MIN_VGS = 0.0
MAX_VGS = 3.0

# 시리얼 출력이 이 시간 동안 전혀 없으면 중단
SERIAL_IDLE_TIMEOUT = 180

GRAPH_END_MARKER = "그래프 데이터 추출 완료"

GRAPH_NAMES = {
    1: "예비측정",
    2: "본측정_IV",
    3: "선형회귀"
}


# ==========================================================
# 2. 한글 글꼴 설정
# ==========================================================

def configure_korean_font():

    available = {
        font.name
        for font in font_manager.fontManager.ttflist
    }

    candidates = [
        "AppleGothic",
        "NanumGothic",
        "Malgun Gothic",
        "Noto Sans CJK KR",
        "Arial Unicode MS"
    ]

    selected = None

    for font in candidates:
        if font in available:
            selected = font
            break

    if selected:
        plt.rcParams["font.family"] = selected

        print(
            "[그래프 설정] 한글 글꼴:",
            selected
        )
    else:
        print(
            "[주의] 한글 글꼴을 찾지 못했습니다."
        )
        print(
            "그래프의 한글이 깨질 수 있습니다."
        )

    plt.rcParams["axes.unicode_minus"] = False


# ==========================================================
# 3. VGS 입력값 확인
# ==========================================================

def get_vgs_target():

    if len(sys.argv) >= 3:

        text = sys.argv[2]

    else:

        text = input(
            "측정할 VGS를 입력하세요 (V): "
        ).strip()

        if not text:
            text = str(DEFAULT_VGS)

    try:
        value = float(text)

    except ValueError:
        raise ValueError(
            "VGS는 숫자로 입력해야 합니다."
        )

    if not (
        MIN_VGS <= value <= MAX_VGS
    ):
        raise ValueError(
            "현재 Arduino 코드의 VGS 입력 범위는 "
            "0~3V입니다."
        )

    return value


# ==========================================================
# 4. CSV 데이터 보조 함수
# ==========================================================

def to_float(value):

    try:
        return float(value.strip())

    except (ValueError, AttributeError):
        return None


def numeric_columns(
    rows,
    x_column,
    y_column
):

    x_values = []
    y_values = []

    for row in rows[1:]:

        if len(row) <= max(
            x_column,
            y_column
        ):
            continue

        x = to_float(row[x_column])
        y = to_float(row[y_column])

        if x is None or y is None:
            continue

        x_values.append(x)
        y_values.append(y)

    return x_values, y_values


def extract_column(rows, column):

    values = []

    for row in rows[1:]:

        if len(row) <= column:
            continue

        value = to_float(row[column])

        if value is not None:
            values.append(value)

    return values


def save_csv(
    folder,
    number,
    rows
):

    if not rows:
        return None

    path = (
        folder /
        f"{number}_{GRAPH_NAMES[number]}.csv"
    )

    with open(
        path,
        "w",
        newline="",
        encoding="utf-8-sig"
    ) as file:

        writer = csv.writer(file)
        writer.writerows(rows)

    return path


# ==========================================================
# 5. 그래프 생성 함수
# ==========================================================

def save_plot(
    folder,
    filename,
    title,
    xlabel,
    ylabel,
    series,
    horizontal_lines=None,
    scatter=False
):

    fig, ax = plt.subplots(
        figsize=(10, 6)
    )

    has_data = False

    for x, y, label in series:

        if not x or not y:
            continue

        if scatter:

            ax.scatter(
                x,
                y,
                s=22,
                label=label
            )

        else:

            ax.plot(
                x,
                y,
                marker="o",
                markersize=3,
                linewidth=1.5,
                label=label
            )

        has_data = True

    if not has_data:

        plt.close(fig)
        return None

    if horizontal_lines:

        for line_value, line_label in horizontal_lines:

            ax.axhline(
                y=line_value,
                linestyle="--",
                linewidth=1,
                alpha=0.7,
                label=line_label
            )

    ax.set_title(
        title,
        fontsize=16,
        pad=16
    )

    ax.set_xlabel(
        xlabel,
        fontsize=12
    )

    ax.set_ylabel(
        ylabel,
        fontsize=12
    )

    ax.grid(
        True,
        alpha=0.3
    )

    ax.legend(
        loc="best",
        fontsize=10
    )

    fig.tight_layout()

    path = folder / filename

    fig.savefig(
        path,
        dpi=250,
        bbox_inches="tight"
    )

    return path


# ==========================================================
# 6. 그래프 6개 생성
# ==========================================================

def generate_graphs(
    folder,
    tables,
    target_vgs
):

    paths = []

    vgs_label = (
        f"VGS 목표 = {target_vgs:.2f}V"
    )

    # ======================================================
    # 그래프 1: MOSFET ID-VDS 출력 특성
    #
    # 본 측정:
    # 0: 실제 VDS
    # 1: ID
    # 2: VGS
    # ======================================================

    if 2 in tables:

        rows = tables[2]

        vds, id_values = numeric_columns(
            rows,
            0,
            1
        )

        measured_vgs = extract_column(
            rows,
            2
        )

        if measured_vgs:

            avg_vgs = sum(
                measured_vgs
            ) / len(measured_vgs)

            vgs_description = (
                f"목표 VGS = {target_vgs:.2f}V, "
                f"실측 평균 = {avg_vgs:.4f}V"
            )

        else:

            vgs_description = vgs_label

        path = save_plot(
            folder,
            "01_MOSFET_ID_VDS.png",

            "MOSFET 출력 특성\n"
            + vgs_description,

            "VDS (드레인-소스 전압, V)",

            "ID (드레인 전류, mA)",

            [
                (
                    vds,
                    id_values,
                    f"VGS = {target_vgs:.2f}V"
                )
            ]
        )

        if path:
            paths.append(path)


    # ======================================================
    # 그래프 2: TIP41C 베이스-이미터 전압
    #
    # 예비 측정:
    # 0: DAC C
    # 3: 베이스
    # 4: 이미터
    # ======================================================

    if 1 in tables:

        rows = tables[1]

        xb, vb = numeric_columns(
            rows,
            0,
            3
        )

        xe, ve = numeric_columns(
            rows,
            0,
            4
        )

        path = save_plot(
            folder,
            "02_TIP41C_베이스_이미터.png",

            "TIP41C 베이스-이미터 전압 특성\n"
            + vgs_label,

            "DAC C (DAC 제어 전압, V)",

            "전압 (베이스 및 이미터 전압, V)",

            [
                (
                    xb,
                    vb,
                    "VB (베이스 전압)"
                ),
                (
                    xe,
                    ve,
                    "VE (이미터 전압)"
                )
            ]
        )

        if path:
            paths.append(path)


        # ==================================================
        # 그래프 3: 베이스-이미터 전달률
        #
        # 5: 원본 전달률
        # 6: 중앙값 필터 전달률
        # ==================================================

        xr, raw_ratio = numeric_columns(
            rows,
            0,
            5
        )

        xf, filtered_ratio = numeric_columns(
            rows,
            0,
            6
        )

        path = save_plot(
            folder,
            "03_전달률_비교.png",

            "TIP41C 베이스-이미터 전달률\n"
            + vgs_label,

            "DAC C (DAC 제어 전압, V)",

            "전압 전달률 (이미터 변화량 / 베이스 변화량, %)",

            [
                (
                    xr,
                    raw_ratio,
                    "원본 전달률"
                ),
                (
                    xf,
                    filtered_ratio,
                    "중앙값 필터 적용"
                )
            ],

            horizontal_lines=[
                (
                    95.0,
                    "전달률 하한 95%"
                ),
                (
                    105.0,
                    "전달률 상한 105%"
                )
            ]
        )

        if path:
            paths.append(path)


        # ==================================================
        # 그래프 4: DAC C - VDS 전달 특성
        #
        # 0: DAC C
        # 1: 원본 VDS
        # 2: 상대 VDS
        # ==================================================

        xc, vds_raw = numeric_columns(
            rows,
            0,
            1
        )

        xr, vds_rel = numeric_columns(
            rows,
            0,
            2
        )

        path = save_plot(
            folder,
            "04_DAC_C_VDS_특성.png",

            "DAC C에 따른 VDS 변화\n"
            + vgs_label,

            "DAC C (DAC 제어 전압, V)",

            "VDS (드레인-소스 전압, V)",

            [
                (
                    xc,
                    vds_raw,
                    "실제 VDS"
                ),
                (
                    xr,
                    vds_rel,
                    "초기값 보정 VDS"
                )
            ]
        )

        if path:
            paths.append(path)


    # ======================================================
    # 그래프 5: 선형 회귀 비교
    #
    # 회귀 CSV:
    # 0: DAC C
    # 1: 실측 상대 VDS
    # 2: 회귀 예측값
    # 4: 회귀 제외 여부
    # ======================================================

    if 3 in tables:

        rows = tables[3]

        xm, measured = numeric_columns(
            rows,
            0,
            1
        )

        xp, predicted = numeric_columns(
            rows,
            0,
            2
        )

        path = save_plot(
            folder,
            "05_선형회귀_비교.png",

            "DAC C - VDS 선형 회귀 비교\n"
            + vgs_label,

            "DAC C (DAC 제어 전압, V)",

            "VDS (보정된 드레인-소스 전압, V)",

            [
                (
                    xm,
                    measured,
                    "실제 측정값"
                ),
                (
                    xp,
                    predicted,
                    "선형 회귀 예측값"
                )
            ]
        )

        if path:
            paths.append(path)


        # ==================================================
        # 그래프 6: 회귀 잔차
        #
        # 0: DAC C
        # 3: 잔차
        # ==================================================

        xr, residuals = numeric_columns(
            rows,
            0,
            3
        )

        path = save_plot(
            folder,
            "06_회귀_잔차.png",

            "선형 회귀 잔차 분석\n"
            + vgs_label,

            "DAC C (DAC 제어 전압, V)",

            "잔차 (실측 VDS - 회귀 예측 VDS, V)",

            [
                (
                    xr,
                    residuals,
                    "회귀 잔차"
                )
            ],

            horizontal_lines=[
                (
                    0.0,
                    "오차 0V"
                ),
                (
                    0.05,
                    "허용 오차 +50mV"
                ),
                (
                    -0.05,
                    "허용 오차 -50mV"
                )
            ]
        )

        if path:
            paths.append(path)

    return paths


# ==========================================================
# 7. 시리얼 로그 수신
# ==========================================================

def read_serial_line(
    ser,
    log_file
):

    raw = ser.readline()

    if not raw:
        return None

    line = raw.decode(
        "utf-8",
        errors="replace"
    ).strip()

    if not line:
        return None

    print(line)

    log_file.write(
        line + "\n"
    )

    log_file.flush()

    return line


# ==========================================================
# 8. 측정 시작
# ==========================================================

def run_measurement(
    ser,
    log_file,
    target_vgs
):

    command = (
        f"v{target_vgs:g}\n"
    )

    print()
    print(
        "[Python] MOSFET 자동 측정을 시작합니다."
    )

    print(
        f"[Python] 설정 VGS = {target_vgs:.3f}V"
    )

    ser.write(
        command.encode("utf-8")
    )

    ser.flush()

    last_received = time.monotonic()

    error_seen = False

    while True:

        line = read_serial_line(
            ser,
            log_file
        )

        if line is None:

            if (
                time.monotonic() - last_received
                > SERIAL_IDLE_TIMEOUT
            ):
                raise TimeoutError(
                    "측정 로그가 오랫동안 수신되지 않았습니다."
                )

            continue

        last_received = time.monotonic()

        if "[오류로 중단]" in line:
            error_seen = True

        if "[측정 완료]" in line:
            return True

        if "[측정 종료]" in line:
            return False

        # 예비 측정 도중 오류가 난 경우에도
        # 저장된 일부 데이터를 가져올 수 있도록
        # 종료 문구를 우선 기다림.


# ==========================================================
# 9. 그래프 CSV 수신
# ==========================================================

def collect_graph_data(
    ser,
    log_file
):

    tables = {}

    current_graph = None

    print()
    print(
        "[Python] 저장된 그래프 데이터를 요청합니다."
    )

    ser.write(b"g\n")
    ser.flush()

    last_received = time.monotonic()

    expected_columns = {
        1: 9,
        2: 9,
        3: 5
    }

    while True:

        line = read_serial_line(
            ser,
            log_file
        )

        if line is None:

            if (
                time.monotonic() - last_received
                > SERIAL_IDLE_TIMEOUT
            ):
                raise TimeoutError(
                    "그래프 데이터 수신 시간 초과"
                )

            continue

        last_received = time.monotonic()

        if "[그래프 1]" in line:

            current_graph = 1
            tables[1] = []
            continue

        if "[그래프 2]" in line:

            current_graph = 2
            tables[2] = []
            continue

        if "[그래프 3]" in line:

            current_graph = 3
            tables[3] = []
            continue

        if GRAPH_END_MARKER in line:
            break

        if current_graph is None:
            continue

        if line.startswith("["):
            continue

        if line.startswith("="):
            continue

        if "," not in line:
            continue

        row = next(
            csv.reader([line])
        )

        if (
            len(row)
            != expected_columns[current_graph]
        ):
            continue

        tables[current_graph].append(row)

    print()
    print(
        "[Python] 그래프 데이터 수신 완료"
    )

    return tables


# ==========================================================
# 10. 메인 프로그램
# ==========================================================

def main():

    if len(sys.argv) < 2:

        print(
            "사용법:"
        )

        print(
            "python3 mosfet_graph.py "
            "/dev/cu.usbmodem101 2.0"
        )

        return

    port = sys.argv[1]

    try:

        target_vgs = get_vgs_target()

    except ValueError as error:

        print(
            "[입력 오류]",
            error
        )

        return

    configure_korean_font()

    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    vgs_tag = (
        f"{target_vgs:.2f}"
        .replace(".", "p")
    )

    folder = Path(
        f"MOSFET_VGS_{vgs_tag}V_{timestamp}"
    )

    folder.mkdir(
        parents=True,
        exist_ok=True
    )

    log_path = (
        folder / "전체_시리얼_로그.txt"
    )

    print()
    print(
        "======================================"
    )

    print(
        "MOSFET 자동 I-V 측정 프로그램"
    )

    print(
        "======================================"
    )

    print(
        "[설정] 포트:",
        port
    )

    print(
        "[설정] 통신 속도:",
        BAUD_RATE
    )

    print(
        "[설정] 목표 VGS:",
        f"{target_vgs:.3f}V"
    )

    print(
        "[설정] 저장 폴더:",
        folder
    )

    print(
        "======================================"
    )

    ser = None

    try:

        # DTR/RTS를 포트 열기 전에 비활성화
        ser = serial.Serial(
            port=None,
            baudrate=BAUD_RATE,
            timeout=0.5
        )

        ser.dtr = False
        ser.rts = False

        ser.port = port
        ser.open()

        time.sleep(2)

        ser.reset_input_buffer()

        with open(
            log_path,
            "w",
            encoding="utf-8"
        ) as log_file:

            # ------------------------------
            # 자동 측정
            # ------------------------------

            success = run_measurement(
                ser,
                log_file,
                target_vgs
            )

            # ESP32가 메인 루프로 복귀할 시간
            time.sleep(1.0)

            # ------------------------------
            # 그래프 데이터 수신
            # ------------------------------

            tables = collect_graph_data(
                ser,
                log_file
            )

        # ------------------------------
        # CSV 저장
        # ------------------------------

        print()
        print(
            "[Python] CSV 파일을 저장합니다."
        )

        for number, rows in tables.items():

            path = save_csv(
                folder,
                number,
                rows
            )

            if path:

                print(
                    "[CSV 저장]",
                    path
                )

        # ------------------------------
        # 그래프 생성
        # ------------------------------

        print()
        print(
            "[Python] 그래프 6개를 생성합니다."
        )

        paths = generate_graphs(
            folder,
            tables,
            target_vgs
        )

        for path in paths:

            print(
                "[그래프 저장]",
                path
            )

        print()
        print(
            "[생성된 그래프 수]",
            len(paths)
        )

        print(
            "[저장 위치]",
            folder.resolve()
        )

        if not success:

            print()
            print(
                "[주의] 측정이 중단되었으므로 "
                "일부 데이터만 있을 수 있습니다."
            )

        if paths:

            print()
            print(
                "[그래프 표시] 그래프 창을 엽니다."
            )

            # 각 그래프는 별도 Figure이므로
            # 여러 개의 창으로 나타날 수 있음
            plt.show()

        else:

            print(
                "[안내] 그래프를 만들 데이터가 없습니다."
            )

    except serial.SerialException as error:

        print()
        print(
            "[시리얼 연결 오류]",
            error
        )

        print(
            "Arduino IDE 시리얼 모니터가 "
            "닫혀 있는지 확인하세요."
        )

    except KeyboardInterrupt:

        print()
        print(
            "[사용자 중단] 중단 명령을 전송합니다."
        )

        if ser is not None and ser.is_open:

            try:

                ser.write(b"x\n")
                ser.flush()

            except serial.SerialException:
                pass

    except TimeoutError as error:

        print()
        print(
            "[통신 시간 초과]",
            error
        )

        if ser is not None and ser.is_open:

            try:
                ser.write(b"x\n")
                ser.flush()
            except serial.SerialException:
                pass

    finally:

        if ser is not None and ser.is_open:

            ser.close()

        print()
        print(
            "[Python] 시리얼 연결을 종료했습니다."
        )


if __name__ == "__main__":
    main()

