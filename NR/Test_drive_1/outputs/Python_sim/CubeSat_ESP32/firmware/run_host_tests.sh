#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
PYTHON_SIM_DIR=$(dirname "$(dirname "$SCRIPT_DIR")")
CC=${CC:-cc}

if ! command -v "$CC" >/dev/null 2>&1; then
    printf 'Host C compiler not found: %s\n' "$CC" >&2
    printf 'Install GCC or Clang, or set CC to a compatible compiler.\n' >&2
    exit 127
fi

TEMP_DIR=$(mktemp -d "${TMPDIR:-/tmp}/ae-c-tests.XXXXXX")
trap 'rm -rf "$TEMP_DIR"' EXIT HUP INT TERM

build_and_run() {
    name=$1
    source=$2
    test_source=$3
    "$CC" -std=c11 -Wall -Wextra -Werror \
        -I"$SCRIPT_DIR/main" \
        "$SCRIPT_DIR/main/$source" \
        "$SCRIPT_DIR/$test_source" \
        -lm -o "$TEMP_DIR/$name"
    "$TEMP_DIR/$name"
    printf 'PASS: %s\n' "$name"
}

cd "$PYTHON_SIM_DIR"
build_and_run yaw-controller-test yaw_controller.c test_yaw_controller.c
build_and_run attitude-estimator-test attitude_estimator.c test_attitude_estimator.c
build_and_run black-target-test black_target.c test_black_target.c
