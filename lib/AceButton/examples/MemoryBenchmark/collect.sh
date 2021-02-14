#!/bin/bash
#
# Shell script that runs 'auniter verify ${board} MemoryBenchmark.ino', and
# collects the flash memory and static RAM usage for each of the FEATURE [0,5].
#
# Usage: collect.sh {board} {result_file}
#
# Creates a ${board}.out file containing:
#
#  FEATURE flash max_flash ram max_ram
#  0 aa bb cc dd
#  1 aa bb cc dd
#  ...
#  5 aa bb cc dd

set -eu

PROGRAM_NAME='MemoryBenchmark.ino'

# Assume that https://github.com/bxparks/AUniter is installed as a
# sibling project to AceButton.
AUNITER_CMD='../../../AUniter/tools/auniter.sh'
auniter_out_file=

function usage() {
    echo 'Usage: collect.sh {board} {result_file}'
    exit 1
}

function cleanup() {
    # Remove any temporary file
    if [[ "$auniter_out_file" != '' ]]; then
        rm -f $auniter_out_file
    fi

    # Restore the 'FEATURE' line
    sed -i -e "s/#define FEATURE [0-9]*/#define FEATURE 0/" $PROGRAM_NAME
}

function create_temp_file() {
    auniter_out_file=
    auniter_out_file=$(mktemp /tmp/memory_benchmark.auniter.XXXXXX)
}

# Usage: collect_for_board $board $result_file
# Output: A file named '${board}.out' that contains
function collect_for_board() {
    local cli_flag="$1"
    local board=$2
    local result_file=$3

    for feature in {0..5}; do
        echo "Collecting flash and ram usage for FEATURE $feature"
        sed -i -e "s/#define FEATURE [0-9]*/#define FEATURE $feature/" \
            $PROGRAM_NAME

        if ! ($AUNITER_CMD "$cli_flag" verify $board $PROGRAM_NAME 2>&1) > \
                $auniter_out_file; then
            # Ignore 'Sketch too big' condition, since we just want to
            # collect the flash and ram usage numbers.
            if ! grep -q 'Sketch too big' $auniter_out_file; then
                cat $auniter_out_file
                exit 1
            fi
        fi
        flash=$(grep 'Sketch uses' $auniter_out_file |
            awk '{print $3, $12}')
        memory=$(grep 'Global variables' $auniter_out_file |
            awk '{print $4, $18}')
        echo $feature $flash $memory >> $result_file
    done

}

trap "cleanup" EXIT

# Parse flags.
cli_flag='--cli'
while [[ $# -gt 0 ]]; do
    case $1 in
        --cli) cli_flag='--cli' ;;
        --ide) cli_flag='--ide' ;;
        --help|-h) usage ;;
        --) shift; break ;;
        -*) echo "Unknown flag '$1'" 1>&2; usage 1>&2 ;;
        *) break ;;
    esac
    shift
done
if [[ $# < 2 ]]; then
    usage
fi

rm -f $2
create_temp_file
echo "==== Collecting for board: $1"
collect_for_board "$cli_flag" "$@"
