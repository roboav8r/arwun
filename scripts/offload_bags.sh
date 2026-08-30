#!/usr/bin/env bash
#
# Copy finished bags off the Jetson to the workstation.
#
# WHY THIS EXISTS RATHER THAN A BARE rsync: two hazards specific to this rig.
#
# 1. AN IN-PROGRESS BAG MUST NOT BE COPIED. rosbag2 writes metadata.yaml on its
#    normal shutdown path, so a bag directory WITHOUT metadata.yaml is either
#    still being written or was killed unfinalized. Copying one gets you a
#    truncated file that `ros2 bag info` refuses, and -- worse -- the copy
#    looks complete. This script only offers up directories that have
#    metadata.yaml, so a take in progress is skipped rather than corrupted.
#
# 2. DISK IS THE BINDING CONSTRAINT, so deleting after transfer is the point of
#    the exercise, and a bad delete costs a field session. --delete-local is
#    therefore opt-in, runs only after rsync exits 0, and re-verifies with a
#    checksum pass before removing anything.
#
# THE LINK IS WIFI AND IT IS THE SLOW PART. This Jetson has no wired interface
# up -- it is on wlP1p1s0, and the payload is ~79 MB/s produced against a link
# that will not sustain anything close to that. Budget offload time in multiples
# of record time, not fractions of it: a full 32-minute disk is a multi-hour
# transfer. Recording and offloading at once will cost you frames.
#
# Usage:
#   ./scripts/offload_bags.sh                    # copy everything finalized
#   ./scripts/offload_bags.sh --dry-run          # show what would move
#   ./scripts/offload_bags.sh --delete-local     # copy, verify, then free disk
#   ./scripts/offload_bags.sh --bag arwun_2026...  # just one
#
# THE DESTINATION IS NOT HARDCODED, on purpose. This repository is public, and
# a default of the form user@10.x.y.z publishes someone's account name and the
# internal layout of their network to everyone who clones it, in exchange for
# saving one line of local setup. Set it once per rig, whichever way suits:
#
#   git config --local arwun.offloadDest 'user@workstation:~/arwun_bags/'
#   export ARWUN_DEST='user@workstation:~/arwun_bags/'
#   ./scripts/offload_bags.sh --dest user@workstation:~/arwun_bags/
#
# Precedence is --dest, then ARWUN_DEST, then git config. The source directory
# follows the same idea via ARWUN_BAGS, defaulting to ~/arwun_bags.

set -euo pipefail

# Git config is consulted last and must not abort the script when unset, hence
# the `|| true` -- `git config --get` exits 1 on a missing key, which `set -e`
# would otherwise treat as fatal.
DEST="${ARWUN_DEST:-$(git config --get arwun.offloadDest 2>/dev/null || true)}"
SRC="${ARWUN_BAGS:-$HOME/arwun_bags}"

DRY_RUN=0
DELETE_LOCAL=0
ONLY_BAG=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run)      DRY_RUN=1; shift ;;
        --delete-local) DELETE_LOCAL=1; shift ;;
        --bag)          ONLY_BAG="${2:?--bag needs a bag directory name}"; shift 2 ;;
        --dest)         DEST="${2:?--dest needs user@host:/path}"; shift 2 ;;
        # Prints the header block above, i.e. everything up to `set -euo`.
        # Derived rather than hardcoded so editing the header cannot silently
        # truncate --help, which is exactly what happened when the destination
        # note was added and the old fixed range stopped covering it.
        -h|--help)      sed -n "2,$(($(grep -n '^set -euo' "$0" | cut -d: -f1) - 1))p" "$0"; exit 0 ;;
        *)              echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [[ -z "$DEST" ]]; then
    cat >&2 <<'EOF'
No offload destination set. Pick one of:

  git config --local arwun.offloadDest 'user@workstation:~/arwun_bags/'
  export ARWUN_DEST='user@workstation:~/arwun_bags/'
  ./scripts/offload_bags.sh --dest user@workstation:~/arwun_bags/

The first is per-clone and stays out of version control, which is what you
want on a rig: this repository is public and a committed default would
publish an account name and an internal address.
EOF
    exit 2
fi

[[ -d "$SRC" ]] || { echo "no such bag directory: $SRC" >&2; exit 1; }

# --- pick the finalized bags ----------------------------------------------
# A bag is a DIRECTORY containing metadata.yaml. Anything else in here is
# either mid-write or wreckage, and neither should travel.
mapfile -t BAGS < <(
    find "$SRC" -mindepth 1 -maxdepth 1 -type d \
        ${ONLY_BAG:+-name "$ONLY_BAG"} \
        -exec test -f '{}/metadata.yaml' \; -print | sort
)

if [[ ${#BAGS[@]} -eq 0 ]]; then
    echo "Nothing to offload: no finalized bags in $SRC"
    # Call out unfinalized directories rather than staying silent about them --
    # they are the ones that need `ros2 bag reindex`, and a bare "nothing to do"
    # would hide a take you thought you had.
    unfinalized=$(find "$SRC" -mindepth 1 -maxdepth 1 -type d \
        '!' -exec test -f '{}/metadata.yaml' \; -print | sort)
    if [[ -n "$unfinalized" ]]; then
        echo
        echo "These have no metadata.yaml -- still recording, or need reindex:"
        echo "$unfinalized" | sed 's/^/  /'
        echo "  (ros2 bag reindex <dir>, then re-run this script)"
    fi
    exit 0
fi

echo "Source:      $SRC"
echo "Destination: $DEST"
echo "Bags:        ${#BAGS[@]}"
du -sh "${BAGS[@]}" 2>/dev/null | sed 's/^/  /'
echo

RSYNC_OPTS=(
    --archive               # keep mtimes; bag timestamps are the only ordering
    --partial --progress    # a dropped wifi association resumes instead of restarting
    --human-readable
)
[[ $DRY_RUN -eq 1 ]] && RSYNC_OPTS+=(--dry-run)

rsync "${RSYNC_OPTS[@]}" "${BAGS[@]}" "$DEST"

if [[ $DRY_RUN -eq 1 ]]; then
    echo
    echo "Dry run only -- nothing was transferred."
    exit 0
fi

echo
echo "Transfer complete."

if [[ $DELETE_LOCAL -eq 1 ]]; then
    # Second pass with --checksum, ignoring size/mtime shortcuts. If this
    # reports any file needing transfer, the copy is not trustworthy and
    # nothing gets deleted. Cheap insurance against freeing the only copy.
    echo
    echo "Verifying by checksum before deleting anything locally..."
    remaining=$(rsync --archive --checksum --dry-run --itemize-changes \
        "${BAGS[@]}" "$DEST" | grep -E '^[<>ch]' || true)

    if [[ -n "$remaining" ]]; then
        echo "VERIFY FAILED -- these differ at the destination:" >&2
        echo "$remaining" | sed 's/^/  /' >&2
        echo "Nothing deleted locally. Re-run the transfer." >&2
        exit 1
    fi

    echo "Verified. Deleting local copies:"
    for bag in "${BAGS[@]}"; do
        echo "  rm -rf $bag"
        rm -rf "$bag"
    done
    echo
    df -h "$SRC" | tail -1
fi
