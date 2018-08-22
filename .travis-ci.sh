#!/bin/bash

# This script is triggered from the script section of .travis.yml
# It runs the appropriate commands depending on the task requested.

set -e

SPELLINGBLACKLIST=$(cat <<-BLACKLIST
      -wholename "./.codespellignore" -or \
      -wholename "./.git/*"
BLACKLIST
)

if [[ $TASK = 'codespell' ]]; then
  # run codespell only if it is the requested task
  # count the number of codespell errors
  spellingerrors=$(zrun codespell --check-filenames --check-hidden --quiet 2 --regex "[a-zA-Z0-9][\\-'a-zA-Z0-9]+[a-zA-Z0-9]" $spellingfiles 2>&1 | wc -l)
  if [[ $spellingerrors -ne 0 ]]; then
    # print the output for info
    zrun codespell --check-filenames --check-hidden --quiet 2 --regex "[a-zA-Z0-9][\\-'a-zA-Z0-9]+[a-zA-Z0-9]" $spellingfiles
    echo "Found $spellingerrors spelling errors via codespell"
    exit 1;
  else
    echo "Found $spellingerrors spelling errors via codespell"
  fi;
else
  # Otherwise compile and check as normal
  platformio ci --lib="." --board=$BOARD
fi
