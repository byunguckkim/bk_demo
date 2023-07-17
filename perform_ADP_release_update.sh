#!/bin/bash

set -eu

# Use this script to keep your ADP protos and libraries up-to-date each release!
function print_usage() {
	echo
	echo "Usage: ./perform_ADP_release_update.sh [required-args] [optional-args]"
  echo "[required-args]"
	echo "-r | --api_release_zip_path : Path to public_API_release_<interface_architecture>_<ADP_version>.zip in the latest downloaded release."
	echo "-a | --applied_path : Path to your local applied folder."
  echo "[optional-args]"
	echo "-g | --replace_glue_stack_base : Overwrite existing versions of your interface glue and stack base files. Note that this should only be used in specific cases when accessing a new feature requires upgrading to a newer interface version."
	echo
	echo "Use this script to keep your ADP protos and libraries up-to-date every release!"
}

# Parse and validate options
PATH_TO_APPLIED=""
PATH_TO_API_RELEASE_ZIP=""
OVERWRITE_GLUE_AND_STACK_BASE=0
while [ $# -gt 0 ]
do
    case "$1" in
    -a | --applied_path)
      shift
      PATH_TO_APPLIED=$1
      ;;
    -r | --api_release_zip_path)
      shift
      PATH_TO_API_RELEASE_ZIP=$1
      ;;
    -g | --replace_glue_stack_base)
      OVERWRITE_GLUE_AND_STACK_BASE=1
      ;;
    *)    # unknown option
      print_usage
      exit 1
      ;;
    esac
    shift
done

if [ -z "$PATH_TO_APPLIED" ]; then
  echo 'Detected a missing option: -a.' >&2
  print_usage
  exit 1
elif [ -z "$PATH_TO_API_RELEASE_ZIP" ]; then
  echo 'Detected a missing option: -r.' >&2
  print_usage
  exit 1
fi

# Remove the previous applied/ folder and unzip the latest one.
if [ "$OVERWRITE_GLUE_AND_STACK_BASE" == 1 ]; then
  echo 'Overwriting existing versions of glue and stack base files...'
  rm -r $PATH_TO_APPLIED
  unzip $PATH_TO_API_RELEASE_ZIP -d "${PATH_TO_APPLIED%applied*}"
else
  echo 'Preserving existing versions of glue and stack base files...'
  find $PATH_TO_APPLIED ! -name "customer_interface_glue.cc" ! -name "stack_interface_v*.h" ! -name "customer_stack_base.*" -type f -exec rm -r {} +
  unzip -n $PATH_TO_API_RELEASE_ZIP -d "${PATH_TO_APPLIED%applied*}"
fi