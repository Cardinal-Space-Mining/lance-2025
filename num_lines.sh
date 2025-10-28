#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

wc -l \
    $SCRIPTPATH/{cardinal-perception,csm-metrics,csm-sim,lance,launch-utils}/{*/*,*/*/*,*/*/*/*}.{cpp,hpp,py} \
    $SCRIPTPATH/{multiscan-driver,phoenix-driver}/*/*.{cpp,hpp,py}
