#!/bin/sh

for pcl in "$@"; do
    awk '

    BEGIN {p=0; n=0}
    /^VIEWPOINT/ {x=$2; y=$3; z=$4}
    {if(p) {print; n++}}
    /^DATA ascii/ {p=1; print "NODE",x,y,z,0,0,0}
    
    ' "$pcl"

    e=$?

    if [ $e -ne 0 -a $e -ne 141 ]; then
        echo "error: $pcl is not in ascii format ($e)" 1>&2
        exit 1
    fi
done
