#!/bin/bash

# show usage message
usage () {
    echo "Usage: $0 <template_file> <output_file> <sw_verno> <hw_verno>"
    echo ""
    echo "Example:"
    echo "       $0 verno.template verno.c SDK_V3.3.2 mt2523_hdk"
}

# set exit immediately if a command exits with a non-zero status.
set -e

# read arguments
template_file=$1
dest_file=$2
proj_name=$3
cust_code=$4
sw_verno=$5
gnss_verno=$6
prod_name=$7
hw_verno=$8
date_time=$(date +"%Y/%m/%d %H:%M:%S   GMT %:z")
mon_day=$(date +"%m%d")

# check arguments
if [ -z "$1" ]; then usage; exit 1; fi
if [ -z "$2" ]; then usage; exit 1; fi
if [ -z "$3" ]; then usage; exit 1; fi
if [ -z "$4" ]; then usage; exit 1; fi
if [ -z "$5" ]; then usage; exit 1; fi
if [ -z "$6" ]; then usage; exit 1; fi
if [ -z "$7" ]; then usage; exit 1; fi
if [ -z "$8" ]; then usage; exit 1; fi
if [ ! -z "$MTK_VERNO_DATE_TIME" ]; then date_time=$MTK_VERNO_DATE_TIME; fi

# replace string.
sed  "s|\$DATE|${date_time}|g; \
	s|\$MON_DAY|${mon_day}|g; \
	s|\$PROJ_NAME|${proj_name}|g; \
	s|\$CUST_CODE|${cust_code}|g; \
	s|\$SW_VERNO|${sw_verno}|g; \
	s|\$GNSS_VERNO|${gnss_verno}|g; \
	s|\$PROD_NAME|${prod_name}|g; \
	s|\$HW_VERNO|${hw_verno}|g" $template_file > $dest_file

