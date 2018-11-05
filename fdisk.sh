#!/bin/sh
#This file only needs to be run on a new board and only once in the factory 
#then it is unuseful.
#20180816

mkdosfs -F 32 /dev/mmcblk0p3	#mpp
echo "mkdosfs mmcblk0p3 mpp OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p4	#usr_lib
echo "mkdosfs mmcblk0p4 usr_lib OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p5	#usr_app
echo "mkdosfs mmcblk0p5 usr_app OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p6	#usr_data1
echo "mkdosfs mmcblk0p6 usr_data1 OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p7	#usr_data2
echo "mkdosfs mmcblk0p7 usr_data2 OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p10	#mpp_bk
echo "mkdosfs mmcblk0p10 mpp_bk OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p11	#usr_lib_bk
echo "mkdosfs mmcblk0p11 usr_lib_bk OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p12	#usr_app_bk
echo "mkdosfs mmcblk0p12 usr_app_bk OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p13	#usr_data1_bk
echo "mkdosfs mmcblk0p13 usr_data1_bk OK"
echo ""
mkdosfs -F 32 /dev/mmcblk0p14	#usr_data2_bk
echo "mkdosfs mmcblk0p14 usr_data2_bk OK"
echo ""