#/mpp/mpp/ko30-0604/load185-512

mount -t ext4 /dev/mmcblk0p4 /usr_lib
mount /dev/mmcblk0p5 /usr_data1

#mount -t ext4 /dev/mmcblk0p12 /usr_app
#mount -t ext4 /dev/mmcblk0p13 /usr_data1

#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/mpp 
#LD_LIBRARY_PATH="/usr/local/lib:/usr/lib:/root/mpp/lib"

#source /etc/profile LD_LIBRARY_PATH=/usr/local/lib:/usr/lib:/root/mpp/lib:/mpp
cd /mpp/mpp/ko30-0604/ && ./load185-512



#mount -t ext4 /dev/mmcblk0p13 /usr_data1
#mount -t ext4 /dev/mmcblk0p14 /usr_data2


