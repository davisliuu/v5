原始文件：
	testmmc_mpp003-5_1.xml
	u-boot-001.bin
	uImage-mpp003_5_1_10
开发文件：
	uImage-37：用内核原始配置文件编译的镜像，可以运行sensor例程
	uImage-37-2：用自己配置文件
		去掉如下配置：
		# CONFIG_EXT2_FS is not set
		# CONFIG_EXT3_FS is not set
		添加如下配置
		CONFIG_EXT4_USE_FOR_EXT23=y
		CONFIG_NFS_FS=y  //NFS client support
		CONFIG_NFS_V2=y
		CONFIG_NFS_V3=y
		CONFIG_NFS_V3_ACL=y
	uImage-38:
		同上，在板子上运行fdisk.sh之后，开机可以自动挂载mpp分区
