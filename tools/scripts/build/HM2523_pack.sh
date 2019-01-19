#/bin/bash

source_dir=$(cd $1; pwd)
out_dir=$2
proj_name=$3

proj_name_str=$(grep "proj_name_str" $out_dir/obj/middleware/MTK/verno/verno.c | cut -d"\"" -f 2)
cust_code_str=$(grep "cust_code_str" $out_dir/obj/middleware/MTK/verno/verno.c | cut -d"\"" -f 2)
sw_verno_str=$(grep "sw_verno_str" $out_dir/obj/middleware/MTK/verno/verno.c | cut -d"\"" -f 2)
gnss_verno_str=$(grep "gnss_verno_str" $out_dir/obj/middleware/MTK/verno/verno.c | cut -d"\"" -f 2)
build_mon_and_day_str=$(grep "build_mon_and_day_str" $out_dir/obj/middleware/MTK/verno/verno.c | cut -d"\"" -f 2)
prod_name_str=$(grep "prod_name_str" $out_dir/obj/middleware/MTK/verno/verno.c | cut -d"\"" -f 2)

ver_str="$proj_name_str"_"$cust_code_str"_"$sw_verno_str"_"$gnss_verno_str"_"$build_mon_and_day_str"_"$prod_name_str"

rm -rf $source_dir/version_pack

mkdir -p $source_dir/version_pack/$ver_str

mkdir -p $source_dir/version_pack/保留文件

cp $out_dir/flash_download.cfg $source_dir/version_pack/$ver_str/
cp $out_dir/gnss_firmware.bin $source_dir/version_pack/$ver_str/
cp $out_dir/flash.bin $source_dir/version_pack/$ver_str/

## Just copy to out dir.
#cp -rf $source_dir/tools/version_pack/flash_download_develop.cfg $out_dir/
#rm -rf  $out_dir/system.bin 
#unzip $source_dir/FOTA/$ver_str.zip system.bin -d $out_dir/
#cp -rf $source_dir/tools/version_pack/trigger_flag $out_dir/

#mkdir $source_dir/version_pack/$ver_str/development
#cp $source_dir/tools/version_pack/flash_download_develop.cfg $source_dir/version_pack/$ver_str/development/
#unzip $source_dir/FOTA/$ver_str.zip system.bin -d $source_dir/version_pack/$ver_str/development/
##cp $out_dir/mt2523_bootloader.bin $source_dir/version_pack/$ver_str/development/
#cp $source_dir/tools/version_pack/trigger_flag $source_dir/version_pack/$ver_str/development/
#cp $out_dir/gnss_firmware.bin $source_dir/version_pack/$ver_str/development/

cp $out_dir/$proj_name.bin $source_dir/version_pack/保留文件
cp $out_dir/$proj_name.elf $source_dir/version_pack/保留文件

cd $source_dir/version_pack/
zip -r ./$ver_str.zip $ver_str
zip -r ./保留文件.zip 保留文件

rm -rf $ver_str 保留文件

cd - > /dev/null

