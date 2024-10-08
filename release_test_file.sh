#record sdk version
SDK_VERSION=0.2
echo "Telink BLE SDK_VERSION = $SDK_VERSION" > ./sdk_version.txt

dir_release=../../SIG_MESH_test_file
sdk_dir=$dir_release/bin
tools_dir=$dir_release/tools

rm -rf $dir_release
mkdir $dir_release
mkdir $sdk_dir
mkdir $tools_dir
cp -rf ../reference/tl_bulk/release/sig_mesh_tool.exe  $tools_dir/
cp -rf ../reference/tl_bulk/sig_mesh_master.ini  $tools_dir/
cp -rf ../reference/tl_bulk/tl_node_gateway.ini  $tools_dir/
cp -rf ../ble_lt_mesh/8269_mesh/8269_mesh.bin  $sdk_dir/
cp -rf ../ble_lt_mesh/8269_mesh_gw/8269_mesh_gw.bin  $sdk_dir/
cp -rf ../ble_lt_mesh/8269_mesh_LPN/8269_mesh_LPN.bin  $sdk_dir/
cp -rf ../ble_lt_mesh/sig_mesh_master_dongle_8267_69/sig_mesh_master_dongle_8267_69.bin  $sdk_dir/

rm -rf $sdk_dir/*.IAB
rm -rf $sdk_dir/*.IAD
rm -rf $sdk_dir/*.IMB
rm -rf $sdk_dir/*.IMD
rm -rf $sdk_dir/*.PFI
rm -rf $sdk_dir/*.PO
rm -rf $sdk_dir/*.PR
rm -rf $sdk_dir/*.PRI
rm -rf $sdk_dir/*.PS
rm -rf $sdk_dir/*.SearchResults
rm -rf $sdk_dir/*.WK3
cd $dir_release
find . $dir_release/ble_lt_mesh/ -name "*.c" -maxdepth 10 | xargs rm  
find . $dir_release/ble_lt_mesh/ -name "*.link" -maxdepth 10 | xargs rm 
find . $dir_release/ble_lt_mesh/ -name "*.txt" -maxdepth 10 | xargs rm 
find . $dir_release/ble_lt_mesh/ -name "*.S" -maxdepth 10 | xargs rm 
find . $dir_release/ble_lt_mesh/ -name *.py" -maxdepth 10 | xargs rm 
find . $dir_release/ble_lt_mesh/ -name "*.bak" -maxdepth 10 | xargs rm 
find . $dir_release/ble_lt_mesh/ -name "*.pl" -maxdepth 10 | xargs rm
find . $dir_release/ble_lt_mesh/ -name "*project" -maxdepth 10 | xargs rm
find . $dir_release/ble_lt_mesh/ -name "*.a" -maxdepth 10 | xargs rm
pause
::exit