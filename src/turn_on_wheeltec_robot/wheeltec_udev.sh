echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", GROUP:="dialout", SYMLINK+="imu_sensor"' >/etc/udev/rules.d/imu_sensor.rules

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", ATTRS{serial}=="", MODE:="0777", GROUP:="dialout", SYMLINK+="imu_sensor"' >/etc/udev/rules.d/imu_sensor.rules
001 Device 009: ID 067b:2303 

service udev reload
sleep 2
service udev restart


