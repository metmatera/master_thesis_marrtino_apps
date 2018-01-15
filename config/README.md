# MARRtino firmware parameters configuration #

## Script version ##

1) to upload firmware parameters from file to Arduino

```
cat upload_config.script | rosrun srrg2_orazio_core orazio -serial-device /dev/orazio 
```

2) to download firmware parameters from Arduino to file

```
cat download_config.script | rosrun srrg2_orazio_core orazio -serial-device /dev/orazio 
```


Note: change file names in the .script files if needed.



## Manual version ##

```
$ cd ~/config
$ run_orazio_server.bash
[press Enter]
orazio> load_config marrtino2017_firmware_params.cfg
orazio> send system_params
orazio> send joint_params
orazio> send drive_params
orazio> store system_params
orazio> store joint_params
orazio> store drive_params
orazio> quit
```




