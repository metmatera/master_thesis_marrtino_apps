# MARRtino firmware parameters configuration #

NOTE: Instructions have been updated for 2018 firmware. 
If you have 2017 firmware, please use the scripts named with 2017
or refer to previous checkout ```19c4c221f81e2414651aceeba900ca3a0c66988e``` of this repository.

## Scripted version ##


* to upload firmware parameters from file to Arduino


        cat upload_config.script | rosrun srrg2_orazio_core orazio -serial-device /dev/orazio 


* to download firmware parameters from Arduino to file


        cat download_config.script | rosrun srrg2_orazio_core orazio -serial-device /dev/orazio 



Note: change file names in the .script files if needed.



## Manual version ##


    cd ~/config
    rosrun srrg2_orazio_core orazio -serial-device /dev/orazio

    orazio> load_config XXX_firmware_params.cfg
    orazio> send system_params
    orazio> send joint_params[0]
    orazio> send joint_params[1]
    orazio> send drive_params
    orazio> store system_params
    orazio> store joint_params[0]
    orazio> store joint_params[1]
    orazio> store drive_params
    orazio> quit





