# Nexigo_Mando_androidapp_edgedevice_code-vishnu-
A bluetooth app which will connect to edge esp32 device via bluetooth which will sends the data of both battery and motor controller collecting through CAN(RTOS)

This app will show the Battery, Motor Controller Data of Nexigo battery pack with the CAN Matrix-BMS CAN Matrix.xlsx.

You need to use 2 ESP32's since the battery is in command mode.

->One device will send a CAN command to battery such a way that battery can send the CAN frames.

->Other is used the read the CAN frames to do byte operations and write the data to the bluetooth such a way that a Mobile can be paired to receive the data. 

Upload CAN_write_ithin_sucessfullllllllllllll.ino in one device(sending the command).
Upload test_4.ino in other device(Capturing the battery response, and controller do byte operations, synchronizing the data of both battery and motor controller and write the data to bluetooth.
Battery, Motor controller, Device 1 and Device 2 all three has to be connected via CAN.
You can install the apk, pair ESP_Control bluetooth device(As mentioned in edge device firmware, you can change it).
Connect to the device then you will see the data that will be streaming to Mobile phone.
Battery_Motor_portrait_update_button_1 (1).aia can be imported into MIT app inventor to see the underlying logics, if you want you can do changes for the logics and UI.

---------------------------------------------------------------------------:)))--------------------------------------------------------------------------------
