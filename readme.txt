1. Plugin the SD card, which contains the prebuilt system.

2. Install sensor about 2.5 - 3.0 meter high above.

3. Plugin sensor, and plugin network cable (must have internet access).

4. Power up

5. To preview, open a terminal, run command to verify if sensor works fine and adjust sensor position:

   sudo ./peoplecount view 1000 100 20 2
   
   then press ESC to quit

4. Before starting the application, make sure the scene is clear and no person visible by the sensor.

5. Run command to start application:

   sudo ./peoplecount train-record 1000 100 20 2 /home/cat/data
   
   Write down the device ID, will be used to access result from IoT.
   
   Watch the person pass through, and observe the counting result from the terminal.

6. Open iot_demo.html (refer to the screenshot) from a PC which connected to internet.

7. Enter the device ID obtained from step 5 into the device field, and click Subscribe button

8. Observe the counting result.

9. Ctrl + C to shutdown.

   Recorded data in folder /home/cat/data
   
   
