Accurately adjusts gimbal using an accelerometer/gyroscope.
Directly accesses MPU6050's registers to record data and establish a connection with I2C bus connected to the Arduino board.
Moving the sensor in the upward or downward direction gives values from -16000 to 16000. These are mapped from 0 to 180 to move the servo motor.
