# Automated-Headlight
Automated Headlight System Prototype using IoT
Hardware and connections are as follows:
Arduino Nano
DHT 22 connected on D13
LDR digital read on D8
GY521/MPU6050 connected via A4 and A5 for I2C communication
2-Channel relay with low voltage actuation has input via D9 and D10.
1st channel of relay has 12V common on common
                                        Common of Headlight on NO(normally Open)
2nd relay has 12V Live on Common
                       Headlight low beam on NC(Normally Closed)
                       Headlight High beam on NO(Normally Open)
