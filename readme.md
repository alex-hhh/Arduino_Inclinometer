# Arduino Based Inclinometer

The inclinometer is a device that displays roll and pitch inside a vehicle.
It does that by reading the acceleration vector from an ADXL345 accelerometer
module and displaying it on an OLED display.

Unlike other inclinometers which have to be installed level inside the vehicle
and facing forward, this program allows the accelerometer module to be
installed in any position inside the vehicle, on startup it will determine its
orientation automatically.
