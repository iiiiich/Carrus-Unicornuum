# Carrus Unicornuum
World Robot Olympiad 2023 Future Engineers

## Hardware
### Motorisierung
Der Antrieb des Roboter-Autos geschieht durch einen DC-Motor. Dieser ist über ein Getriebe mit der Hinterachse und somit mit den Rädern verbunden. Die Lenkung geschieht unabhängig vom Antrieb, mit Hilfe eines Servo-Motors des Typs HS-81MG und läuft über die Vorderachse. Alle Motoren sind mit einem Motortreiber L298N verbunden. Dieser regelt die Stromversorgung der Motoren.

### Energie
Die Energieversorgung des Roboters-Autos geschieht über zwei Einheiten. Zum einen wird der Motortreiber von einer Batterie-Box mit 8 Batterien des Typs AA betrieben. Der Raspberry Pi wir hingegen von einer Powerbank (5V; 3A out; ca. 10000 mAh) mit Strom versorgt

### Sensoren
Das Roboter-Auto verfügt neben einer Kamera  mit Weitwinkellinse auch über 3 Ultraschallsensoren (HC-SR04), welche dazu dienen, den Abstand nach vorne und zu den Wänden zu bemessen. Somit kann die Lage des Roboter-Autos innerhalb des Parcours ermittelt werden. Die Kamera hingegen bestimmt den Abstand und die Farbe von Hindernissen.  


## Software
### Eröffnungsrennen

### Hindernisrennen
![Flussdiagramm zur Hindernisumfahrung](https://user-images.githubusercontent.com/128396963/236519192-251315be-2b41-4ad9-8319-2199691f5c98.jpeg)
