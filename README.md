# Carrus Unicornuum
World Robot Olympiad 2023 Future Engineers

## Ordnerstruktur
- Im Ordner `Programmcode` befindet sich der Code, der vom Roboter zum autonomen Fahren verwendet wird, sowie anderer Code, der zur Umsetzung benötigt wird.
- Im Ordner `Roboterauto` befinden sich sechs Fotos des Roboters, die ihn von allen Seiten zeigen.
- Im Ordner `Schaltpläne` befindet sich ein Schaltplan, auf dem zu erkenn ist, wie die einzelnen Komponenten miteinander verbunden sind.
- Im Ordner `Technische Zeichnungen` befinden sich technische Zeichnungen von 3D-gedruckten Komponenten des Roboterautos.
- Im Ordner `3D-Modelle` befindet sich eine .step-Datei, in der alle 3D-Modelle von 3D-gedruckten Komponenten des Roboters zu finden sind.
- Im Ordner `Video` befinden sich zwei Links zu YouTube-Videos, die den Roboter bei der autonomen Fahrt sowohl beim Eröffnungs- als auch beim Hindernisrennen zeigen.

## Hardware
### Motorisierung
Der Antrieb des Roboter-Autos geschieht durch einen DC-Motor. Dieser ist über ein Getriebe mit der Hinterachse und somit mit den Rädern verbunden. Die Lenkung geschieht unabhängig vom Antrieb, mit Hilfe eines Servo-Motors des Typs HS-81MG und läuft über die Vorderachse. Alle Motoren sind mit einem Motortreiber L298N verbunden. Dieser regelt die Stromversorgung der Motoren.

### Energie
Die Energieversorgung des Roboters-Autos geschieht über zwei Einheiten. Zum einen wird der Motortreiber von einer Batterie-Box mit 8 Batterien des Typs AA betrieben. Der Raspberry Pi wir hingegen von einer Powerbank (5V; 3A out; ca. 10000 mAh) mit Strom versorgt

### Sensoren
Das Roboter-Auto verfügt neben einer Kamera  mit Weitwinkellinse
 auch über 3 Ultraschallsensoren (HC-SR04), welche dazu dienen, den Abstand nach vorne und zu den Wänden zu bemessen. Somit kann die Lage des Roboter-Autos innerhalb des Parcours ermittelt werden. Die Kamera hingegen bestimmt den Abstand und die Farbe von Hindernissen.  


## Software

### Eröffnungsrennen
#### Kurven
![Flussdiagramm zum Kurvenfahren](https://user-images.githubusercontent.com/128396963/236560181-bcf6c7e0-176b-4ac8-8dcf-6b0db0ab8153.jpeg)
#### Korrekturen
![Flussdiagramm zum Korrigieren](https://user-images.githubusercontent.com/128396963/236562212-91d419c3-25d5-4b6d-8efb-cccdcb9835c8.jpeg)

### Hindernisrennen
![Flussdiagramm zur Hindernisumfahrung](https://user-images.githubusercontent.com/128396963/236519192-251315be-2b41-4ad9-8319-2199691f5c98.jpeg)
