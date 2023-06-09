# Carrus Unicornuum
World Robot Olympiad 2023 Future Engineers

## Ordnerstruktur
- Im Ordner `3D-Modelle` befindet sich eine .step-Datei, in der alle 3D-Modelle von 3D-gedruckten Komponenten des Roboters zu finden sind.
- Im Ordner `Programmcode` befindet sich der Code, der vom Roboter zum autonomen Fahren verwendet wird, sowie anderer Code, der zur Umsetzung benötigt wird.
- Im Ordner `Roboterauto` befinden sich sechs Fotos des Roboters, die ihn von allen Seiten zeigen.
- Im Ordner `Schaltpläne` befindet sich ein Schaltplan, auf dem zu erkenn ist, wie die einzelnen Komponenten miteinander verbunden sind.
- Im Ordner `Technische Zeichnungen` befinden sich technische Zeichnungen von 3D-gedruckten Komponenten des Roboterautos.
- Im Ordner `Video` befinden sich zwei Links zu YouTube-Videos, die den Roboter bei der autonomen Fahrt sowohl beim Eröffnungs- als auch beim Hindernisrennen zeigen.


## Hardware
### Motorisierung
Der Antrieb des Roboterautos geschieht durch einen DC-Motor. Dieser ist über ein Getriebe mit der Hinterachse und somit mit den Rädern verbunden. Die Lenkung geschieht unabhängig vom Antrieb mit Hilfe eines Servo-Motors des Typs HS-81MG und läuft über die Vorderachse. Alle Motoren sind mit einem Motortreiber L298N verbunden. Dieser regelt die Stromversorgung der Motoren.

### Energie
Die Energieversorgung des Roboterautos geschieht über zwei Einheiten. Zum einen wird der Motortreiber von einer Batterie-Box mit 8 Batterien des Typs AA betrieben. Der Raspberry Pi wird hingegen von einer Powerbank (5V, 3A out; ca. 10.000 mAh) mit Strom versorgt.

### Sensoren
Das Roboter-Auto verfügt neben einer Kamera mit Weitwinkellinse auch über 3 Ultraschallsensoren (HC-SR04), welche dazu dienen, den Abstand nach vorne und zu den Wänden zu bemessen. Somit kann die Lage des Roboterautos innerhalb des Parcours ermittelt werden. Die Kamera hingegen bestimmt den Abstand und die Farbe von Hindernissen.  


## Software
### Eröffnungsrennen
#### Kurven
![Flussdiagramm zum Kurvenfahren](https://github.com/iiiiich/Carrus-Unicornuum/assets/128396963/cc4d4d2d-dada-4c40-b822-a638dc890044)
#### Korrekturen
![Flussdiagramm zum Korrigieren](https://github.com/iiiiich/Carrus-Unicornuum/assets/128396963/df0b16e8-3673-497b-8bff-4ec701958d3b)

### Hindernisrennen
![Flussdiagramm zur Hindernisumfahrung](https://github.com/iiiiich/Carrus-Unicornuum/assets/128396963/85abef02-4d8c-4d84-8352-cbd545168401)
