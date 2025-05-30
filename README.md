# Robotik Projekt 2024/25

Dieses Repository enthält die Lösung für die Übungs- und Belegaufgaben aus dem Modul **Robotik Projekt**. Implementiert werden ROS2-Packages in C++ und Python. 

---

## Aufgabe 1: Publisher (C++) und Subscriber (Python)

### Funktionalität
- **Publisher** ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/cpp_pub/src/publisher_member_function.cpp)):
  - Sendet Integer-Werte auf das Topic `number`.
  - Der Wert erhöht sich bei jeder Nachricht um `1`.
  - Veröffentlichungsintervall: 1 Sekunde.
  
- **Subscriber** ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/py_sub/py_sub/subscriber_member_function.py)):
  - Empfängt die vom Publisher gesendeten Werte.
  - Berechnet die zeitliche Differenz zwischen zwei aufeinanderfolgenden Nachrichten.
  - Veröffentlicht die berechnete Differenz auf dem Topic `diff`.

### Nutzung

#### Vorbereitung
1. **Abhängigkeiten installieren**:
    - ROS2 Humble ([Installationsanleitung](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
    - Colcon:
      ```bash
      sudo apt install python3-colcon-common-extensions
      ```
    - Pakete mit `colcon` bauen:
      ```bash
      colcon build
      ```
2. **Workspace erstellen**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/smn-hrtzsch/robotik_projekt.git
   cd ~/ros2_ws
   colcon build
   source ~/ros2_ws/install/local_setup.bash
   ```

#### Publisher starten
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run cpp_pub publisher
  ```

#### Subscriber starten
In einem neuen Terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run py_sub subscriber
```

#### Funktionalität testen
In einem neuen Terminal:
- Daten des Publishers anzeigen:
  ```bash
  ros2 topic echo /number
  ```
In einem neuen Terminal:
- Zeitdifferenzen des Subscribers überprüfen:
  ```bash
  ros2 topic echo /diff
  ```

---

## Aufgabe 2 Hindernisverfolgung mittels Laserscanner (Python)

### Funktionalität ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/following_obstacle/following_obstacle/laserscanner.py))
- Implementiert eine ROS2-Node, die den nächsten Hindernissen innerhalb eines einstellbaren Bereichs folgt.
- **Features:**
  - Hindernisse werden im konfigurierbaren Winkelbereich und Abstand erkannt.
  - Der Roboter stoppt, wenn er zu nahe an ein Hindernis herankommt.
  - Der Roboter verfolgt Hindernisse in vorgegebenen Abständen und lenkt entsprechend, um ihnen zu folgen.

### Parameter
Die Hindernisverfolgung unterstützt mehrere konfigurierbare Parameter, die zur Laufzeit geändert werden können:
- **`distance_to_stop`**: Minimale Entfernung zum Hindernis, bei der der Roboter stoppt (Standard: `0.3 m`).
- **`distance_to_follow`**: Maximale Entfernung, innerhalb derer der Roboter einem Hindernis folgt (Standard: `1.0 m`).
- **`speed_linear`**: Maximale Vorwärtsgeschwindigkeit des Roboters (Standard: `0.15 m/s`).
- **`speed_angular`**: Maximale Drehgeschwindigkeit des Roboters (Standard: `0.5 rad/s`).
- **`scan_angle_range`**: Winkelbereich (in Grad), in dem Hindernisse erkannt werden (Standard: `90°`).

### Nutzung

#### Vorbereitung
1. **Abhängigkeiten installieren**:
   Die Vorbereitung ist identisch mit Aufgabe 1. Die Schritte zur Einrichtung des Workspaces und des Repositories bleiben gleich.

2. **Node kompilieren und starten**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run following_obstacle follow_obstacle
   ```

#### Parameter ändern
Die Parameter können während der Laufzeit über die ROS2 CLI geändert werden.
Dazu kann man ein neues Terminal öffnen und dann mit folgenden Befehlen arbeiten:
```bash
ros2 param set /follow_obstacle scan_angle_range 60
ros2 param set /follow_obstacle distance_to_stop 0.5
```

### Funktionalität testen
1. **Laserscandaten visualisieren**:
   ```bash
   ros2 topic echo /scan
   ```

2. **Bewegungsbefehle überprüfen**:
   ```bash
   ros2 topic echo /cmd_vel
   ```

---

## Aufgabe 3: Linienverfolgung mittels Kamera (Python)

### Funktionalität
- **Focused Line Follower Node** ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/line_follower/line_follower/line_follower.py)):
  - Verarbeitet Live-Bilddaten von einer Kamera, um eine weiße Linie auf dem Boden zu erkennen und zu verfolgen.
  - Berechnet die Position der Linie im Bild und passt die Bewegungsbefehle des Roboters an, um der Linie zu folgen.
  - Verwendet OpenCV für Bildverarbeitung und ROS2 für die Kommunikation.
  - Debug-Ansichten der Bildverarbeitung (Graustufen- und binarisiertes Bild) können in OpenCV angezeigt werden.

- **Stop Node** ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/line_follower/line_follower/stop.py)):
  - Sendet mehrfach Bewegungsbefehle, um den Roboter sicher zu stoppen.
  - Ermöglicht ein koordiniertes Herunterfahren nach einem Keyboard-Interrupt.

### Parameter der Focused Line Follower Node
Die Node unterstützt mehrere Parameter, die zur Laufzeit angepasst werden können:
- **`speed_drive`**: Maximale Geschwindigkeit für Geradeausfahren (Standard: `0.1 m/s`).
- **`speed_turn`**: Maximale Geschwindigkeit für Drehbewegungen (Standard: `0.2 rad/s`).
- **`threshold`**: Schwellenwert für die Binarisierung des Bildes (Standard: `150`).

### Nutzung

#### Vorbereitung

Die Vorbereitung ist identisch mit Aufgabe 1. Die Schritte zur Einrichtung des Workspaces und des Repositories bleiben gleich.

Um die Kamerabilder anzeigen zu lassen, während die Node läuft, müssen folgende Pakete und Abhängigkeiten installiert sein:

1. OpenCV und Abhängigkeiten installieren:

   Unter Ubuntu (ROS2 Humble empfohlen):
   ```bash
   sudo apt update
   sudo apt install python3-opencv libopencv-dev
   ```

3. GUI-Unterstützung sicherstellen:

   Je nach verwendetem GUI-Backend sollte eines der folgenden Pakete installiert werden:
   ```bash
   sudo apt install libqt5gui5  # Qt als Backend
   ```
   oder
   ```bash
   sudo apt install libgtk-3-dev  # GTK als Backend
   ```

3. Python-Abhängigkeiten mit `pip` installieren:

   Falls `cv_bridge` oder andere ROS-spezifische Python-Pakete fehlen:
   ```bash
   pip install opencv-python opencv-contrib-python
   pip install cv-bridge
   ```

4. Zusätzliche ROS-Pakete:

   Für die volle Integration von `cv_bridge` in ROS2:
   ```bash
   sudo apt install ros-humble-cv-bridge
   ```

#### Nodes starten

Die Nodes können über das bereitgestellte Launch-File gemeinsam gestartet werden:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch line_follower line_follower_launch.py
```

#### Parameter ändern
Die Parameter können während der Laufzeit über die ROS2 CLI angepasst werden. 
Dazu kann man ein neues Terminal öffnen und dann mit folgenden Befehlen arbeiten:
```bash
ros2 param set /focused_line_follower threshold 160
ros2 param set /focused_line_follower speed_drive 0.2
```

#### Funktionalität testen
1. **Debug-Ansichten der Bildverarbeitung**:
   - Während die Node läuft, werden Graustufenbilder und binarisierte Bilder in OpenCV-Fenstern angezeigt.
2. **Bewegungsbefehle prüfen**:
   - Veröffentlicht auf dem Topic `/cmd_vel`:
     ```bash
     ros2 topic echo /cmd_vel
     ```

---

# Aufgabe 4: Linienverfolgung zwischen zwei Hindernissen (Python)

### Funktionalität

- **Handle Obstacle Node** ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/line_follower_bo/line_follower_bo/handle_obstacle.py)):

  - Erkennt Hindernisse mit einem Laserscanner und stoppt den Roboter, wenn ein Hindernis erkannt wird.
  - Führt Drehbewegungen durch, um Hindernisse zu umgehen, und ermöglicht eine sichere Fortsetzung der Linienverfolgung.
  - Arbeitet mit dem State Manager zusammen, um Zustände wie "ROTATE" oder "FOLLOW_LINE" zu verwalten.

- **Line Follower Node** ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/line_follower_bo/line_follower_bo/line_follower.py)):

  - komplett gleiche Funktionalität wie in Aufgabe 3

- **State Manager** ([Code ansehen](https://github.com/smn-hrtzsch/robotik_projekt/blob/main/src/line_follower_bo/line_follower_bo/state_manager.py)):

  - Verarbeitet und verwaltet die Zustände des Roboters, wie "FOLLOW_LINE" oder "ROTATE".
  - Leitet Bewegungsbefehle entsprechend des aktuellen Zustands weiter.

### Parameter der Handle Obstacle Node

- **`distance_to_stop`**: Minimaler Abstand zum Hindernis, bei dem der Roboter anhält (Standard: `0.3 m`).
- **`rotate_speed`**: Drehgeschwindigkeit während der 180° Drehung am Hindernis (Standard: `0.75 rad/s`).
- **`scan_angle_range`**: Winkelbereich für die Hinderniserkennung (Standard: `30°`).

### Nutzung

#### Vorbereitung

Die Vorbereitung ist identisch mit Aufgabe 1. Die Schritte zur Einrichtung des Workspaces und des Repositories bleiben gleich.

Um die Kamerabilder anzeigen zu lassen, während die Node läuft, müssen gewisse Pakete und Abhängigkeiten installiert sein, siehe Vorbereitung Aufgabe 3.

#### Nodes starten

Die Nodes können über das bereitgestellte Launch-File gemeinsam gestartet werden:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch line_follower_bo line_follower_bo_launch.py
```

#### Parameter ändern

Die Parameter können während der Laufzeit über die ROS2 CLI angepasst werden:

```bash
ros2 param set /handle_obstacle distance_to_stop 0.5
ros2 param set /handle_obstacle rotate_speed 1.0
```

#### Funktionalität testen

1. **Hindernisdaten prüfen**:
   - Veröffentlicht auf dem Topic `/scan`:
     ```bash
     ros2 topic echo /scan
     ```
2. **Bewegungsbefehle prüfen**:
   - Veröffentlicht auf dem Topic `/cmd_vel`:
     ```bash
     ros2 topic echo /cmd_vel
     ```
