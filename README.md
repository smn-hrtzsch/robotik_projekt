# Robotik Projekt 2024/25

Dieses Repository enthält die Lösung für die Übungs- und Belegaufhaben aus dem Modul **Robotik Projekt**. Implementiert werden ROS2-Packages in C++ und Python. 

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
1. **Workspace erstellen**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/smn-hrtzsch/robotik_projekt.git
   cd ~/ros2_ws
   colcon build
   source ~/ros2_ws/install/local_setup.bash
   ```

2. **Abhängigkeiten installieren**:
  - ROS2 Humble ([Installationsanleitung](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
  - Colcon:
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

#### Publisher starten
  ```bash
  ros2 run cpp_pub publisher
  ```

#### Subscriber starten
In einem neuen Terminal:
```bash
source ~/ros2_ws/install/local_setup.bash
ros2 run py_sub subscriber
```

#### Funktionalität testen
In einem neuen Terminal:
- Daten des Publishers anzeigen:
  ```bash
  ros2 topic echo /number
  ```
- Zeitdifferenzen des Subscribers überprüfen:
  ```bash
  ros2 topic echo /diff
  ```

---

## Aufgabe 2
