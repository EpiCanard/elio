# Elio ring controller

Sketch Arduino pour Seeed XIAO nRF52840 Sense. La bague lit son accelerometre
LSM6DS3, transforme deux inclinaisons du doigt en commandes `angle vitesse`,
puis les envoie au robot Eliobot avec le service BLE Nordic UART.

## Utilisation

- Allumer le robot avec `eliobot/main.py`.
- Televerser ce sketch sur la XIAO nRF52840 Sense.
- Garder la main dans la position neutre pendant environ 1 seconde au demarrage.
  Cette position est calibree comme zero.
- Incliner le doigt vers l'avant/arriere pour commander la vitesse.
- Incliner lateralement le doigt pour commander l'angle.
- Bouton sur la broche `10`: recalibre la position neutre.
- Bouton sur la broche `8`: active/desactive le mode stop. En mode stop, la
  bague continue d'envoyer `0 0` jusqu'au prochain appui.

Le robot recoit des lignes texte:

```text
angle vitesse
```

Exemples:

```text
0 0
-30 45
20 -35
```

## Commandes utiles

Depuis la racine du depot:

```bash
./scripts/flash_ring.sh
```

Le script compile dans `/tmp/elio-ring-build`, detecte la XIAO avec
`/dev/serial/by-id` quand c'est possible, puis utilise le vrai port
`/dev/ttyACM*` pour l'upload.

Pour ouvrir les logs serie:

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## Algorithme

### 1. Lecture IMU

La fonction `read_accel()` lit les trois axes bruts de l'accelerometre:

```cpp
raw_x = myIMU.readFloatAccelX();
raw_y = myIMU.readFloatAccelY();
raw_z = myIMU.readFloatAccelZ();
```

Pour le montage actuel de la bague, les mesures de debug ont montre que:

- avancer/reculer fait surtout varier `Z`;
- tourner gauche/droite fait surtout varier `X`;
- `Y` reste proche de l'axe de reference au neutre.

Le code utilise donc `Y` comme reference commune:

```cpp
speed_tilt = atan2(Z, Y)
roll = atan2(X, Y)
```

Les angles sont convertis en degres.

### 2. Calibration neutre

Au demarrage, `calibrate_neutral()` prend `CALIBRATION_SAMPLES` mesures. Avec la
valeur actuelle `200` et un `delay(5)`, la calibration dure environ 1 seconde.

Le code calcule deux biais:

```cpp
speed_tilt_bias = moyenne(atan2(Z, Y))
roll_bias = moyenne(atan2(X, Y))
```

Ensuite, a chaque boucle:

```cpp
speed_tilt = angle_delta_degrees(speed_tilt_degrees(reading), speed_tilt_bias);
roll = roll_degrees(reading) - roll_bias;
```

`angle_delta_degrees()` ramene l'ecart dans `[-180, 180]`. C'est surtout utile
pour eviter une discontinuite si un angle traverse `180` ou `-180`.

Le facteur:

```cpp
const float SPEED_DIRECTION = -1.0f;
```

inverse la vitesse, parce que le geste naturel "avancer" produit un `Z` negatif
sur ce montage.

### 3. Deadzone et mise a l'echelle

Les inclinaisons mesurees ne sont pas envoyees directement au robot. Elles
passent par `apply_deadzone_scaled()`:

```cpp
target_speed = apply_deadzone_scaled(speed_tilt, 3, 24, 100);
target_angle = apply_deadzone_scaled(roll, 8, 45, 90);
```

Pour chaque axe:

- si la valeur absolue est sous la deadzone, la commande vaut `0`;
- entre la deadzone et le full-scale, la commande augmente lineairement;
- au-dela du full-scale, la commande est saturee au maximum.

Avec les constantes actuelles:

```cpp
const float SPEED_DEADZONE_DEG = 3.0f;
const float SPEED_FULL_SCALE_DEG = 24.0f;
const float STEERING_DEADZONE_DEG = 8.0f;
const float STEERING_FULL_SCALE_DEG = 45.0f;
const float MAX_STEERING_COMMAND = 90.0f;
```

La vitesse atteint donc `-100` ou `100` vers 24 degres d'inclinaison. L'angle
atteint `-90` ou `90` vers 45 degres.

### 4. Lissage

Pour eviter des commandes trop nerveuses, le sketch applique un filtre
exponentiel:

```cpp
filtered_speed += (target_speed - filtered_speed) * SPEED_SMOOTHING;
filtered_angle += (target_angle - filtered_angle) * STEERING_SMOOTHING;
```

Avec:

```cpp
const float SPEED_SMOOTHING = 0.22f;
const float STEERING_SMOOTHING = 0.25f;
```

Plus la valeur est proche de `1`, plus la commande suit vite la main. Plus elle
est proche de `0`, plus le mouvement est amorti.

### 5. Envoi BLE

`setup_bluetooth()` configure la XIAO en BLE central. Elle scanne les appareils
qui exposent le service Nordic UART, se connecte, puis decouvre le service:

```cpp
BLEClientUart robot_uart;
```

La fonction `send_drive_command()` limite la frequence d'envoi avec:

```cpp
const uint16_t SEND_INTERVAL_MS = 50;
```

Elle arrondit ensuite les commandes:

```cpp
angle -> [-90, 90]
speed -> [-100, 100]
```

Puis envoie:

```cpp
robot_uart.printf("%d %d\n", command_angle, command_speed);
```

Le robot attend exactement ce format dans `eliobot/main.py`.

### 6. Stop toggle

Le bouton `STOP_PIN` est lu avec une detection de front et un debounce:

```cpp
const uint16_t BUTTON_DEBOUNCE_MS = 200;
```

Quand le bouton est presse:

- `stop_enabled` bascule entre `true` et `false`;
- les valeurs filtrees sont remises a zero;
- `0 0` est envoye immediatement.

Tant que `stop_enabled` vaut `true`, la boucle continue d'envoyer `0 0` et ne
calcule plus les gestes.

## Mode debug IMU

Pour analyser les axes sans piloter le robot:

```cpp
const bool IMU_DEBUG_ONLY = true;
```

Dans ce mode:

- le BLE n'est pas demarre;
- le robot ne recoit aucune commande;
- le port serie affiche les axes bruts, les deltas depuis la calibration et les
  deux angles calcules.

Exemple:

```text
imu: raw=(0.018,0.993,0.074) delta=(0.037,0.017,0.005) speed_tilt=2.1 roll=-0.7
```

Le bouton `STOP_PIN` sert alors a activer/desactiver l'affichage:

```text
debug_output: off
debug_output: on
```

Ce mode a servi a choisir le mapping actuel:

- `Z` negatif pendant le geste avancer;
- `Z` positif pendant le geste reculer;
- `X` negatif pendant le geste tourner gauche;
- `X` positif pendant le geste tourner droite.

## Reglages courants

Si le robot avance au lieu de reculer, inverser:

```cpp
const float SPEED_DIRECTION = -1.0f;
```

en:

```cpp
const float SPEED_DIRECTION = 1.0f;
```

Si le robot tourne du mauvais cote, inverser le signe dans `roll_degrees()`:

```cpp
return atan2f(reading.raw_x, reading.raw_y) * RADIANS_TO_DEGREES;
```

en:

```cpp
return atan2f(-reading.raw_x, reading.raw_y) * RADIANS_TO_DEGREES;
```

Si la commande est trop sensible, augmenter `SPEED_FULL_SCALE_DEG` ou
`STEERING_FULL_SCALE_DEG`. Si elle ne va pas assez loin avec le mouvement du
doigt, les diminuer.

Si le robot bouge trop facilement autour du neutre, augmenter
`SPEED_DEADZONE_DEG` ou `STEERING_DEADZONE_DEG`.
