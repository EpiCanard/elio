# Eliobot Ring Controller

Ce depot contient le code pour piloter un robot Eliobot en Bluetooth Low Energy
avec une bague basee sur une Seeed XIAO nRF52840 Sense.

## Composition

- `eliobot/` : firmware CircuitPython du robot ESP32-S3. `main.py` expose un
  service BLE Nordic UART, recoit des commandes `angle vitesse`, pilote les
  moteurs et alterne la couleur des yeux avec la commande `eyes`.
- `elio_ring/` : sketch Arduino de la bague. Il lit l'accelerometre LSM6DS3,
  convertit l'inclinaison du doigt en vitesse/angle, puis envoie les commandes
  au robot en BLE.
- `elio_client/` : petit client Python BLE pour tester l'envoi de commandes
  depuis un ordinateur.

## Commandes BLE

Le robot attend une ligne texte au format :

```text
angle vitesse
```

Exemples :

```text
0 0
-30 50
20 -40
eyes
```

## Construire et uploader

Lister les cartes et ports disponibles :

```bash
arduino-cli board list
```

Uploader le programme de la bague :

```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:nrf52:xiaonRF52840Sense elio_ring/eliobot_controller.ino
```

Deployer le firmware CircuitPython du robot :

```bash
ampy --port /dev/serial/by-id/usb-ELIO_Eliobot_4BA354FAB9CC-if00 put eliobot/main.py /main.py
```

## Test rapide

Depuis `elio_client/`, installer les dependances puis envoyer une commande :

```bash
pipenv install
pipenv run python client.py --message "0 0" --wait 2
```
