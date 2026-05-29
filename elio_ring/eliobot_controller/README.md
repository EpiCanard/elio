# Elio ring controller

Sketch Arduino pour Seeed XIAO nRF52840 Sense.

Il se connecte en BLE central au robot `Eliobot`, cherche le service Nordic UART, puis envoie des lignes:

```text
angle vitesse
```

Le format correspond a `eliobot/main.py`, par exemple `-30 45`.

## Utilisation

- Allumer le robot avec `eliobot/main.py`.
- Televerser `eliobot_controller.ino` sur le XIAO nRF52840 Sense.
- Garder la main dans la position neutre pendant environ 1 seconde au demarrage: cette position est calibree comme zero.
- Incliner le doigt vers l'avant/arriere pour commander la vitesse.
- Incliner lateralement le doigt pour commander l'angle.
- Bouton sur la broche `10`: recalibre la position neutre.
- Bouton sur la broche `8`: envoie `0 0`.

## Reglage de sensibilite

Les amplitudes sont volontairement faibles pour une bague:

```cpp
const float SPEED_FULL_SCALE_DEG = 24.0f;
const float SPEED_DIRECTION = 1.0f;
const float STEERING_FULL_SCALE_DEG = 45.0f;
const float MAX_STEERING_COMMAND = 90.0f;
```

Diminue ces valeurs si le mouvement du doigt est trop petit pour atteindre la vitesse ou l'angle max. Augmente-les si le robot est trop nerveux.

Si le sens avant/arriere est inverse selon l'orientation de la bague, remplace:

```cpp
const float SPEED_DIRECTION = 1.0f;
```

par `-1.0f`.
