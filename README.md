# moto-metrik

Boîtier télémétrique moto : ESP32 + GPS + carte SD. Le firmware écrit le NMEA brut sur la SD, un fichier par démarrage, nommé d'après l'heure de démarrage (`2026-10-14_15-02-33.nmea`, heure de Paris) dès le premier fix GPS ; il reste `log_NNNN.nmea` si aucun fix n'est obtenu, l'analyse (tours, tracés, deltas) se fait sur PC.

## Câblage

Le GPS et l'IMU partagent le 3V3 et un GND (épissures en Y soudées, une seule fiche Dupont femelle sur la broche ESP32). La SD a son alimentation à part.

| Module | Broche module | Fil | ESP32 |
|---|---|---|---|
| GPS | VCC | rouge | 3V3 (Y partagé avec IMU VCC) |
| GPS | GND | noir | GND n° 1 (Y partagé avec IMU GND) |
| GPS | TX | bleu | GPIO16 (RX2) |
| GPS | RX | violet | GPIO17 (TX2) |
| IMU | VCC | rouge | 3V3 (Y partagé avec GPS VCC) |
| IMU | GND | noir | GND n° 1 (Y partagé avec GPS GND) |
| IMU | SCLK (SCL) | blanc | GPIO22 |
| IMU | SDI (SDA) | marron | GPIO21 |
| IMU | NCS | gris | GPIO4 (piloté à l'état haut : mode I2C) |
| IMU | SDO (AD0) | orange | GPIO26 (piloté à l'état bas : adresse 0x68) |
| IMU | FSY | vert | GPIO25 (piloté à l'état bas) |
| IMU | INT | jaune | GPIO27 (pas encore utilisé) |
| IMU | ACL / ADA | — | non branchés |
| SD | VCC | rouge | VIN (5 V) |
| SD | GND | noir | GND n° 2 |
| SD | MISO | orange | GPIO19 |
| SD | MOSI | marron | GPIO23 |
| SD | SCK | blanc | GPIO18 |
| SD | CS | gris | GPIO5 |

Optionnel, lecture du bus CAN de la moto (écoute seule, le boîtier n'émet jamais) : transceiver SN65HVD230 en 3V3.

| Module | Broche module | ESP32 / moto |
|---|---|---|
| CAN | 3V3 | 3V3 |
| CAN | GND | GND (et masse de la moto) |
| CAN | CTX (D) | GPIO32 |
| CAN | CRX (R) | GPIO33 |
| CAN | CANH / CANL | CAN-H / CAN-L de la prise diagnostic |

Le débit (125 k à 1 Mbit/s) est détecté automatiquement. Trames enregistrées en `$PCAN,ms,ID,DATA` ; `tools/can_sniff.py LOG.nmea` liste les identifiants et classe les octets selon leur corrélation avec la vitesse GPS, l'accélération et l'angle.

Ne pas brancher le GPS sur RX0/TX0 : ces broches sont reliées au convertisseur USB-série (conflit upload/moniteur).
Pins modifiables dans `firmware/include/config.h`.

LED intégrée : clignote vite = erreur SD, double clignotement = plus de données du GPS (fil débranché, GPS sans alim), lentement = attente fix GPS, fixe = enregistrement avec fix.

Seules les trames NMEA complètes avec un checksum valide sont écrites. Si le GPS se tait plus de 2 s, le firmware le recherche et le reconfigure (10 Hz, modèle automobile) toutes les 5 s, ce qui couvre le GPS qui redémarre sur ses réglages d'usine après une coupure d'alim.

## Firmware

```sh
cd firmware
pio run -t upload
pio device monitor
```

## Analyse

```sh
python3 -m venv .venv && .venv/bin/pip install -r tools/requirements.txt
.venv/bin/python tools/analyze.py /chemin/vers/log_0001.nmea --gate LAT,LON
```

Les circuits de `tools/tracks/` (tracé OpenStreetMap + position de la ligne et des secteurs en mètres le long du tracé) sont détectés automatiquement : tours, secteurs et tour idéal sans option. Circuit Carole : ligne de départ/arrivée et secteurs calés sur les vraies lignes.

Hors circuit connu et sans `--gate`, la ligne est placée au point de vitesse max (cohérent pour comparer les tours, mais pas la vraie ligne). Sorties dans `out/<log>/` : `track.png`, `lean.png` (angle sur le tracé), `laps.png` (vitesse, angle et delta vs meilleur tour en fonction de la distance), `laps.csv`, `session.csv` (chaque point : vitesse, cap, angle, accélérations), `session.gpx`.

### Vidéo GoPro avec incrustation

```sh
.venv/bin/python tools/overlay.py LOG.nmea GX010325.MP4 GX020325.MP4            # session entière
.venv/bin/python tools/overlay.py LOG.nmea GX010325.MP4 --start 600 --duration 90  # extrait
.venv/bin/python tools/overlay.py LOG.nmea GX010325.MP4 --still 600               # image de contrôle
```

La vidéo est calée sur le log par son propre GPS (heure GPS de la GoPro, puis corrélation des vitesses : précision ~2 images). Incrustation en direct : chrono du tour, écart au meilleur tour, temps des secteurs (violet = meilleur), carte avec la position, angle (avec le max du tour), vitesse, g longitudinal. Les chapitres d'un même enregistrement (GX01…, GX02…) sont donnés dans l'ordre et raccordés ; la vidéo verticale (caméra montée de côté) est gérée. Sortie : `out/<log>/overlay.mp4`. ffmpeg est fourni par le paquet `imageio-ffmpeg`.

Angle estimé depuis le GPS : `tan(angle) = v × vitesse de rotation du cap / g` (virage stabilisé, pneu fin, pilote dans l'axe). Validé sur simulation : erreur moyenne 1,2°. La moto réelle penche quelques degrés de plus (largeur du pneu), moins si le pilote se déhanche.
