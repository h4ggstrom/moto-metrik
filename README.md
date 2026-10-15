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

Angle estimé depuis le GPS : `tan(angle) = v × vitesse de rotation du cap / g` (virage stabilisé, pneu fin, pilote dans l'axe). Validé sur simulation : erreur moyenne 1,2°. La moto réelle penche quelques degrés de plus (largeur du pneu), moins si le pilote se déhanche.
