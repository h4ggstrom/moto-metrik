# moto-metrik

Boîtier télémétrique moto : ESP32 + GPS + carte SD. Le firmware écrit le NMEA brut sur la SD (un fichier `log_NNNN.nmea` par démarrage), l'analyse (tours, tracés, deltas) se fait sur PC.

## Câblage

| Module | Broche module | ESP32 |
|---|---|---|
| GPS | VCC / GND | 3V3 / GND |
| GPS | TX | GPIO16 (RX2) |
| GPS | RX | GPIO17 (TX2) |
| SD | VCC / GND | VIN (5 V) / GND |
| SD | MISO | GPIO19 |
| SD | MOSI | GPIO23 |
| SD | SCK | GPIO18 |
| SD | CS | GPIO5 |

Ne pas brancher le GPS sur RX0/TX0 : ces broches sont reliées au convertisseur USB-série (conflit upload/moniteur).
Pins modifiables dans `firmware/include/config.h`.

LED intégrée : clignote vite = erreur SD, lentement = attente fix GPS, fixe = enregistrement avec fix.

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

Les circuits de `tools/tracks/` (tracé OpenStreetMap + position de la ligne et des secteurs en mètres le long du tracé) sont détectés automatiquement : tours, secteurs et tour idéal sans option. Circuit Carole : ligne et secteurs **provisoires**, à caler sur les vraies lignes.

Hors circuit connu et sans `--gate`, la ligne est placée au point de vitesse max (cohérent pour comparer les tours, mais pas la vraie ligne). Sorties dans `out/<log>/` : `track.png`, `lean.png` (angle sur le tracé), `laps.png` (vitesse, angle et delta vs meilleur tour en fonction de la distance), `laps.csv`, `session.csv` (chaque point : vitesse, cap, angle, accélérations), `session.gpx`.

Angle estimé depuis le GPS : `tan(angle) = v × vitesse de rotation du cap / g` (virage stabilisé, pneu fin, pilote dans l'axe). Validé sur simulation : erreur moyenne 1,2°. La moto réelle penche quelques degrés de plus (largeur du pneu), moins si le pilote se déhanche.
