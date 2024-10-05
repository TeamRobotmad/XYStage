# XYStage app

Tildagon app for EMFCamp Badge 2024 to control a simple Microscope XY Stage via two DRV8825 stepper motor drivers with 2 endstop microswitches (one per axis).

## User guide

Install the XYStage app and then plug your Hexpansion board into SLOT 1 on your EMF Camp 2024 Badge.

### Main Menu ###

The main menu will present options for "XYStage", "Settings", "About"and "Exit".

### Settings ###

The main menu includes a sub-menu of Settings which can be adjusted.
My hardware is using 1/32th microstepping - hence the *32
#### Stepper Axis Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| width            | width of stage in stepper pulses          | 2000*32        | 10     | 100000 |
| height           | height of stage in stepper pulses         | 2000*32        | 10     | 100000 |
| XRange           | X Range of movement in stepper pulses     | 2200*32        | 10     | 100000 |
| YRange           | Y Range of movement in stepper pulses     | 2000*32        | 10     | 100000 |

#### Other Settings ####
| Setting          | Description                               | Default        | Min    | Max    |
|------------------|-------------------------------------------|----------------|--------|--------|
| logging          | Enable or disable logging                 | False          | False  | True   |

### Limitations ###

My hexpansion hardware does not have an EEPROM so the slot is hardcoded at present as the Badge can't automatically detect which it is plugged into.
This uses one PWM resource per stepper motor - so there must be two available - hence if you are using lots of PWM or Timer resources for other things there may not be enough available and the app won't run.

### Install guide

Stable version available via [Tildagon App Directory](https://apps.badge.emfcamp.org/).

This repo contains lots of files that you don't need on your badge. If you want to load a minimal application onto a badge directly you only need the files:
+ app.py
+ tildagon.toml
+ metadata.json
+ utils.py

### Running tests
```
pytest test_smoke.py
```

### Best practise
Run `isort` on in-app python files. Check `pylint` for linting errors.


### Contribution guidelines
