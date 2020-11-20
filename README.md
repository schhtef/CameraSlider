# Camera Actuation Control System
## Description
The CACS is software which provides an intuitive interface between a filmmaker and the camera slider’s motorized system. It attempts to give control to the user such that they can easily make adjustments to camera movements without being hindered, but also hide complexity which is irrelevant to filmmaking.

### Dependencies
The system is arduino based, so it relies heavily on the [Arduino AVR Core](https://github.com/arduino/ArduinoCore-avr).
It also uses the following libraries:
- [ArduinoMenu](https://github.com/neu-rah/ArduinoMenu)
- [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)

### Current Features
- Manual enable/disable of stepper motors
- Fully functioning, navigatable, extendable menu
- Timelapse mode
- Homing function

### Planned Features
- Control of a second motor for camera rotation
- Enriched timelapse mode with calculations for incremental camera rotation
- Dynamic setting of camera position, ie. movements in direct response to input
- Additional modes

## Behaviour
The table below summarizes the CACS. The intersection of an input and menu context is the resulting output.
| Input | Main Menu | Timelapse Menu | Input Field | Enable Stepper | Home Stepper | Execute Timelapse |
|-|-|-|-|-|-|-|
| Clockwise encoder rotation | Shift focus down one item | Shift focus down one item | Increase field value | N/A | N/A | N/A |
| Counter-clockwise encoder rotation  | Shift focus up one item | Shift focus up one item | Decrease field value | N/A | N/A | N/A |
| Encoder Push-Button Press | Enter sub-menu | Trigger menu item | Change field change resolution | Turn on stepper motor | Start stepper homing function | Carry out timelapse program |
