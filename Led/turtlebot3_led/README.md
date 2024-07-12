Allumer les LED de manière à ce que je puisse le copier :
```bash
rosservice call /chooseColor 1 0 1

voila les couleur disponible associer à leur états binair 
- `turnOff()`: Red = 0, Green = 0, Blue = 0
- `greenLED()`: Red = 0, Green = 1, Blue = 0
- `redLED()`: Red = 1, Green = 0, Blue = 0
- `blueLED()`: Red = 0, Green = 0, Blue = 1
- `violetLED()`: Red = 1, Green = 0, Blue = 1
- `cyanLED()`: Red = 0, Green = 1, Blue = 1
- `total()`: Red = 1, Green = 1, Blue = 1

