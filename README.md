# Programacion tangible

Este proyecto controla un tablero tangible mediante fichas con resistencias. Cada ficha representa una instruccion. Los Arduinos miden las resistencias, convierten sus valores en acciones y envian esas acciones a un mecanismo CNC que mueve el robot y reproduce indicaciones de audio.

## Flujo general

1. Las fichas se conectan a los divisores de tension de `tablero_esclavo` y `tablero_subrutina`.
2. Cada Arduino de medicion calcula la resistencia de sus fichas y la convierte en un codigo de accion.
3. `tablero_maestro` consulta los tres Arduinos por I2C.
4. El maestro muestra el estado de las fichas mediante el controlador PCA9685 y sus LEDs.
5. Al pulsar el boton, el maestro ejecuta la secuencia y envia los codigos por Bluetooth al CNC.
6. `cnc` recibe cada codigo, reproduce el audio correspondiente y mueve los motores del sistema H-BOT.

## Carpetas

### `audios/`

Contiene los archivos `.mp3` que reproduce el modulo DFPlayer Mini conectado al Arduino del CNC. El programa `cnc/cnc.ino` usa `myDFPlayer.play(numero)` y asigna una pista a cada accion:

| Pista | Accion |
| ---: | --- |
| 1 | Mover arriba |
| 2 | Mover abajo |
| 3 | Mover izquierda |
| 4 | Mover derecha |
| 5 | Perdida de conexion |
| 6 | Melodia 1 |
| 7 | Inicio de homing |
| 8 | Inicio del recorrido |
| 9 | Homing completo |
| 10 | Inicio de homing alternativo |
| 11 | Recorrido terminado / centro terminado |
| 12 | Accion no valida |
| 13 | Centro |

Los archivos que existen actualmente son `0.005.mp3`, `0.007.mp3`, `0.008.mp3`, `0.009.mp3`, `0.010.mp3`, `0.011.mp3` y `0012.mp3`. No se observan pistas para 1, 2, 3, 4, 6 ni 13. Para que el DFPlayer las encuentre de forma predecible, conviene copiar los audios a una tarjeta microSD y nombrarlos con numeros consecutivos, por ejemplo `0005.mp3`, `0007.mp3` o el formato recomendado por el DFPlayer que se este utilizando.

La tarjeta debe insertarse en el DFPlayer antes de encender el CNC. El volumen se configura en el codigo con `myDFPlayer.volume(30)`.

### `cnc/`

Contiene `cnc.ino`, el firmware del Arduino Nano que controla el mecanismo H-BOT.

Sus responsabilidades son:

- Recibir codigos numericos por Bluetooth HC-05.
- Ejecutar movimientos con dos motores paso a paso 28BYJ-48 y sus controladores ULN2003.
- Realizar homing usando dos finales de carrera o sensores Hall.
- Controlar los LEDs de estado de Bluetooth.
- Reproducir los audios de `audios/` mediante un DFPlayer Mini.
- Aplicar calibracion al cambiar el sentido del eje X.

El CNC recibe los codigos enviados por el maestro a `9600 baudios`. El receptor intenta leer hasta `:` y tiene un tiempo de espera corto; el maestro actualmente envia cada codigo con `println`, por lo que tambien puede procesarlo al vencer ese tiempo de espera. Los codigos principales son:

| Codigo | Accion |
| ---: | --- |
| 1 | Arriba |
| 2 | Abajo |
| 3 | Izquierda |
| 4 | Derecha |
| 6 | Melodia 1 |
| 7 | Homing |
| 10 | Homing / reinicio |
| 12 | Movimiento no valido |
| 13 | Ir al centro |
| 14 | Centro alcanzado |

El HC-05 usa los pines 10 y 11. El DFPlayer usa los pines 12 y 13. Los motores utilizan los pines 2 a 9 y los finales de carrera usan `A0` y `A1`.

### `tablero_esclavo/`

Contiene `tablero_esclavo.ino`, el firmware del Arduino que mide cuatro fichas de una columna.

- Direccion I2C: `0x02`.
- `A0`: voltaje de entrada comun.
- `A1`, `A2`, `A3` y `A6`: salidas de los cuatro divisores de tension.
- `A4` y `A5`: bus I2C.
- Resistencia de referencia del divisor: `1000 ohmios`.
- Envia cuatro valores de instruccion al maestro, cada uno como un `float` de 4 bytes.

El programa toma varias muestras de cada entrada, calcula la resistencia con:

`R = (Vout * Rreferencia) / (Vin - Vout)`

Luego clasifica el valor en una accion. Los rangos configurados son:

| Resistencia aproximada | Codigo | Accion |
| ---: | ---: | --- |
| 100-600 ohmios | 2 | Abajo |
| 800-1600 ohmios | 1 | Arriba |
| 1700-2600 ohmios | 4 | Derecha |
| 3000-6000 ohmios | 3 | Izquierda |
| 7000-15000 ohmios | 6 | Melodia |
| 70000-130000 ohmios | 5 | Bloque de control |

### `tablero_subrutina/`

Contiene `tablero_subrutina.ino`, el firmware del Arduino que mide tres fichas del bloque de control.

- Direccion I2C: `0x03`.
- `A0`: voltaje de entrada comun.
- `A1`, `A2` y `A3`: salidas de los tres divisores de tension.
- `A4` y `A5`: bus I2C.
- Envia tres valores de instruccion al maestro, cada uno como un `float` de 4 bytes.

Utiliza los mismos rangos de resistencia y codigos que `tablero_esclavo`. Sus tres posiciones se ejecutan como subinstrucciones cuando el maestro detecta una ficha de bloque de control.

### `tablero_maestro/`

Contiene `tablero_maestro.ino`, el firmware del Arduino central.

Sus responsabilidades son:

- Consultar por I2C los esclavos `0x01`, `0x02` y `0x03`.
- Leer cuatro instrucciones de cada columna y tres instrucciones del bloque de control.
- Mantener ocho instrucciones principales y tres subinstrucciones.
- Controlar 11 LEDs mediante un PCA9685 en la direccion I2C `0x40`.
- Enviar las acciones al CNC mediante Bluetooth HC-05.
- Validar que los movimientos permanezcan dentro de una cuadricula de `5 x 5`.
- Permitir iniciar, pausar, reanudar y reiniciar la secuencia con un boton.
- Mantener la posicion logica del robot y devolverlo al centro cuando corresponde.

El maestro usa:

- Bus I2C en `A4` (SDA) y `A5` (SCL).
- Boton de inicio en el pin digital `2`, con `INPUT_PULLUP`.
- Bluetooth en los pines 10 y 11.
- Estado del Bluetooth en `A2`.
- LED verde de conexion en `A0` y LED rojo de desconexion en `A1`.

Estados de ejecucion:

- **Lectura:** actualiza las mediciones y mantiene los LEDs al 20%.
- **Corrida:** ejecuta las ocho instrucciones principales en orden.
- **Pausa:** detiene la secuencia y conserva el punto actual.
- **Reinicio:** vuelve al centro o ejecuta el homing necesario.

Una ficha con resistencia de bloque de control no mueve directamente el CNC. En su lugar, hace que el maestro ejecute, en orden, las tres instrucciones leidas por `tablero_subrutina`.

## Comunicaciones

### I2C

El maestro es el controlador del bus:

| Dispositivo | Direccion | Datos enviados al maestro |
| --- | ---: | ---: |
| Columna 1 | `0x01` | 4 instrucciones |
| Columna 2 / `tablero_esclavo` | `0x02` | 4 instrucciones |
| Bloque de control / `tablero_subrutina` | `0x03` | 3 instrucciones |
| PCA9685 | `0x40` | Control de 11 LEDs |

El maestro solicita los datos y cada esclavo responde con valores de 4 bytes por canal. `A4` y `A5` deben estar conectados entre todos los Arduinos del bus y deben compartir GND.

### Bluetooth

El maestro transmite al CNC los codigos de accion mediante un HC-05. Ambos modulos trabajan a `9600 baudios`. El pin `STATE` permite detectar la conexion y evitar ejecutar instrucciones cuando el enlace no esta disponible.

## Puesta en marcha

1. Instalar en Arduino IDE las librerias `Wire`, `SoftwareSerial`, `DFRobotDFPlayerMini` y `Adafruit_PWMServoDriver`.
2. Preparar una tarjeta microSD para el DFPlayer con los audios numerados que requiere `cnc.ino`.
3. Cargar `tablero_esclavo.ino` en cada Arduino de columna. Para la columna con direccion `0x01`, cambiar `I2C_SLAVE_ADDRESS` de `0x02` a `0x01` antes de cargarlo.
4. Cargar `tablero_subrutina.ino` en el Arduino del bloque de control, conservando la direccion `0x03`.
5. Cargar `tablero_maestro.ino` en el Arduino central.
6. Cargar `cnc.ino` en el Arduino que controla los motores.
7. Revisar que los HC-05 esten emparejados, que el bus I2C comparta GND y que los pines no esten cruzados incorrectamente.
8. Encender el sistema, verificar la conexion Bluetooth y esperar el homing inicial del CNC.
9. Colocar las fichas, comprobar las lecturas por el Monitor Serial y pulsar el boton para iniciar la secuencia.

## Monitores seriales

- `tablero_maestro`: `9600 baudios`.
- `tablero_esclavo`: `4800 baudios`.
- `tablero_subrutina`: `4800 baudios`.
- `cnc`: `115200 baudios`.

Los valores de resistencia y las acciones detectadas pueden verificarse desde los monitores seriales de los respectivos Arduinos.
