# Modelos para impresion 3D del mando

Esta carpeta contiene los modelos 3D utilizados para fabricar mediante impresion 3D las piezas del mando del proyecto de programacion tangible.

Los archivos estan organizados en dos grupos:

## `Botones  y cajetin/`

Contiene las piezas de los botones y del cajetin donde se montan:

- `Boton11_F.STL` a `Boton16_F.STL`: archivos preparados para imprimir los botones.
- `Cajetin_F.STL`: archivo preparado para imprimir el cajetin.
- `Tapaboton.STL`: tapa o pieza de cierre de los botones.

En esta misma carpeta se conservan los archivos `.SLDPRT`, que son los modelos originales y editables de SolidWorks correspondientes a los botones y al cajetin.

## `tablero/`

Contiene las piezas estructurales del tablero y sus tapas:

- `Mando_1_2.STL` y `Mando_2_2.STL`: piezas principales del mando.
- `S_MndoF.STL`: soporte o estructura del mando.
- `Tapa_MP_1_2.STL` y `Tapa_MP_2_2.STL`: tapas de las piezas principales.
- `Tapa_S_mndo.STL`: tapa del soporte del mando.
- `Mando.zip`: archivo comprimido con material relacionado con el conjunto del mando.

Los archivos `.SLDPRT` de esta carpeta son los modelos editables utilizados como fuente para generar los archivos `.STL`.

## Uso de los archivos

- Utilizar los archivos `.STL` en el programa laminador de la impresora 3D.
- Seleccionar la orientacion, escala y parametros de impresion segun el material y la impresora disponibles.
- Utilizar los archivos `.SLDPRT` para realizar modificaciones de diseño en SolidWorks antes de volver a exportar a `.STL`.
- Revisar las dimensiones y el ensamble de las piezas antes de imprimir el conjunto completo.

Los archivos `.STL` son los modelos destinados a las impresiones 3D; los archivos `.SLDPRT` se mantienen como archivos de diseño y edicion.
