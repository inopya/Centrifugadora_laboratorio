# Centrifugadora_laboratorio

Autor: Eulogio López Cayuela

Versión 2.0    Fecha: 26/09/2017 


  NOTAS DE ESTA VERSION:
  
    Control de centrifugadora de laboratorio casero mediante ARDUINO UNO
    para actuar sobre el sistema se dispone de un potenciometro y un pulsador
    El potenciometro permite introducir valores en los campos corespondientes
    y el pulsador aceptar dichos valores
    Esta vesion tambien dispone de un tamper de seguridad que desconecta el servo
    en caso de que se abra la tapa de proceccion, avitandose asi posibles accidentes
    Permite asi mismo la apertura controla de dicha tapa durante la ejecucion de un trabajo
    haciendo una pausa en el mismo y que podemos reanudar despues de cerrar de nuevo


    Para los valores de tiempo dispomenos del rango entre 0 y 59 para cada uno de los campos --> HH:mm:ss
    que se modificaran actuando sobre el potenciometro.
    El valor de un campo queda aceptado al activar el pulsador, 'saltando' automaticamente al siguente campo.
    - Podemos corregir un valor ya aceptado de la siguiente manera:
        En la posicion minima del potenciometro el valor que obtenemos es -1, que el programa  
        nos representa como'--'. Si pulsamos en ese estado el programa interpetra que queremos 
        corregir el valor anterior y nos mueve a dicho campo

    Otras explicaciones sobre el funcionamiento de funciones, en el codigo de dichas funciones
    
    
 Esta version no es 100% definitiva y tiene algunos bugs en la funcion de autoregulacion de velocidad
 debido a la precipitacion para terminarla a tiempo para el biotecnoencuentro del dia 29/09/2017
  
