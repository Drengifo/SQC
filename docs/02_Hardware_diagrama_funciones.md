# Descripcion del sistema
El equipo electronico se compone de una Raspberry Pi Zero 2 y la placa PCB SQCv1.2
<p align="center">
  <img width="845" height="534" alt="imagen" src="https://github.com/user-attachments/assets/85791c1f-d3c9-46ad-a3cb-5992ccae93de" />  
</p>

## 1. Pinout PCB SQCv1.2
  La placa contiene un display 0.96 por i2c, boton (en pull-up), led rgb mas selector de modo anodo comun o cátodo comun y resistencia 10k.
  <p align="center">
    <img width="475" height="437" alt="imagen" src="https://github.com/user-attachments/assets/5122cf29-7727-429f-afbe-97a59cdcfd5c" />
  <p
    
  En la siguiente imagen se ilustra el diagrama de conexiones y funciones:
  <p align="center">
    <img width="882" height="477" alt="imagen" src="https://github.com/user-attachments/assets/f0df3cfb-d19a-4044-ba84-672877dbfbc5" />
  </p>

  Hay versiones del OLED que tienen los pines VCC y GND cambiados, se debe tener precaucion con eso. 
  El jumper selector se diseño en la etapa de prototipado y permite escoger entre usar led rgb con cátodo comun o ánodo comun.

## 2. Funcionamiento
  El equipo entrega informacion y recibe ordenes mediante la placa PCB SQCv1.2 o el PORTAL WEB.
  A continuacion se describen las principales funciones de la placa PCB SQCv1.2
  1. PANTALLA
     Cuando el sistema enciende, en la pantalla entrega el nombre de la red, IP, modo y estado.
     <p align="center">
        <img width="283" height="244" alt="imagen" src="https://github.com/user-attachments/assets/786880a7-8838-46eb-aa35-8bc9fcf174e3" />
     </p>
     
     - El display se apaga automaticamente luego de 2 min sin actividad.
     - Se vuelve a prender si se presiona una vez el boton.
    
  2. LED

     El equipo se conectara por defecto a la red wifi conocida o se establecera en modo AP.
     - Cuando está en modo WIFI y conexion exitosa, el led Parpadea en color verde.
     - Cuando el equipo entra en modo AP, parpadea en color azul.
     - Si ocurre un error critico, el led se establece en color rojo.
     - Las indicaciones led se apagan luego de 2 min de inactividad
    
    
  4. BOTON
     - Si se presiona 1 vez, se enciende la pantalla y el led
     - Si se presiona 3 veces seguidas, el sistema toma una foto
     - Si se mantiene por 3 segundos el equipo cambia entre modo WIFI o AP
     - Si se mantiene por 10 segundos o mas, el equipo se reinicia.
        
  


