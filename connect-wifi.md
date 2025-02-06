---
# **Conexión a Wi-Fi en BeagleBone Black (Debian 11.8 IOT)**

Este tutorial explica cómo conectar un BeagleBone Black (BBB) a una red Wi-Fi utilizando un adaptador USB Wi-Fi con el chipset **RTL8188EU**. Se asume que estás utilizando la imagen **Debian 11.8 (Bullseye IOT)** descargada desde el [sitio oficial de BeagleBoard](https://forum.beagleboard.org/t/debian-11-x-bullseye-monthly-snapshot-2023-09-02/31280).

Este tutorial fue replicado por última vez el 29 de enero del 2025
---

## **Requisitos**
1. BeagleBone Black con Debian 11.8 IOT instalado.
2. Adaptador USB Wi-Fi compatible con el chipset **RTL8188EU** (Realtek Semiconductor Corp. RTL8188EUS 802.11n Wireless Network Adapter
 fue usado para este caso).
3. Acceso a una terminal en el BBB (SSH o consola directa).

---

## **Pasos para Conectar a Wi-Fi**

### **1. Verifica que el Adaptador Wi-Fi sea Reconocido**
Conecta el adaptador USB Wi-Fi al BBB y verifica si el sistema lo reconoce.

1. Lista los dispositivos USB conectados:
   ```bash
   lsusb
   ```
   - Deberías ver una entrada relacionada con tu adaptador Wi-Fi, por ejemplo:
     ```
     Bus 001 Device 004: ID 0bda:8176 Realtek Semiconductor Corp. RTL8188EUS 802.11n Wireless Network Adapter
     ```

2. Verifica si el adaptador aparece como una interfaz de red:
   ```bash
   ip a
   ```
   - Busca una interfaz como `wlan0`. Si no aparece, el adaptador no está siendo reconocido.

---

### **2. Asegúrate de que el Módulo del Controlador esté Cargado**
El controlador para el chipset **RTL8188EU** debe estar cargado en el kernel.

1. Verifica si el módulo `r8188eu` está cargado:
   ```bash
   lsmod | grep r8188eu
   ```
   - Si no aparece, carga el módulo manualmente:
     ```bash
     sudo modprobe r8188eu
     ```

2. Verifica que la interfaz Wi-Fi esté activa:
   ```bash
   ip a
   ```
   - Si la interfaz `wlan0` no está activa (`UP`), actívala:
     ```bash
     sudo ip link set wlan0 up
     ```

---

### **3. Conéctate a la Red Wi-Fi usando `connman`**
`connman` es un gestor de conexiones ligero que viene preinstalado en Debian 11.8 IOT.

1. Inicia `connmanctl` en modo interactivo:
   ```bash
   sudo connmanctl
   ```

2. Habilita el agente de conexión:
   ```bash
   connmanctl> agent on
   ```

3. Habilita Wi-Fi:
   ```bash
   connmanctl> enable wifi
   ```

4. Escanea las redes Wi-Fi disponibles:
   ```bash
   connmanctl> scan wifi
   ```

5. Lista las redes detectadas:
   ```bash
   connmanctl> services
   ```
   - Verás una lista de redes con nombres como `wifi_<SSID>_<tipo_de_seguridad>`.

6. Conéctate a la red deseada:
   ```bash
   connmanctl> connect wifi_<SSID>_<tipo_de_seguridad>
   ```
   - Si la red está protegida, se te pedirá que ingreses la contraseña. 
   - Si tienes problemas de Error /net/connman/service/wifi_ ... _managed_psk: Not registered haz uso del comando

    ```bash
    connmanctl> agent on
    ```
y vuelve a intentar el paso 5 y 6.

7. Verifica la conexión:
   ```bash
   connmanctl> state
   ```

8. Sal de `connmanctl`:
   ```bash
   connmanctl> quit
   ```

---

### **4. Verifica la Conexión**
1. Verifica si la interfaz Wi-Fi tiene una dirección IP:
   ```bash
   ip a show wlan0
   ```

2. Prueba la conexión con un `ping`:
   ```bash
   ping google.com
   ```

---
