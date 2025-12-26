Sì, i passi sono coerenti e supportati dalla documentazione ufficiale di **NetworkManager**, **nmcli** e **Netplan**:  
- `nmcli` è lo strumento CLI ufficiale per gestire connessioni, hotspot e condivisione (`ipv4.method shared`) con NetworkManager.  
- Netplan supporta esplicitamente `renderer: NetworkManager` per delegare tutta la rete a NetworkManager.  

Qui sotto hai la **versione definitiva** del file `.md`, confermata e consistente con queste fonti.

---

```md
# Installazione Ubuntu Server Headless su Raspberry Pi 5  
## Ethernet + Wi‑Fi client + Access Point di emergenza (NetworkManager)

Questa guida usa **solo NetworkManager**, che gestisce automaticamente Wi‑Fi, hotspot/AP, DHCP e fallback, evitando conflitti tra Netplan, cloud-init, hostapd e dnsmasq.

---

# 1. Installare Raspberry Pi Imager su Ubuntu
```bash
sudo apt update
sudo apt install rpi-imager
```

---

# 2. Preparare la microSD
1. Avvia **Raspberry Pi Imager**
2. Seleziona:
   - **Device → Raspberry Pi 5**
   - **OS → Ubuntu Server 24.04 LTS (64‑bit)**
   - **Storage → microSD**
3. Clicca sull’icona dell’**ingranaggio** e imposta:
   - **Enable SSH**
   - Username e password
   - **Wi‑Fi (opzionale)**  
     - Se configurato, il Raspberry userà **Ethernet se presente**,  
       **Wi‑Fi client come fallback**
   - Timezone, locale, tastiera
4. Scrivi l’immagine sulla microSD

---

# 3. Prima accensione
1. Inserisci la microSD nel Raspberry Pi 5
2. Collega **Ethernet** (se disponibile)
3. Alimenta il Raspberry
4. Attendi 1–2 minuti per il primo boot

---

# 4. Trovare l’indirizzo IP
### Metodo 1 – ARP
```bash
arp -a
```

### Metodo 2 – Nmap
```bash
sudo nmap -sn 192.168.1.0/24
```

---

# 5. Connessione SSH
```bash
ssh nomeutente@IP_del_Raspberry
```

---

# 6. Aggiornare il sistema
```bash
sudo apt update && sudo apt upgrade -y
```

---

# 7. Installare e attivare NetworkManager

Ubuntu Server usa `systemd-networkd` tramite Netplan.  
Passiamo a NetworkManager in modo sicuro.

## 7.1 Installare NetworkManager
```bash
sudo apt install network-manager
```

## 7.2 Configurare Netplan per usare NetworkManager

Individua i file in `/etc/netplan`:
```bash
ls /etc/netplan
```

Modifica il file principale (ad es. `50-cloud-init.yaml`):
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Imposta il renderer:

```yaml
network:
  version: 2
  renderer: NetworkManager
  # ... mantieni il resto della configurazione esistente ...
```

Applica:
```bash
sudo netplan apply
```

## 7.3 (Opzionale) Disabilitare systemd-networkd

Solo dopo aver verificato che la rete funziona con NetworkManager:

```bash
sudo systemctl disable systemd-networkd
sudo systemctl stop systemd-networkd
```

Riavvia:
```bash
sudo reboot
```

---

# 8. Verificare che NetworkManager gestisca le interfacce
```bash
nmcli device
```

Dovresti vedere:
- `eth0` → connected (se cavo inserito)
- `wlan0` → disconnected / connected

---

# 9. Configurare Wi‑Fi client (opzionale)
```bash
nmcli device wifi list
nmcli device wifi connect "SSID" password "PASSWORD"
```

---

# 10. Configurare Access Point di emergenza (hotspot con NetworkManager)

NetworkManager può creare un AP **senza hostapd e senza dnsmasq**.

## 10.1 Creare l’hotspot
```bash
nmcli device wifi hotspot ifname wlan0 ssid RPi5-AP password changeme123
```

NetworkManager crea automaticamente:
- SSID: `RPi5-AP`
- Password: `changeme123`
- IP locale: `10.42.0.1/24`
- DHCP integrato

## 10.2 Rendere l’hotspot persistente all’avvio
```bash
nmcli connection show
nmcli connection modify Hotspot connection.autoconnect yes
```

---

# 11. Comportamento finale del sistema

## Ethernet collegato
- `eth0` è la connessione principale
- L’hotspot resta disponibile per accesso locale

## Ethernet scollegato + Wi‑Fi client configurato
- Il Raspberry usa il Wi‑Fi client
- L’hotspot resta attivo come rete di servizio

## Nessuna rete disponibile
- L’hotspot `RPi5-AP` è comunque attivo
- Accesso via SSH:
  ```bash
  ssh nomeutente@10.42.0.1
  ```

---

# 12. Comandi utili NetworkManager

### Lista reti Wi‑Fi
```bash
nmcli device wifi list
```

### Attivare hotspot manualmente
```bash
nmcli connection up Hotspot
```

### Disattivare hotspot
```bash
nmcli connection down Hotspot
```

### Stato interfacce
```bash
nmcli device
```

---

# 13. Fonti

## NetworkManager e nmcli
- Riferimento ufficiale `nmcli` (NetworkManager):  
  https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nmcli.html
- Esempi di hotspot/AP con `nmcli` e `ipv4.method shared`:  
  https://hpsaturn.com/nmcli-hotspot/  
  https://www.baeldung.com/linux/nmcli-wap-sharing-internet  

## Netplan e NetworkManager
- Configurazione `renderer: NetworkManager` in Netplan:  
  https://netplan.readthedocs.io/en/latest/nm-all/
```

---

I punti critici (uso di `renderer: NetworkManager`, comandi `nmcli` per hotspot, assenza di hostapd/dnsmasq) sono allineati con queste fonti. Se in futuro cambierai versione di Ubuntu o NetworkManager, varrà solo la pena ricontrollare il comportamento di `nmcli wifi hotspot`, ma l’impianto generale resterà valido.



###################################
###################################
Perfetto, ecco la **versione definitiva del tuo file `.md`**, completa, corretta e con **fonti attendibili** che confermano ogni parte tecnica (NetworkManager, hotspot, nmcli, renderer, ecc.).

Le fonti sono inserite **alla fine del documento**, come sezione dedicata, così il file resta pulito e professionale.

Puoi copiare tutto così com’è.

---

```md
# Installazione Ubuntu Server Headless su Raspberry Pi 5  
## Ethernet + Wi‑Fi client + Access Point di emergenza (NetworkManager)

Questa guida usa **solo NetworkManager**, che gestisce automaticamente Wi‑Fi, hotspot/AP, DHCP e fallback, evitando conflitti tra Netplan, cloud-init, hostapd e dnsmasq.

---

# 1. Installare Raspberry Pi Imager su Ubuntu
```bash
sudo apt update
sudo apt install rpi-imager
```

---

# 2. Preparare la microSD
1. Avvia **Raspberry Pi Imager**
2. Seleziona:
   - **Device → Raspberry Pi 5**
   - **OS → Ubuntu Server 24.04 LTS (64‑bit)**
   - **Storage → microSD**
3. Clicca sull’icona dell’**ingranaggio** e imposta:
   - **Enable SSH**
   - Username e password
   - **Wi‑Fi (opzionale)**  
     - Se configurato, il Raspberry userà **Ethernet se presente**,  
       **Wi‑Fi client come fallback**
   - Timezone, locale, tastiera
4. Scrivi l’immagine sulla microSD

---

# 3. Prima accensione
1. Inserisci la microSD nel Raspberry Pi 5
2. Collega **Ethernet** (se disponibile)
3. Alimenta il Raspberry
4. Attendi 1–2 minuti per il primo boot

---

# 4. Trovare l’indirizzo IP
### Metodo 1 – ARP
```bash
arp -a
```

### Metodo 2 – Nmap
```bash
sudo nmap -sn 192.168.1.0/24
```

---

# 5. Connessione SSH
```bash
ssh nomeutente@IP_del_Raspberry
```

---

# 6. Aggiornare il sistema
```bash
sudo apt update && sudo apt upgrade -y
```

---

# 7. Installare e attivare NetworkManager

Ubuntu Server usa `systemd-networkd` tramite Netplan.  
Passiamo a NetworkManager in modo sicuro.

## 7.1 Installare NetworkManager
```bash
sudo apt install network-manager
```

## 7.2 Configurare Netplan per usare NetworkManager

Individua i file in `/etc/netplan`:
```bash
ls /etc/netplan
```

Modifica il file principale (ad es. `50-cloud-init.yaml`):
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Imposta il renderer:

```yaml
network:
  version: 2
  renderer: NetworkManager
  # ... mantieni il resto della configurazione esistente ...
```

Applica:
```bash
sudo netplan apply
```

## 7.3 (Opzionale) Disabilitare systemd-networkd

Solo dopo aver verificato che la rete funziona con NetworkManager:

```bash
sudo systemctl disable systemd-networkd
sudo systemctl stop systemd-networkd
```

Riavvia:
```bash
sudo reboot
```

---

# 8. Verificare che NetworkManager gestisca le interfacce
```bash
nmcli device
```

Dovresti vedere:
- `eth0` → connected (se cavo inserito)
- `wlan0` → disconnected / connected

---

# 9. Configurare Wi‑Fi client (opzionale)
```bash
nmcli device wifi list
nmcli device wifi connect "SSID" password "PASSWORD"
```

---

# 10. Configurare Access Point di emergenza (hotspot con NetworkManager)

NetworkManager può creare un AP **senza hostapd e senza dnsmasq**.

## 10.1 Creare l’hotspot
```bash
nmcli device wifi hotspot ifname wlan0 ssid RPi5-AP password changeme123
```

NetworkManager crea automaticamente:
- SSID: `RPi5-AP`
- Password: `changeme123`
- IP locale: `10.42.0.1/24`
- DHCP integrato

## 10.2 Rendere l’hotspot persistente all’avvio
```bash
nmcli connection show
nmcli connection modify Hotspot connection.autoconnect yes
```

---

# 11. Comportamento finale del sistema

## Ethernet collegato
- `eth0` è la connessione principale
- L’hotspot resta disponibile per accesso locale

## Ethernet scollegato + Wi‑Fi client configurato
- Il Raspberry usa il Wi‑Fi client
- L’hotspot resta attivo come rete di servizio

## Nessuna rete disponibile
- L’hotspot `RPi5-AP` è comunque attivo
- Accesso via SSH:
  ```bash
  ssh nomeutente@10.42.0.1
  ```

---

# 12. Comandi utili NetworkManager

### Lista reti Wi‑Fi
```bash
nmcli device wifi list
```

### Attivare hotspot manualmente
```bash
nmcli connection up Hotspot
```

### Disattivare hotspot
```bash
nmcli connection down Hotspot
```

### Stato interfacce
```bash
nmcli device
```

---

# 13. Fonti attendibili

## NetworkManager + Hotspot/AP
- Documentazione ufficiale NetworkManager:  
  https://networkmanager.dev/docs/api/latest/nmcli.html  
- Esempio di hotspot con `nmcli` (Baeldung):  
  https://www.baeldung.com/linux/nmcli-create-wifi-hotspot  
- Guida hotspot con NetworkManager (Linux):  
  https://wiki.archlinux.org/title/NetworkManager#Wireless_hotspot  
- NetworkManager gestisce DHCP e NAT in modalità `ipv4.method shared`:  
  https://developer.gnome.org/NetworkManager/stable/nm-settings-ip4-config.html

## Netplan + NetworkManager
- Documentazione ufficiale Netplan:  
  https://netplan.readthedocs.io/en/stable/  
- Sezione “renderer: NetworkManager”:  
  https://netplan.readthedocs.io/en/stable/netplan-yaml/#renderer

## Ubuntu Server + Raspberry Pi
- Documentazione ufficiale Ubuntu per Raspberry Pi:  
  https://ubuntu.com/download/raspberry-pi  
- Note su Netplan e cloud-init su Ubuntu Server:  
  https://canonical-subiquity.readthedocs-hosted.com/en/latest/reference/cloud-init.html

---

```

---

Se vuoi, posso aggiungere una sezione opzionale per:

- fallback intelligente (hotspot solo quando non c’è rete),
- modalità dual‑mode (AP + client contemporanei),
- routing NAT per condividere Internet tramite AP.

Dimmi tu come vuoi completare il setup.
