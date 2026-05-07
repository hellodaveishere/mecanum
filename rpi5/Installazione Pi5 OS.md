"nmcli dev wifi show-password" shows the Wi-Fi name and password.


=====
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
   - **OS → Ubuntu Server 24.04 LTS (64‑bit)**  (Per compatibilità con ROS2!!!!!)
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

NOTA: non fa l'upgrade dell'OS
```

---

```markdown
# Passare a NetworkManager su Ubuntu Server 24.04 LTS (Noble) in modo sicuro

Questa procedura è pensata per **Ubuntu Server 24.04 LTS (64‑bit)**, dove il renderer predefinito è **systemd-networkd** tramite **Netplan**.  
L’obiettivo è:

- usare **NetworkManager** come renderer principale
- **non perdere la connettività**, soprattutto se sei in SSH
- evitare modifiche pericolose a `systemd-networkd`  

Le indicazioni sono allineate alla documentazione Netplan/Ubuntu e alla struttura di networking di Ubuntu 24.04.   [Ubuntu Manpage Repository](https://manpages.ubuntu.com/manpages/noble/man5/netplan.5.html)  [server.hk](https://server.hk/blog/ubuntu-networking-internals-netplan-networkmanager-and-kernel-interfaces/)  

---

## 0. Prima di iniziare

- Se sei collegato via **SSH**, pianifica un accesso alternativo (console, IPMI, ecc.).
- Esegui un backup dei file in `/etc/netplan/`:

```bash
sudo cp -a /etc/netplan /etc/netplan.backup.$(date +%F)
```

---

## 1. Installare NetworkManager

Su Ubuntu Server 24.04, NetworkManager non è installato di default.

```bash
sudo apt update
sudo apt install -y network-manager
```

> NetworkManager è uno dei due renderer supportati da Netplan (insieme a systemd-networkd).   [Ubuntu Manpage Repository](https://manpages.ubuntu.com/manpages/noble/man5/netplan.5.html)  [server.hk](https://server.hk/blog/ubuntu-networking-internals-netplan-networkmanager-and-kernel-interfaces/)  

---

## 2. Identificare il file Netplan attivo

I file possono chiamarsi, ad esempio:

- `00-installer-config.yaml`
- `50-cloud-init.yaml`
- `01-netcfg.yaml`

Elenca i file:

```bash
ls /etc/netplan
```

Scegli il file principale (di solito quello con numero più alto o quello che contiene la tua interfaccia, es. `eth0`, `ens3`, `enp1s0`, ecc.) e aprilo:

```bash
sudo nano /etc/netplan/00-installer-config.yaml
# oppure
sudo nano /etc/netplan/50-cloud-init.yaml
```

---

## 3. Impostare NetworkManager come renderer

Nel file scelto, assicurati che la struttura sia simile a:

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: true
```

Note importanti:

- **Non rimuovere** la sezione `ethernets:` (o `wifis:` ecc.) se è già presente:  
  mantieni IP statici, DHCP, DNS, gateway, ecc.  
- Cambia solo il valore di `renderer:` in `NetworkManager`.  
- Il nome dell’interfaccia (`eth0`, `ens3`, `enp0s3`, ecc.) deve corrispondere a quello reale (`ip a` per verificarlo).

> Netplan usa `renderer` per decidere se usare systemd-networkd o NetworkManager come backend.   [Ubuntu Manpage Repository](https://manpages.ubuntu.com/manpages/noble/man5/netplan.5.html)  [server.hk](https://server.hk/blog/ubuntu-networking-internals-netplan-networkmanager-and-kernel-interfaces/)  

---

## 4. Applicare la configurazione in modo sicuro

Se sei su un server (specie via SSH), usa **`netplan try`** invece di `netplan apply`:

```bash
sudo netplan try
```

- Il sistema applica la nuova configurazione.
- Hai un timeout (es. 120 secondi) per confermare.
- Se **confermi**, la configurazione resta.
- Se **non confermi** (perché hai perso la connessione), Netplan **ripristina automaticamente** la configurazione precedente.

> `netplan try` è il metodo raccomandato per evitare di bloccarsi fuori dal server quando si cambia renderer o IP.   [cr0x.net](https://cr0x.net/en/ubuntu-netplan-changes-not-applying/)  [Ubuntu Manpage Repository](https://manpages.ubuntu.com/manpages/noble/man5/netplan.5.html)  

Se sei in locale (console) e vuoi applicare direttamente:

```bash
sudo netplan apply
```

---

## 5. Verificare che NetworkManager gestisca la rete

Dopo `netplan try`/`apply`, controlla:

```bash
nmcli device
```

Dovresti vedere la tua interfaccia (es. `eth0`) in stato `connected` o `connecting`.

Controlla anche:

```bash
ip a
ip route
ping -c 3 1.1.1.1
ping -c 3 google.com
```

Se:

- hai IP corretto
- hai route
- risolvi DNS

allora NetworkManager sta gestendo correttamente la rete.

---

## 6. Gestione di systemd-networkd

### 6.1 Cosa NON fare

**Sconsigliato** disabilitare brutalmente `systemd-networkd.service`:

```bash
# NON consigliato:
sudo systemctl disable systemd-networkd
sudo systemctl stop systemd-networkd
```

Questo può avere effetti collaterali su boot, DNS e servizi che si aspettano networkd.   [server.hk](https://server.hk/blog/ubuntu-networking-internals-netplan-networkmanager-and-kernel-interfaces/)  

### 6.2 Cosa puoi fare in sicurezza

Se vuoi evitare che il sistema aspetti networkd all’avvio, puoi disabilitare solo il servizio “wait-online”:

```bash
sudo systemctl disable systemd-networkd-wait-online.service
```

Questo riduce i tempi di boot senza interferire con Netplan/NetworkManager.

---

## 7. Riavvio e verifica finale

Quando sei soddisfatto del funzionamento:

```bash
sudo reboot
```

Dopo il riavvio:

```bash
nmcli device
ip a
ip route
ping -c 3 1.1.1.1
ping -c 3 google.com
```

Se tutto è ok, la migrazione a NetworkManager è completata.

---

## 8. Come tornare indietro a systemd-networkd (rollback)

Se vuoi ripristinare la situazione originale:

1. Modifica di nuovo il file Netplan:

```bash
sudo nano /etc/netplan/<nomefile>.yaml
```

Rimetti:

```yaml
network:
  version: 2
  renderer: networkd
  # ... resto invariato ...
```

2. Applica in modo sicuro:

```bash
sudo netplan try
# oppure
sudo netplan apply
```

3. (Opzionale) Se avevi disabilitato `systemd-networkd-wait-online.service` e vuoi ripristinarlo:

```bash
sudo systemctl enable systemd-networkd-wait-online.service
```

4. Riavvia:

```bash
sudo reboot
```

---

## 9. Riepilogo rapido (comandi chiave)

```bash
# Backup
sudo cp -a /etc/netplan /etc/netplan.backup.$(date +%F)

# Installare NetworkManager
sudo apt update
sudo apt install -y network-manager

# Modificare Netplan (esempio)
sudo nano /etc/netplan/00-installer-config.yaml
# -> renderer: NetworkManager

# Applicare in modo sicuro
sudo netplan try

# Verifica
nmcli device
ip a
ip route
ping -c 3 1.1.1.1
ping -c 3 google.com

# (Opzionale) disabilitare solo il wait-online
sudo systemctl disable systemd-networkd-wait-online.service
```

---

Se vuoi, posso aggiungere una **sezione pronta-copia per IP statico** (es. server in LAN con IP fisso) usando NetworkManager via Netplan.  
```

=============== OLD =====================

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
