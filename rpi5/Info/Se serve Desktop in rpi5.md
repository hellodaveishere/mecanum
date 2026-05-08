Sì, **`ros-jazzy-desktop` esiste anche per ARM64**—ma va distinta una cosa fondamentale:

- **come pacchetto Debian (`apt install ros-jazzy-desktop`) → SÌ, supporta arm64**
- **come immagine Docker “desktop-noble” preconfezionata → NO, non c’è variante arm64**

---

### 1. Pacchetto `ros-jazzy-desktop` (apt) ✅ ARM64

Nella doc ufficiale di Jazzy per Ubuntu 24.04 Noble:

- target: **amd64 e arm64**   [ROS Documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)  [ROS Documentation](https://docs.ros.org/en/rolling/Releases/Release-Jazzy-Jalisco.html)  
- installazione “Desktop (Recommended)”:

```bash
sudo apt install ros-jazzy-desktop
```

Quindi su **Ubuntu 24.04 arm64** (come base del tuo container o nativo su RPi5) il pacchetto `ros-jazzy-desktop` è **supportato e testato**.

---

### 2. Immagini Docker “desktop” ❌ ARM64 (nel tuo caso)

Le immagini tipo:

- `osrf/ros:jazzy-desktop-noble`
- `osrf/ros:jazzy-desktop-full-noble`

sono pubblicate su Docker Hub, ma i manifest che hai incrociato sono **solo `linux/amd64`**, da cui l’errore:

> `no match for platform in manifest: not found`

Quindi: **come Docker image pre-fatta, la variante desktop-noble non è disponibile per ARM64**.

---

### 3. Strategia corretta per Raspberry Pi 5

Per RPi5 la combinazione “pulita” è:

```dockerfile
FROM ros:jazzy-ros-base-noble   # immagine ufficiale, ARM64, Ubuntu 24.04

RUN apt update && apt install -y \
    ros-jazzy-desktop \
 && rm -rf /var/lib/apt/lists/*
```

Così ottieni:

- base **ARM64** supportata
- **Noble** garantito
- stack **desktop** ufficiale (RViz, demo, ecc.) via `ros-jazzy-desktop`

Se vuoi, ti scrivo un Dockerfile completo, pensato specificamente per RPi5 + Jazzy + i tuoi pacchetti.
