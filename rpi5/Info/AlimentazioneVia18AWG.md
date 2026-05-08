Sul **Raspberry Pi 5** i problemi di alimentazione *si vedono eccome* nei log, ma devi sapere **esattamente cosa cercare**, perché i messaggi non sono sempre ovvi.

Ti faccio una guida **precisa e completa** su tutti i segnali che il Pi 5 dà quando la corrente non è sufficiente.

---

## 🔥 1. Il segnale più importante: *undervoltage detected*
Il Pi 5, come i modelli precedenti, monitora costantemente la tensione del rail 5 V.

Quando scende sotto ~4.65 V, nei log compare:

```
Under-voltage detected!
```

oppure:

```
Voltage drop below safe threshold
```

Può apparire anche come:

```
throttling due to under-voltage
```

Questi messaggi sono inequivocabili: **il Pi non sta ricevendo abbastanza corrente**.

---

## ⚡ 2. Messaggi di throttling (CPU/GPU rallentata)
Se la tensione cala, il firmware riduce la frequenza della CPU/GPU per evitare crash.

Nei log vedi:

```
CPU throttled due to power issues
```

oppure:

```
Frequency capped due to under-voltage
```

---

## 🧊 3. Messaggi combinati: undervoltage + temperature
Quando il Pi soffre di alimentazione, spesso si scalda di più (perché la CPU entra/uscita dal throttling).

Nei log puoi trovare:

```
Throttling: under-voltage, temperature
```

---

## 🔍 4. Dove guardare esattamente

### **A. Log del kernel**
```bash
dmesg | grep -i voltage
dmesg | grep -i under
dmesg | grep -i throttle
```

### **B. Log di boot**
```bash
sudo journalctl -b | grep -i voltage
```

### **C. Stato corrente del firmware**
```bash
vcgencmd get_throttled
```

Interpretazione rapida:

- `0x0` → tutto ok  
- `0x50000` → undervoltage *in passato*  
- `0x1` → undervoltage *attuale*  
- `0x2` → throttling *attuale*  

---

## 🧪 5. Come testare se la tua alimentazione 18 AWG + UBEC regge davvero

### Test 1 — Stress CPU
```bash
sudo apt install stress-ng
stress-ng --cpu 4 --timeout 30s
```

Poi controlla:

```bash
vcgencmd get_throttled
```

### Test 2 — Stress USB (ventole, periferiche)
Collega tutto, poi:

```bash
dmesg | grep -i voltage
```

Se non compare nulla → **alimentazione stabile**.

---

## 🟢 6. Conclusione per il tuo caso specifico (18 AWG, 10–15 cm, UBEC Henge)

Con questa configurazione, **se i log sono puliti**, significa che:

- il tuo UBEC regge il carico  
- i cavi 18 AWG sono adeguati  
- il Pi 5 non sta andando in undervoltage  
- puoi usare ventole USB + CPU sotto carico senza problemi  

---

## Vuoi che ti prepari un comando unico che:
- stressa CPU  
- stressa USB  
- monitora in tempo reale undervoltage  
- e ti dice se la tua alimentazione è perfetta?

Posso generarti uno script pronto all’uso.
