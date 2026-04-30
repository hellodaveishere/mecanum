/**
 * SERVER NODE.JS PER GESTIRE PIÙ SCRIPT
 * --------------------------------------
 * Funzionalità:
 *  - Serve la pagina web dalla cartella ../web
 *  - Usa lanciaScript.html come pagina principale
 *  - Controlla che lanciaScript.html esista prima di inviarlo
 *  - Gestisce più script Python (start/stop/log)
 *
 * COME LANCIARE IL SERVER:
 *   cd project/webserver
 *   npm install express        (solo la prima volta)
 *   node server.js
 *
 * Il server sarà disponibile su:
 *   http://localhost:8000
 */

const express = require('express');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');

const app = express();
const PORT = 8000;

// Percorsi principali
const WEB_DIR = path.join(__dirname, '../web');
const SCRIPTS_DIR = path.join(WEB_DIR, 'scripts');

// Whitelist degli script gestibili
const scripts = {
  start_mecanum: "start_mecanum.sh",
  script1: "script1.py",
  script2: "script2.py",
  script3: "script3.py"
};

// Stato dei processi attivi
const running = {};

console.log("Serving static files from:", WEB_DIR);

// per avere index automatico (index.html)
//app.use(express.static(WEB_DIR));

// Disattiviamo l'index automatico (index.html)
app.use(express.static(WEB_DIR, { index: false }));



/* ============================================================
   PAGINA PRINCIPALE (lanciaScript.html)
   ============================================================ */
app.get('/', (req, res) => {
  const file = path.join(WEB_DIR, 'lanciaScript.html');

  // Controllo che il file esista
  if (fs.existsSync(file)) {
    return res.sendFile(file);
  }

  // Se manca, mostriamo un errore chiaro
  res
    .status(500)
    .send("ERRORE: Il file lanciaScript.html non esiste in /web/");
});



/* ============================================================
   START SCRIPT
   ============================================================ */
app.post('/start/:name', (req, res) => {
  const name = req.params.name;

  if (!scripts[name]) {
    return res.status(404).send("Script non trovato");
  }

  if (running[name]) {
    return res.send(`Lo script '${name}' è già in esecuzione (PID ${running[name].pid})`);
  }

  const scriptPath = path.join(SCRIPTS_DIR, scripts[name]);
  const logFile = path.join(SCRIPTS_DIR, `${name}.log`);

  // Reset log
  fs.writeFileSync(logFile, "");

  // Determina se è script Python o Bash
  let cmd, args;

  if (scriptPath.endsWith(".py")) {
    cmd = "python3";
    args = ["-u", scriptPath];
  } else if (scriptPath.endsWith(".sh")) {
    cmd = "bash";
    args = [scriptPath];
  } else {
    return res.status(400).send("Tipo di script non supportato");
  }

  // Avvio processo
  const child = spawn(cmd, args, {
    cwd: SCRIPTS_DIR,
    shell: false,
    detached: true
  });



  running[name] = {
    process: child,
    pid: child.pid,
    logFile
  };

  child.stdout.on("data", d => fs.appendFileSync(logFile, d.toString()));
  child.stderr.on("data", d => fs.appendFileSync(logFile, d.toString()));

  child.on("close", code => {
    fs.appendFileSync(logFile, `\nProcesso terminato con codice ${code}\n`);
    delete running[name];
  });

  res.send(`Script '${name}' avviato con PID ${child.pid}`);
});



/* ============================================================
   STOP SCRIPT (SIGINT)
   ============================================================ */
app.post('/stop/:name', (req, res) => {
  const name = req.params.name;

  if (!running[name]) {
    return res.send(`Lo script '${name}' non è in esecuzione`);
  }

  const pid = running[name].pid;

  try {
    process.kill(pid, "SIGINT");
    res.send(`SIGINT inviato allo script '${name}' (PID ${pid})`);
  } catch (err) {
    res.send(`Errore nell'invio di SIGINT: ${err.message}`);
  }
});



/* ============================================================
   LOG SCRIPT
   ============================================================ */
app.get('/log/:name', (req, res) => {
  const name = req.params.name;

  if (!scripts[name]) {
    return res.status(404).send("Script non trovato");
  }

  const logFile = path.join(SCRIPTS_DIR, `${name}.log`);

  if (!fs.existsSync(logFile)) {
    return res.send("Nessun log disponibile");
  }

  res.setHeader("Cache-Control", "no-store");
  res.type("text/plain").send(fs.readFileSync(logFile, "utf8"));
});



/* ============================================================
   AVVIO SERVER
   ============================================================ */
app.listen(PORT, () => {
  console.log(`Web server attivo su http://localhost:${PORT}`);
});
