const express = require('express');
const path = require('path');
const app = express();
const PORT = 8000;

// Serve tutti i file statici dalla cartella "web"
const staticPath = path.join(__dirname, '../web');
console.log("Serving static files from:", staticPath);
app.use(express.static(staticPath));

// Fallback per index.html
app.get('/', (req, res) => {
  res.sendFile(path.join(staticPath, 'index.html'));
});

// Avvia il server
app.listen(PORT, () => {
  console.log(`Web server attivo su http://localhost:${PORT}`);
});
