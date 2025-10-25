const express = require('express');
const path = require('path');
const app = express();
const PORT = 8000;

// Serve la cartella web
app.use(express.static(path.join(__dirname, '../web')));

app.listen(PORT, () => {
  console.log(`Web server attivo su http://localhost:${PORT}`);
});
