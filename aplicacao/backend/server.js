const express = require('express');
const { Pool } = require('pg');
const axios = require('axios');
const cron = require('node-cron');
const cors = require('cors');
const bcrypt = require('bcryptjs');
const jwt = require('jsonwebtoken');

const app = express();
const SECRET_KEY = 'pi5-secret-key'; // Em produção, use env var

app.use(cors());
app.use(express.json());

// Logger de Requisições
app.use((req, res, next) => {
  console.log(`[${new Date().toLocaleTimeString()}] ${req.method} ${req.url}`);
  next();
});

const pool = new Pool({ connectionString: process.env.DATABASE_URL });

const initDB = async (retries = 5) => {
  while (retries) {
    try {
      // Tabela de Medições
      await pool.query(`
        CREATE TABLE IF NOT EXISTS measurements (
          id SERIAL PRIMARY KEY,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
          thingspeak_at TIMESTAMP WITH TIME ZONE UNIQUE,
          v_fonte FLOAT, i_fonte FLOAT, p_fonte FLOAT,
          carga_perc FLOAT, v_buck FLOAT, i_acs FLOAT,
          health FLOAT, slope_v FLOAT,
          temp_fonte FLOAT, temp_mpu FLOAT, vibracao FLOAT,
          pgood INTEGER, lm2596_cc INTEGER, adc_ok INTEGER, 
          temp_ok INTEGER, mpu_ok INTEGER
        );
      `);
      // Tabela de Usuários
      await pool.query(`
        CREATE TABLE IF NOT EXISTS users (
          id SERIAL PRIMARY KEY,
          username TEXT UNIQUE NOT NULL,
          password TEXT NOT NULL,
          created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
        );
      `);

      // Garantir usuário admin inicial
      const adminExists = await pool.query('SELECT * FROM users WHERE username = $1', ['admin']);
      if (adminExists.rows.length === 0) {
        const hashedAdminPass = await bcrypt.hash('univesp', 10);
        await pool.query('INSERT INTO users (username, password) VALUES ($1, $2)', ['admin', hashedAdminPass]);
        console.log('👤 Usuário administrador padrão criado (admin / univesp).');
      }

      console.log('✅ Banco de dados pronto (Medições e Usuários).');
      return;
    } catch (err) {
      console.log(`❌ Erro no DB. Restam ${retries - 1} tentativas...`);
      retries -= 1;
      await new Promise(res => setTimeout(res, 5000));
    }
  }
  process.exit(1);
};

// Middleware de Autenticação
const authenticateToken = (req, res, next) => {
  const authHeader = req.headers['authorization'];
  const token = authHeader && authHeader.split(' ')[1];
  if (!token) return res.sendStatus(401);

  jwt.verify(token, SECRET_KEY, (err, user) => {
    if (err) return res.sendStatus(403);
    req.user = user;
    next();
  });
};

// Rotas de Auth
app.post('/api/register', async (req, res) => {
  try {
    const { username, password } = req.body;
    const hashedPassword = await bcrypt.hash(password, 10);
    await pool.query('INSERT INTO users (username, password) VALUES ($1, $2)', [username, hashedPassword]);
    res.status(201).json({ message: 'Usuário criado com sucesso!' });
  } catch (err) {
    res.status(400).json({ error: 'Usuário já existe ou dados inválidos.' });
  }
});

app.post('/api/login', async (req, res) => {
  const { username, password } = req.body;
  const result = await pool.query('SELECT * FROM users WHERE username = $1', [username]);
  const user = result.rows[0];

  if (user && await bcrypt.compare(password, user.password)) {
    const token = jwt.sign({ username: user.username }, SECRET_KEY, { expiresIn: '24h' });
    res.json({ token });
  } else {
    res.status(401).json({ error: 'Usuário ou senha incorretos.' });
  }
});

// Rotas Protegidas
app.get('/api/history', authenticateToken, async (req, res) => {
  try {
    const hours = req.query.hours ? parseInt(req.query.hours) : null;
    const limit = parseInt(req.query.limit) || 1000;
    let query = 'SELECT * FROM measurements ';
    const params = [limit];
    if (hours) query += `WHERE thingspeak_at >= NOW() - INTERVAL '${hours} hours' `;
    query += 'ORDER BY thingspeak_at DESC LIMIT $1';
    const result = await pool.query(query, params);
    res.json(result.rows);
  } catch (err) { res.status(500).json({ error: err.message }); }
});

app.get('/api/last', async (req, res) => {
  const result = await pool.query('SELECT thingspeak_at FROM measurements ORDER BY thingspeak_at DESC LIMIT 1');
  res.json(result.rows[0] || null);
});

app.get('/api/stats', async (req, res) => {
  const result = await pool.query('SELECT COUNT(*) as total FROM measurements');
  res.json(result.rows[0]);
});

// Worker do ThingSpeak (Continua público e automático)
const fetchAndStore = async () => {
  try {
    const ch1 = process.env.THINGSPEAK_CH1;
    const key1 = process.env.THINGSPEAK_KEY1;
    const ch2 = process.env.THINGSPEAK_CH2;
    const key2 = process.env.THINGSPEAK_KEY2;

    const [resp1, resp2] = await Promise.all([
      axios.get(`https://api.thingspeak.com/channels/${ch1}/feeds.json?results=1000&api_key=${key1}`),
      axios.get(`https://api.thingspeak.com/channels/${ch2}/feeds.json?results=1000&api_key=${key2}`)
    ]);

    const feeds1 = resp1.data.feeds || [];
    const feeds2 = resp2.data.feeds || [];

    let inserted = 0;
    let skipped = 0;

    for (const f1 of feeds1) {
      const t1 = new Date(f1.created_at).getTime();
      
      // Encontrar o feed no canal 2 mais próximo (janela de 5 minutos)
      const f2 = feeds2.find(f => {
        const t2 = new Date(f.created_at).getTime();
        return Math.abs(t1 - t2) < 300000; 
      });
      
      if (f2) {
        try {
          const query = `
            INSERT INTO measurements 
            (thingspeak_at, v_fonte, i_fonte, p_fonte, carga_perc, v_buck, i_acs, health, slope_v, 
             temp_fonte, temp_mpu, vibracao, pgood, lm2596_cc, adc_ok, temp_ok, mpu_ok)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14, $15, $16, $17)
            ON CONFLICT (thingspeak_at) DO NOTHING
          `;
          const res = await pool.query(query, [
            f1.created_at, parseFloat(f1.field1)||0, parseFloat(f1.field2)||0, parseFloat(f1.field3)||0,
            parseFloat(f1.field4)||0, parseFloat(f1.field5)||0, parseFloat(f1.field6)||0,
            parseFloat(f1.field7)||0, parseFloat(f1.field8)||0,
            parseFloat(f2.field1)||0, parseFloat(f2.field2)||0, parseFloat(f2.field3)||0,
            parseInt(f2.field4)||0, parseInt(f2.field5)||0, parseInt(f2.field6)||0,
            parseInt(f2.field7)||0, parseInt(f2.field8)||0
          ]);
          if (res.rowCount > 0) inserted++;
          else skipped++;
        } catch (dbErr) {
          console.error(`❌ DB Insert Error at ${f1.created_at}:`, dbErr.message);
        }
      }
    }
    
    console.log(`📊 [Sync] Resultado: ${inserted} inseridos, ${skipped} duplicados/pulados.`);
  } catch (err) { 
    console.error('⚠️ Sync Error:', err.message); 
  }
};

cron.schedule('* * * * *', fetchAndStore);

const PORT = 3000;
app.listen(PORT, async () => {
  await initDB();
  console.log(`🚀 Backend na porta ${PORT}`);
  fetchAndStore();
});
