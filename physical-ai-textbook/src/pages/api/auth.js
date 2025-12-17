// Docusaurus API route for authentication
// This will serve as a proxy to the backend authentication service

export default async function handler(req, res) {
  // Set CORS headers
  res.setHeader('Access-Control-Allow-Credentials', true);
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET,OPTIONS,PATCH,DELETE,POST,PUT');
  res.setHeader(
    'Access-Control-Allow-Headers',
    'X-CSRF-Token, X-Requested-With, Accept, Accept-Version, Content-Length, Content-MD5, Content-Type, Date, X-Api-Version'
  );

  if (req.method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  // Proxy the request to the backend auth service
  const { method, body } = req;

  try {
    // Forward the request to the backend service
    const backendResponse = await fetch(`http://localhost:8000/auth/${req.query.auth || ''}`, {
      method,
      headers: {
        'Content-Type': 'application/json',
      },
      body: method !== 'GET' && body ? JSON.stringify(body) : undefined,
    });

    const data = await backendResponse.json();
    res.status(backendResponse.status).json(data);
  } catch (error) {
    console.error('Auth API error:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
}