/**
 * Contract: Homepage Redirect Component
 *
 * Purpose: Redirect site root (/) to introduction page (/intro)
 *
 * Location: src/pages/index.jsx
 *
 * Why Needed:
 * - With routeBasePath: '/', docs are served at root
 * - But exact path '/' doesn't automatically show intro page
 * - This redirect ensures users accessing '/' land on '/intro'
 *
 * How It Works:
 * - Docusaurus pages in src/pages/ override doc routes
 * - index.jsx at src/pages/index.jsx serves path '/'
 * - Uses @docusaurus/router Redirect component
 * - Client-side redirect (instant, no server config needed)
 *
 * Testing:
 * - Access http://localhost:3000/book-ai/ → should redirect to /book-ai/intro
 * - Check browser URL changes to /book-ai/intro
 * - Verify introduction page content displays
 *
 * Alternative:
 * - If you want a custom landing page, replace Redirect with custom React component
 * - See: https://docusaurus.io/docs/creating-pages
 */

import React from 'react';
import { Redirect } from '@docusaurus/router';

export default function Home() {
  // Redirect root path to introduction page
  // With baseUrl '/book-ai/', this redirects:
  // http://localhost:3000/book-ai/ → http://localhost:3000/book-ai/intro
  return <Redirect to="/intro" />;
}

/**
 * Validation:
 *
 * 1. Verify @docusaurus/router is available (included in Docusaurus core)
 * 2. Test redirect works locally: npm start, access root, verify URL changes
 * 3. Test redirect works in production build: npm run build, serve build/, test root
 * 4. Confirm no redirect loop (intro page should not redirect back to /)
 */
