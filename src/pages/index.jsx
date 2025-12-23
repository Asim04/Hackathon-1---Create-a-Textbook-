/**
 * Homepage Redirect Component
 *
 * Purpose: Redirect root path (/) to the introduction page (/intro)
 *
 * Why needed:
 * - With routeBasePath: '/', docs are served at root
 * - But exact path '/' doesn't automatically show intro page
 * - This redirect ensures users accessing '/' land on '/intro'
 *
 * How it works:
 * - Docusaurus pages in src/pages/ override doc routes
 * - index.jsx at src/pages/index.jsx serves path '/'
 * - Uses @docusaurus/router Redirect component for client-side redirect
 *
 * Testing:
 * - Access http://localhost:3000/book-ai/ → should redirect to /book-ai/intro
 * - Check browser URL changes to /book-ai/intro
 * - Verify introduction page content displays
 */

import React from 'react';
import { Redirect } from '@docusaurus/router';

export default function Home() {
  // Redirect root path to introduction page
  // With baseUrl '/book-ai/', this redirects:
  // http://localhost:3000/book-ai/ → http://localhost:3000/book-ai/intro
  return <Redirect to="/intro" />;
}
