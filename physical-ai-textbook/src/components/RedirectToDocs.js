import React from 'react';
import { Redirect } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function RedirectToDocs() {
  const { siteConfig } = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl;

  // Redirect to the docs intro page (which is the homepage based on slug: / in intro.md)
  return <Redirect to={`${baseUrl}docs/intro`} />;
}

export default RedirectToDocs;