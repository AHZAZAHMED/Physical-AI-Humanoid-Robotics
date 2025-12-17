# Docusaurus Routing Configuration Research

**Feature**: 1-fix-platform-issues
**Research Area**: Fix routing and 404 errors for intro page
**Date**: 2025-12-16

## Objective
Research Docusaurus routing configuration to properly handle the intro page slug and navbar links to prevent 404 errors.

## Decision: Configure Slug in Doc Page and Navbar Link
**Rationale**: Docusaurus uses a combination of file-based routing and explicit slug configuration. Setting the correct slug in the intro document and configuring the navbar link properly will ensure consistent routing behavior.

**Alternatives considered**:
1. **Custom routing with React Router**: More complex, unnecessary for basic documentation routing
2. **URL rewriting**: Would add complexity without clear benefits
3. **Custom plugin**: Overkill for simple routing fixes

## Technical Implementation

### Intro Page Configuration
The intro page (likely `intro.mdx`) needs a proper slug frontmatter to ensure it's served at `/docs/intro`:

```md
---
slug: /docs/intro
title: Introduction
---

# Introduction to Physical AI & Humanoid Robotics
```

### Navbar Configuration
In `docusaurus.config.ts`, the navbar configuration should point to the correct intro page:

```ts
navbar: {
  title: 'Physical AI Textbook',
  items: [
    {
      type: 'docSidebar',
      sidebarId: 'tutorialSidebar',
      position: 'left',
      label: 'Docs',
    },
    {
      to: '/docs/intro',  // This should point to the intro page
      label: 'Docs',
      position: 'left'
    },
    {
      href: 'https://github.com/AHZAZAHMED/physical-ai-textbook',
      label: 'GitHub',
      position: 'right',
    },
  ],
},
```

### Sidebar Configuration
Ensure the sidebar configuration in `sidebars.ts` properly references the intro document:

```ts
{
  tutorialSidebar: [
    'intro',  // This should match the id of your intro document
    // other sidebar items...
  ],
}
```

## Common Docusaurus Routing Issues
1. **Missing slug**: Documents without explicit slugs use the filename as the URL
2. **Incorrect navbar links**: Navbar items pointing to non-existent routes
3. **Sidebar mismatches**: Sidebar referencing documents that don't exist or have different IDs

## Implementation Path
1. Add proper slug to intro.mdx file
2. Update navbar configuration to point to `/docs/intro`
3. Verify sidebar configuration matches document IDs
4. Test all navigation paths for 404 errors
5. Verify that clicking the main title navigates to the correct page