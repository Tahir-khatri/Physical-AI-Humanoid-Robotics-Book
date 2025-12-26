# Technical Implementation Guide: Futuristic Landing Page with Module Grid

This document provides technical recommendations for implementing the futuristic landing page, based on the design plan.

## 1. Visual Identity (CSS Variables & Global Styles)

### Background & Accent Colors:
The core visual identity will be set using CSS variables and applied to the global `body` or relevant container elements.

```css
/* In Book-Frontend/src/css/custom.css */

:root {
  /* Futuristic Dark Theme Base */
  --ifm-background-color: #050505; /* Deep black */
  --ifm-code-background: #0a0a15; /* Even deeper for code blocks */

  /* Accent Colors */
  --neon-purple: #7c3aed;
  --robotic-blue: #2563eb;

  /* Card Style */
  --card-background: rgba(15, 15, 25, 0.7); /* Dark glassmorphism */
  --card-border: 1px solid rgba(124, 58, 237, 0.3);
  --card-hover-glow: 0 0 15px rgba(124, 58, 237, 0.7); /* Purple glow */
}

/* Gradient background for hero/main content */
body {
  background: linear-gradient(180deg, #050505 0%, #0a0a15 100%);
}

/* Pulse Animation for CTA button */
@keyframes pulse {
  0% {
    transform: scale(1);
    box-shadow: 0 0 0 0 rgba(37, 99, 235, 0.7);
  }
  70% {
    transform: scale(1.05);
    box-shadow: 0 0 0 15px rgba(37, 99, 235, 0);
  }
  100% {
    transform: scale(1);
    box-shadow: 0 0 0 0 rgba(37, 99, 235, 0);
  }
}

.button--pulse {
  animation: pulse 2s infinite;
}
```

## 2. Module Grid Implementation

### ModuleGrid Component (`Book-Frontend/src/components/ModuleGrid/index.tsx`)

This component will display the 5 module cards. It will replace the `HomepageFeatures` component.

```jsx
import React from 'react';
import Link from '@docusaurus/Link';

const ModuleCards = [
  {
    title: 'Introduction',
    description: 'Overview of Physical AI and Humanoid Robotics.',
    link: '/docs/introduction/01-ros-2-overview',
  },
  {
    title: 'Module 1: The Robotic Nervous System',
    description: 'Deep dive into ROS 2 and its role in robotics.',
    link: '/docs/module-1/chapter-1',
  },
  {
    title: 'Module 2: The Digital Twin',
    description: 'Explore digital twin simulations and virtual environments.',
    link: '/docs/module-2/chapter-1',
  },
  {
    title: 'Module 3: The AI-Robot Brain',
    description: 'Understanding AI and perception for robotic intelligence.',
    link: '/docs/module-3/chapter-1',
  },
  {
    title: 'Module 4: Vision-Language-Action',
    description: 'Integrating LLMs with robotic perception and action.',
    link: '/docs/module-4/chapter-1',
  },
];

function ModuleCard({ title, description, link }) {
  return (
    <div className="module-card"> {/* Custom class for styling */}
      <h3>{title}</h3>
      <p>{description}</p>
      <Link to={link} className="button button--primary">Learn More</Link>
    </div>
  );
}

export default function ModuleGrid() {
  return (
    <section style={{ padding: '50px 0' }}>
      <div className="container" style={{ display: 'grid', gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))', gap: '20px' }}>
        {ModuleCards.map((card, idx) => (
          <ModuleCard key={idx} {...card} />
        ))}
      </div>
    </section>
  );
}
```

### CSS for Module Cards (`Book-Frontend/src/css/custom.css`)

```css
/* In Book-Frontend/src/css/custom.css */

.module-card {
  background: var(--card-background);
  border: var(--card-border);
  border-radius: 10px;
  padding: 20px;
  backdrop-filter: blur(5px); /* Glassmorphism effect */
  transition: all 0.3s ease;
  color: white; /* Ensure readability */
}

.module-card:hover {
  box-shadow: var(--card-hover-glow);
  transform: translateY(-5px);
}

.module-card h3 {
  color: var(--robotic-blue); /* Title color */
  font-size: 1.5rem;
  margin-bottom: 10px;
}

.module-card p {
  color: #e0e0e0;
  font-size: 1rem;
  margin-bottom: 20px;
}

.module-card .button {
  background-color: var(--robotic-blue);
  color: white;
  border-radius: 5px;
  padding: 8px 15px;
  text-decoration: none;
  transition: background-color 0.3s ease;
}

.module-card .button:hover {
  background-color: var(--neon-purple);
}
```

## 3. Custom Footer (Configuration & Styling)

### `docusaurus.config.ts` updates:

```javascript
// Inside themeConfig.footer
footer: {
  style: 'dark', // or 'light'
  links: [
    // ... existing links
    {
      title: 'Community',
      items: [
        { label: 'LinkedIn', href: 'https://www.linkedin.com/in/tahir-khatri-204039346/' },
        { label: 'Twitter', href: 'https://x.com/tahirkhatri926' },
        { label: 'Instagram', href: 'https://www.instagram.com/tahir_khatri21/' },
      ],
    },
    // ... other links
  ],
  copyright: `Copyright © 2026 Physical AI And Humanoid Robotics Book`,
},
```

### CSS for Footer (`Book-Frontend/src/css/custom.css`)

```css
/* In Book-Frontend/src/css/custom.css */

.footer {
  background-color: #0a0a15; /* Deep dark background */
  border-top: 2px solid var(--neon-purple); /* Neon-purple top border divider */
  padding-top: 50px; /* Adjust as needed */
  padding-bottom: 50px;
}

/* Style for social media icons */
.footer a {
  color: white;
  transition: all 0.3s ease;
}

.footer a:hover {
  color: var(--neon-purple);
  filter: drop-shadow(0 0 8px var(--neon-purple));
}
```
