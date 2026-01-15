import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook">
      <main>
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--10 col--offset-1">
              <div className="text--center padding-horiz--md">
                {/* Hero Section */}
                <section className="hero-section padding-vert--xl">
                  <div className="hero-icon-container margin-bottom--lg">
                    <svg className="hero-icon" width="120" height="120" viewBox="0 0 100 100" fill="none" xmlns="http://www.w3.org/2000/svg">
                      <circle cx="50" cy="50" r="45" stroke="currentColor" strokeWidth="1.5" opacity="0.3"/>
                      <path d="M30 40H70V60H30V40Z" fill="currentColor" opacity="0.8"/>
                      <path d="M40 30L60 30L60 40L40 40L40 30Z" fill="currentColor" opacity="0.6"/>
                      <path d="M45 65L55 65L55 70L45 70L45 65Z" fill="currentColor" opacity="0.6"/>
                      <circle cx="45" cy="50" r="3" fill="currentColor"/>
                      <circle cx="55" cy="50" r="3" fill="currentColor"/>
                    </svg>
                  </div>

                  <h1 className="hero__title hero-main-title text--bold">{siteConfig.title}</h1>
                  <p className="hero__subtitle hero-main-subtitle text--normal padding-horiz--md">
                    {siteConfig.tagline}
                  </p>

                  <div className="hero-intro-text margin-vert--lg">
                    <p className="text--large">
                      This comprehensive textbook bridges the gap between artificial intelligence and physical embodiment,
                      providing you with the knowledge to create truly embodied intelligent systems.
                    </p>
                  </div>

                  <div className="margin-vert--lg">
                    <a
                      className="button button--primary button--lg button--cta margin-horiz--sm"
                      href="/Physical-AI-Humanoid-Robotics/docs/docs/intro"
                    >
                      Start Learning
                    </a>
                    <a
                      className="button button--secondary button--lg margin-horiz--sm"
                      href="/Physical-AI-Humanoid-Robotics/docs/docs/intro"
                    >
                      Read Introduction
                    </a>
                  </div>
                </section>

                {/* Key Modules Section */}
                <section className="modules-section padding-vert--lg">
                  <div className="modules-intro margin-vert--lg">
                    <h2 className="modules-title text--center padding-bottom--sm">Learning Modules</h2>
                    <p className="text--center text--normal">
                      Our textbook is organized into interconnected modules that can be studied independently
                      or as part of a complete curriculum. Each module builds upon fundamental concepts while
                      allowing flexibility in learning paths.
                    </p>
                  </div>

                  <div className="modules-grid margin-vert--lg">
                    <div className="module-card card padding--lg">
                      <div className="module-icon margin-bottom--md">
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M12 2L2 7L12 12L22 7L12 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                          <path d="M2 17L12 22L22 17" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                          <path d="M2 12L12 17L22 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                        </svg>
                      </div>
                      <h3 className="text--bold"><strong>Module 1:</strong> ROS 2 Fundamentals</h3>
                      <p className="text--normal">Learn the Robot Operating System fundamentals for building robust robotic applications.</p>
                    </div>
                    <div className="module-card card padding--lg">
                      <div className="module-icon margin-bottom--md">
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M3 15V19C3 20.1046 3.89543 21 5 21H19C20.1046 21 21 20.1046 21 19V15" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
                          <path d="M17 8L12 3L7 8" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                          <path d="M12 3V15" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                        </svg>
                      </div>
                      <h3 className="text--bold"><strong>Module 2:</strong> Simulation Environments</h3>
                      <p className="text--normal">Master simulation tools for testing and validating your robotic systems in virtual worlds.</p>
                    </div>
                    <div className="module-card card padding--lg">
                      <div className="module-icon margin-bottom--md">
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M4 11H20M4 15H20M4 7H20V19C20 19.5304 19.7893 20.0391 19.4142 20.4142C19.0391 20.7893 18.5304 21 18 21H6C5.46957 21 4.96086 20.7893 4.58579 20.4142C4.21071 20.0391 4 19.5304 4 19V7Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                        </svg>
                      </div>
                      <h3 className="text--bold"><strong>Module 3:</strong> NVIDIA Isaac Platform</h3>
                      <p className="text--normal">Explore NVIDIA's robotics platform for AI-powered robotic development.</p>
                    </div>
                    <div className="module-card card padding--lg">
                      <div className="module-icon margin-bottom--md">
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                          <path d="M20 21V19C20 17.9391 19.5786 16.9217 18.8284 16.1716C18.0783 15.4214 17.0609 15 16 15H8C6.93913 15 5.92172 15.4214 5.17157 16.1716C4.42143 16.9217 4 17.9391 4 19V21" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                          <path d="M12 11C14.2091 11 16 9.20914 16 7C16 4.79086 14.2091 3 12 3C9.79086 3 8 4.79086 8 7C8 9.20914 9.79086 11 12 11Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                        </svg>
                      </div>
                      <h3 className="text--bold"><strong>Module 4:</strong> VLA & Humanoid Development</h3>
                      <p className="text--normal">Build advanced humanoid robots with vision-language-action models.</p>
                    </div>
                  </div>
                </section>

                {/* Call to Action Section */}
                <section className="cta-section padding-vert--lg">
                  <div className="margin-vert--lg">
                    <h3 className="text--center padding-bottom--md">Ready to dive into Physical AI & Humanoid Robotics?</h3>
                    <div className="row">
                      <div className="col col--6 col--offset-3">
                        <a
                          className="button button--primary button--lg button--cta button--block"
                          href="/Physical-AI-Humanoid-Robotics/docs/docs/intro"
                        >
                          Begin Your Journey
                        </a>
                      </div>
                    </div>
                  </div>

                  <div className="hero-features margin-vert--lg text--center">
                    <p>Designed for students, researchers, and engineers passionate about the future of robotics.</p>
                  </div>
                </section>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}