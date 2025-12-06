import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="relative h-screen flex items-center justify-center overflow-hidden">
      {/* Background Image with Overlay */}
      <img
        src="/img/homepage.jpg"
        alt="Physical AI & Humanoid Robotics"
        className="absolute z-0 w-full h-full object-cover brightness-50"
      />
      <div className="absolute z-10 p-5 text-center text-white">
        <Heading as="h1" className="text-fluid-7xl md:text-fluid-8xl font-extrabold leading-tight mb-4 animate-fade-in-down animation-delay-300 text-gradient-to-r text-balance">
          {siteConfig.title}
        </Heading>
        <p className="text-fluid-2xl md:text-fluid-3xl text-white mb-8 animate-fade-in-up animation-delay-500 text-balance">
          {siteConfig.tagline}
        </p>
        <div className="flex justify-center animate-fade-in animation-delay-700">
          <Link
            className="
              bg-primary-500 hover:bg-primary-600
              text-white font-bold
              py-3 px-8 rounded-full shadow-lg
              transition duration-300 ease-in-out transform hover:scale-105
              text-lg
            "
            to="/docs/intro">
            Explore the Textbook - Start Learning 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main className="relative z-20">
        {/* About the Book Section */}
        <section className="py-20 bg-gray-50 dark:bg-gray-800">
          <div className="container mx-auto px-4">
            <div className="flex flex-col md:flex-row items-center gap-8">
              <div className="md:w-1/2">
                <img
                  src="/img/robotics-concept.jpg"
                  alt="Physical AI & Humanoid Robotics Concept"
                  className="w-full h-auto rounded-lg shadow-lg"
                />
              </div>
              <div className="md:w-1/2 text-center md:text-left">
                <Heading as="h2" className="text-fluid-4xl font-bold mb-4 text-gray-800 dark:text-white text-balance">
                  About "Physical AI & Humanoid Robotics"
                </Heading>
                <p className="text-lg text-gray-600 dark:text-gray-300 mb-8">
                  Dive into the transformative world where artificial intelligence meets physical embodiment. This textbook provides a comprehensive guide to understanding, designing, and deploying AI systems for humanoid robots, covering everything from fundamental concepts to advanced simulations and real-world applications.
                </p>
                <Link
                  className="
                    bg-secondary-500 hover:bg-secondary-600
                    text-white font-bold
                    py-3 px-8 rounded-full shadow-lg
                    transition duration-300 ease-in-out transform hover:scale-105
                  "
                  to="/docs/intro">
                  Learn More
                </Link>
              </div>
            </div>
          </div>
        </section>

        {/* Key Concepts Section */}
        <section className="py-20 bg-white dark:bg-gray-900">
          <div className="container mx-auto px-4">
            <Heading as="h2" className="text-fluid-4xl font-bold mb-12 text-gray-800 dark:text-white text-center text-balance">
              Key Concepts You'll Master
            </Heading>
            <div className="space-y-12">
              <div className="flex flex-col md:flex-row items-center gap-8">
                <div className="md:w-1/2">
                  <img src="/img/ros2-architecture.svg" alt="ROS 2 Architecture" className="w-full h-auto rounded-lg shadow-md" />
                </div>
                <div className="md:w-1/2 text-center md:text-left">
                  <h3 className="text-2xl font-semibold mb-3 text-primary-500">ROS 2 & Robotics Middleware</h3>
                  <p className="text-gray-600 dark:text-gray-300">Understand the backbone of modern robotics.</p>
                </div>
              </div>
              <div className="flex flex-col md:flex-row-reverse items-center gap-8">
                <div className="md:w-1/2">
                  <img src="/img/undraw_docusaurus_react.svg" alt="Advanced Simulation" className="w-full h-auto rounded-lg shadow-md" />
                </div>
                <div className="md:w-1/2 text-center md:text-left">
                  <h3 className="text-2xl font-semibold mb-3 text-secondary-500">Advanced Simulation Techniques</h3>
                  <p className="text-gray-600 dark:text-gray-300">Master digital twins and synthetic data generation.</p>
                </div>
              </div>
              <div className="flex flex-col md:flex-row items-center gap-8">
                <div className="md:w-1/2">
                  <img src="/img/feature-embodied-ai.svg" alt="Embodied AI" className="w-full h-auto rounded-lg shadow-md" />
                </div>
                <div className="md:w-1/2 text-center md:text-left">
                  <h3 className="text-2xl font-semibold mb-3 text-[var(--ifm-color-tertiary)]">Embodied AI & Reinforcement Learning</h3>
                  <p className="text-gray-600 dark:text-gray-300">Develop intelligent agents for physical interaction.</p>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Visual Overview Section */}
        <section className="py-20 bg-gray-100 dark:bg-gray-800">
          <div className="container mx-auto px-4">
            <Heading as="h2" className="text-fluid-4xl font-bold mb-12 text-gray-800 dark:text-white text-center text-balance">
              A Glimpse Inside: Visual Overview
            </Heading>
            <div className="space-y-12">
              <div className="flex flex-col md:flex-row items-center gap-8">
                <div className="md:w-1/2">
                  <img src="/img/undraw_docusaurus_mountain.svg" alt="Modular Content" className="w-full h-auto rounded-lg shadow-md" />
                </div>
                <div className="md:w-1/2 text-center md:text-left">
                  <h3 className="text-2xl font-semibold mb-3 text-primary-500">Modular Content Structure</h3>
                  <p className="text-gray-600 dark:text-gray-300">Navigate complex topics with ease through our logically organized modules.</p>
                </div>
              </div>
              <div className="flex flex-col md:flex-row-reverse items-center gap-8">
                <div className="md:w-1/2">
                  <img src="/img/undraw_docusaurus_react.svg" alt="Interactive Learning" className="w-full h-auto rounded-lg shadow-md" />
                </div>
                <div className="md:w-1/2 text-center md:text-left">
                  <h3 className="text-2xl font-semibold mb-3 text-secondary-500">Interactive Learning Experience</h3>
                  <p className="text-gray-600 dark:text-gray-300">Engage with practical examples, code snippets, and hands-on exercises.</p>
                </div>
              </div>
              <div className="flex flex-col md:flex-row items-center gap-8">
                <div className="md:w-1/2">
                  <img src="/img/undraw_docusaurus_tree.svg" alt="Future-Proof Skills" className="w-full h-auto rounded-lg shadow-md" />
                </div>
                <div className="md:w-1/2 text-center md:text-left">
                  <h3 className="text-2xl font-semibold mb-3 text-[var(--ifm-color-tertiary)]">Future-Proof Your Skills</h3>
                  <p className="text-gray-600 dark:text-gray-300">Stay ahead with content covering the latest advancements in AI and Robotics.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
