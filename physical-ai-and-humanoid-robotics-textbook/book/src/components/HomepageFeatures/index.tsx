import type {ReactNode} from 'react';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link'; // Import Link for image captions or links

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
  delay?: string; // Add delay property
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Build Embodied AI Systems',
    Svg: require('@site/static/img/feature-embodied-ai.svg').default, // Using a more specific SVG
    description: (
      <>
        Learn to design, simulate, and deploy AI systems that interact with the physical world, bridging the gap between digital intelligence and physical embodiment.
      </>
    ),
    delay: 'animation-delay-100',
  },
  {
    title: 'Master Robotics Middleware',
    Svg: require('@site/static/img/ros2-architecture.svg').default, // Using ROS2 specific SVG
    description: (
      <>
        Gain expertise in ROS 2, the industry-standard middleware for robotics, and learn to control complex robotic systems.
      </>
    ),
    delay: 'animation-delay-200',
  },
  {
    title: 'Explore Advanced Simulations',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default, // Reverted to SVG
    description: (
      <>
        Dive into Gazebo, Unity, and NVIDIA Isaac Sim for high-fidelity physics simulations, digital twins, and synthetic data generation.
      </>
    ),
    delay: 'animation-delay-300',
  },
];

type ImageItem = {
  src: string;
  alt: string;
  caption?: string;
  link?: string;
};

const ImageGallery: ImageItem[] = [
  {
    src: '/img/ros2-architecture.svg',
    alt: 'ROS 2 Architecture',
    caption: 'Overview of ROS 2 framework architecture for robotics development.',
    link: '/docs/module-1/ros-overview',
  },
  {
    src: '/img/ros2-publish-subscribe.svg',
    alt: 'ROS 2 Publish Subscribe Mechanism',
    caption: 'Illustrating data flow with ROS 2 Publish/Subscribe patterns.',
    link: '/docs/module-1/02-ros-nodes-topics-services',
  },
  {
    src: '/img/ros2-request-response.svg',
    alt: 'ROS 2 Request Response Services',
    caption: 'Understanding ROS 2 service calls for request-response interactions.',
    link: '/docs/module-1/02-ros-nodes-topics-services',
  },
  {
    src: '/img/feature-embodied-ai.svg',
    alt: 'Embodied AI Concept',
    caption: 'The convergence of AI and physical robotics for intelligent agents.',
    link: '/docs/intro',
  },
];

function Feature({title, Svg, description, delay}: FeatureItem) {
  return (
    <div className="w-full p-4">
      <div className={`
        flex flex-col md:flex-row items-center gap-6
        bg-white dark:bg-gray-800
        rounded-lg shadow-lg p-6
        transform transition duration-300 hover:scale-105 hover:shadow-xl hover:-translate-y-2
        animate-fade-in-up ${delay}
      `}>
        <div className="md:w-2/5 flex justify-center">
          <Svg className="w-24 h-24 text-[var(--ifm-color-primary)]" role="img" />
        </div>
        <div className="md:w-3/5 text-center md:text-left">
          <Heading as="h3" className="text-xl font-semibold mb-2 text-balance">
            {title}
          </Heading>
          <p className="text-gray-600 dark:text-gray-300 text-balance">{description}</p>
        </div>
      </div>
    </div>
  );
}

function ImageGrid() {
  return (
    <section className="py-16 bg-gray-100 dark:bg-gray-900">
      <div className="container mx-auto px-4">
        <Heading as="h2" className="text-fluid-4xl font-bold text-center mb-12 text-gray-800 dark:text-white text-balance">
          Visualizing Key Concepts in Physical AI
        </Heading>
        <div className="space-y-8">
          {ImageGallery.map((item, idx) => (
            <div key={idx} className={`flex flex-col md:flex-row items-center gap-6 bg-white dark:bg-gray-800 rounded-lg shadow-lg p-6 animate-fade-in delay-${(idx + 1) * 100}`}>
              <div className="md:w-2/5">
                <img
                  src={item.src}
                  alt={item.alt}
                  className="w-full h-auto rounded-lg"
                />
              </div>
              <div className="md:w-3/5 text-center md:text-left">
                <p className="text-lg font-semibold mb-2 text-gray-800 dark:text-white">
                  {item.caption}
                </p>
                {item.link && (
                  <Link to={item.link} className="inline-block bg-primary-500 hover:bg-primary-600 text-white font-bold py-2 px-4 rounded-full shadow-lg transition duration-300 ease-in-out transform hover:scale-105">
                    View Details
                  </Link>
                )}
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <>
      <section className="py-16 bg-white dark:bg-gray-800">
        <div className="container mx-auto px-4">
          <div className="flex flex-wrap -mx-4 justify-center">
            {FeatureList.map((props, idx) => (
              <Feature key={idx} {...props} delay={props.delay || ''} />
            ))}
          </div>
        </div>
      </section>
      <ImageGrid />
    </>
  );
}
