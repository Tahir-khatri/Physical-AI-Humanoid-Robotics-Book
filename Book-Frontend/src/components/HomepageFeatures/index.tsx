import type {ReactNode} from 'react';
import Heading from '@theme/Heading';
import { cn } from "../../lib/utils";

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Interactive Code',
    description: (
      <>
        Engage with live code examples and experiment with Physical AI algorithms directly in your browser.
      </>
    ),
  },
  {
    title: 'Simulated Environments',
    description: (
      <>
        Explore realistic simulations of humanoid robots and complex environments, bringing theoretical concepts to life.
      </>
    ),
  },
  {
    title: 'Hardware-Acceleration',
    description: (
      <>
        Understand the principles of optimized computing for robotics, leveraging modern hardware for enhanced performance.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className="flex flex-col items-center p-6 text-center">
      <div className="padding-horiz--md">
        <h3 className="mb-2 text-xl font-semibold">{title}</h3>
        <p className="text-gray-600 dark:text-gray-300">{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className="container mx-auto px-4 py-16">
      <div className="grid grid-cols-1 gap-8 md:grid-cols-3">
        {FeatureList.map((props, idx) => (
          <Feature key={idx} {...props} />
        ))}
      </div>
    </section>
  );
}