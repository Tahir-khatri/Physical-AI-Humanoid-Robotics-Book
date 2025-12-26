import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/HeroSection';

export default function Home(): ReactNode {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="An immersive learning experience for AI and Robotics enthusiasts.">
      <HeroSection />
    </Layout>
  );
}
