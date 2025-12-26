import React, { useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Sphere, MeshDistortMaterial } from '@react-three/drei';
import Link from '@docusaurus/Link';

const AnimatedSphere = () => {
  const ref = useRef();
  useFrame(() => (ref.current.rotation.x = ref.current.rotation.y += 0.01));
  return (
    <Sphere ref={ref} args={[1.5, 32, 32]} scale={2}>
      <MeshDistortMaterial
        color="#7c3aed"
        attach="material"
        distort={0.4}
        speed={1.5}
        roughness={0.5}
        metalness={0.8}
      />
    </Sphere>
  );
};

export default function HeroSection() {
  return (
    <div style={{
      position: 'relative',
      width: '100%',
      height: '100vh',
      display: 'flex',
      flexDirection: 'column',
      justifyContent: 'center',
      alignItems: 'center',
      textAlign: 'center',
      color: 'white',
      background: 'linear-gradient(180deg, #0a192f 0%, #000000 100%)',
      padding: '100px 0',
      overflow: 'hidden',
    }}>
      <Canvas style={{ position: 'absolute', top: 0, left: 0, zIndex: 0 }}>
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10]} intensity={1} color="#2563eb" />
        <pointLight position={[-10, -10, -10]} intensity={0.8} color="#7c3aed" />
        <AnimatedSphere />
      </Canvas>
      <div style={{ position: 'relative', zIndex: 1, maxWidth: '800px', padding: '0 20px' }}>
        <h1 style={{
          fontSize: '4rem',
          fontWeight: 900,
          fontFamily: 'Inter, sans-serif',
          color: 'white',
          textShadow: '0 0 10px rgba(124, 58, 237, 0.7)',
        }}>
          Physical AI & Humanoid Robotics
        </h1>
        <p style={{
          fontSize: '1.5rem',
          color: '#a0aec0',
          fontFamily: 'Inter, sans-serif',
          marginTop: '1rem',
        }}>
          Bridging Digital Intelligence with Physical Bodies
        </p>
        <div style={{ marginTop: '3rem' }}>
          <Link
            className="button button--secondary button--lg cta-button"
            to="/docs/introduction/01-ros-2-overview">
            Start Reading
          </Link>
        </div>
      </div>
    </div>
  );
}