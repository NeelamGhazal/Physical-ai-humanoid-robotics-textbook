import React from 'react';
import { useTranslation } from 'react-i18next';
import { motion } from 'framer-motion';

const TitlePage = () => {
  const { t } = useTranslation();

  return (
    <div
      className="relative w-full min-h-screen flex items-center justify-center overflow-hidden"
      role="banner"
      aria-label="Title page with textbook information"
    >
      {/* Gradient background from deep indigo to space black */}
      <div
        className="absolute inset-0 bg-gradient-to-br from-deep-indigo to-space-black"
        style={{
          background: 'linear-gradient(135deg, #4169e1 0%, #0d0d1a 100%)'
        }}
      >
        {/* Animated particle effects */}
        <div className="absolute inset-0">
          {[...Array(50)].map((_, i) => (
            <motion.div
              key={i}
              className="absolute w-1 h-1 bg-neon-cyan rounded-full"
              style={{
                top: `${Math.random() * 100}%`,
                left: `${Math.random() * 100}%`,
              }}
              animate={{
                y: [0, -20, 0],
                opacity: [0.3, 1, 0.3],
              }}
              transition={{
                duration: 3 + Math.random() * 2,
                repeat: Infinity,
                ease: "easeInOut",
                delay: Math.random() * 2,
              }}
            />
          ))}
        </div>
      </div>

      {/* Floating humanoid SVG */}
      <motion.div
        className="absolute top-1/4 left-1/4"
        animate={{
          y: [-20, 20, -20],
          rotate: [-2, 2, -2]
        }}
        transition={{
          duration: 4,
          repeat: Infinity,
          ease: "easeInOut",
        }}
      >
        <svg
          width="80"
          height="80"
          viewBox="0 0 100 100"
          className="text-neon-cyan opacity-70"
        >
          <circle cx="50" cy="30" r="15" stroke="currentColor" strokeWidth="2" fill="none" />
          <path d="M50 45 L50 75" stroke="currentColor" strokeWidth="2" />
          <path d="M40 65 L50 75 L60 65" stroke="currentColor" strokeWidth="2" fill="none" />
          <path d="M30 90 L45 80 M70 90 L55 80" stroke="currentColor" strokeWidth="2" />
        </svg>
      </motion.div>

      {/* Floating humanoid SVG - second one */}
      <motion.div
        className="absolute top-2/3 right-1/3"
        animate={{
          y: [20, -20, 20],
          rotate: [3, -3, 3]
        }}
        transition={{
          duration: 5,
          repeat: Infinity,
          ease: "easeInOut",
          delay: 1,
        }}
      >
        <svg
          width="60"
          height="60"
          viewBox="0 0 100 100"
          className="text-magenta opacity-60"
        >
          <circle cx="50" cy="30" r="12" stroke="currentColor" strokeWidth="2" fill="none" />
          <path d="M50 42 L50 65" stroke="currentColor" strokeWidth="2" />
          <path d="M42 58 L50 65 L58 58" stroke="currentColor" strokeWidth="2" fill="none" />
          <path d="M35 80 L47 72 M65 80 L53 72" stroke="currentColor" strokeWidth="2" />
        </svg>
      </motion.div>

      {/* Content */}
      <div className="relative z-10 text-center px-4">
        <motion.h1
          className="text-5xl md:text-7xl font-orbitron font-bold text-neon-cyan mb-6 leading-tight"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
        >
          {t('titlePage.title')}
        </motion.h1>

        <motion.p
          className="text-xl md:text-2xl text-text-light mb-8 max-w-3xl mx-auto font-inter"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.2 }}
        >
          {t('titlePage.subtitle')}
        </motion.p>

        <motion.p
          className="text-lg text-text-light/80 font-inter"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, delay: 0.4 }}
        >
          {t('titlePage.author')}
        </motion.p>
      </div>

      {/* Floating elements */}
      <motion.div
        className="absolute bottom-20 left-1/2 transform -translate-x-1/2"
        animate={{ y: [0, -10, 0] }}
        transition={{
          duration: 2.5,
          repeat: Infinity,
          ease: "easeInOut"
        }}
      >
        <div className="w-8 h-8 border-l-2 border-b-2 border-neon-cyan rotate-45"></div>
      </motion.div>
    </div>
  );
};

export default TitlePage;