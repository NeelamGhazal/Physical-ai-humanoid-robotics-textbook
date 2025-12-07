import React from 'react';
import { motion } from 'framer-motion';

const ContentCard = ({ title, children, animated = true, className = "" }) => {
  const cardId = `content-card-${title ? title.toLowerCase().replace(/\s+/g, '-') : 'default'}`;

  return (
    <motion.div
      className={`
        relative p-6 rounded-lg bg-space-black text-text-light border-2
        ${animated ? 'border-neon-cyan animate-pulse-border' : 'border-neon-cyan'}
        ${className}
      `}
      role="region"
      aria-labelledby={title ? cardId : undefined}
      whileHover={{ scale: 1.02 }}
      animate={animated ? {
        boxShadow: '0 0 15px rgba(0, 240, 255, 0.5)',
        transition: { duration: 2, repeat: Infinity, repeatType: "reverse" }
      } : {}}
    >
      {title && (
        <h3 id={cardId} className="text-xl font-orbitron text-neon-cyan mb-3 flex items-center">
          {title}
        </h3>
      )}
      <div className="text-inter">
        {children}
      </div>
    </motion.div>
  );
};

export default ContentCard;