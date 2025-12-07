import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';

const CustomNavbar = () => {
  const { t, i18n } = useTranslation();
  const location = useLocation();
  const [isMenuOpen, setIsMenuOpen] = useState(false);

  const changeLanguage = (lng) => {
    i18n.changeLanguage(lng);
  };

  const toggleMenu = () => {
    setIsMenuOpen(!isMenuOpen);
  };

  return (
    <nav
      className="navbar relative flex flex-wrap items-center justify-between px-4 py-3 bg-[#0d0d1a] border-b border-neon-cyan z-50"
      role="navigation"
      aria-label="Main navigation"
    >
      <div className="flex items-center justify-between w-full md:w-auto">
        <Link
          to="/"
          className="navbar__brand flex items-center text-xl font-orbitron font-bold text-neon-cyan no-underline hover:text-neon-cyan"
          aria-label="Home page"
        >
          <span className="navbar__title font-orbitron text-neon-cyan">
            {t('titlePage.title')}
          </span>
        </Link>

        {/* Mobile menu button */}
        <button
          className="md:hidden text-text-light focus:outline-none"
          onClick={toggleMenu}
          aria-label={isMenuOpen ? "Close menu" : "Open menu"}
          aria-expanded={isMenuOpen}
        >
          <svg
            className="w-6 h-6"
            fill="none"
            stroke="currentColor"
            viewBox="0 0 24 24"
            xmlns="http://www.w3.org/2000/svg"
          >
            {isMenuOpen ? (
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            ) : (
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
            )}
          </svg>
        </button>
      </div>

      {/* Navigation links - hidden on mobile by default */}
      <div
        className={`w-full md:flex md:items-center md:w-auto ${
          isMenuOpen ? 'block' : 'hidden'
        }`}
        id="navbar-collapse"
      >
        <div className="flex flex-col md:flex-row md:items-center md:justify-end space-y-4 md:space-y-0 md:space-x-6 mt-4 md:mt-0">
          <Link
            to="/"
            className={`font-inter ${
              location.pathname === '/' ? 'text-neon-cyan' : 'text-text-light'
            } hover:text-neon-cyan no-underline`}
          >
            {t('navbar.home')}
          </Link>
          <Link
            to="/docs/intro"
            className={`font-inter ${
              location.pathname.startsWith('/docs') ? 'text-neon-cyan' : 'text-text-light'
            } hover:text-neon-cyan no-underline`}
          >
            {t('navbar.textbook')}
          </Link>
          <Link
            to="/blog"
            className={`font-inter ${
              location.pathname.startsWith('/blog') ? 'text-neon-cyan' : 'text-text-light'
            } hover:text-neon-cyan no-underline`}
          >
            {t('navbar.blog')}
          </Link>

          {/* Language toggle */}
          <button
            onClick={() => changeLanguage(i18n.language === 'en' ? 'ur' : 'en')}
            className="px-3 py-1 bg-magenta text-text-light rounded font-inter text-sm hover:bg-[#ff2a6d]/90 transition-colors"
            aria-label={t('urduToggle.description')}
          >
            {i18n.language === 'en' ? t('urduToggle.label') : 'English'}
          </button>
        </div>
      </div>
    </nav>
  );
};

export default CustomNavbar;