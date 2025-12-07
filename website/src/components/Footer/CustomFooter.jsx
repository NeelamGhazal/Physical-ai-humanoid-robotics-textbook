import React from 'react';
import { useTranslation } from 'react-i18next';
import Link from '@docusaurus/Link';

const CustomFooter = () => {
  const { t, i18n } = useTranslation();

  return (
    <footer
      className="footer relative bg-[#0d0d1a] border-t border-neon-cyan text-text-light py-8"
      role="contentinfo"
      aria-label="Site footer"
    >
      <div className="container mx-auto px-4">
        <div className="flex flex-col md:flex-row justify-between items-center">
          <div className="mb-4 md:mb-0">
            <p className="text-center md:text-left font-inter">
              {t('footer.copyright')} {t('titlePage.title')}
            </p>
          </div>

          <div className="flex flex-wrap justify-center md:justify-end gap-6 mb-4 md:mb-0">
            <Link
              to="#"
              className="text-text-light hover:text-neon-cyan font-inter transition-colors"
              aria-label="GitHub repository"
            >
              GitHub
            </Link>
            <Link
              to="#"
              className="text-text-light hover:text-neon-cyan font-inter transition-colors"
              aria-label="Twitter account"
            >
              Twitter
            </Link>
            <Link
              to="#"
              className="text-text-light hover:text-neon-cyan font-inter transition-colors"
              aria-label="LinkedIn profile"
            >
              LinkedIn
            </Link>
          </div>

          <div className="flex items-center">
            <button
              onClick={() => i18n.changeLanguage(i18n.language === 'en' ? 'ur' : 'en')}
              className="px-3 py-2 bg-magenta text-text-light rounded font-inter text-sm hover:bg-[#ff2a6d]/90 transition-colors"
              aria-label={i18n.language === 'en' ? t('footer.urduToggle') : t('footer.switchToUrdu')}
            >
              {i18n.language === 'en' ? 'اردو' : 'English'}
            </button>
          </div>
        </div>

        <div className="mt-6 pt-6 border-t border-[#2a2a3a] text-center">
          <p className="font-inter text-sm text-text-light/70">
            {t('common.backToTop')} - Built with Docusaurus
          </p>
        </div>
      </div>
    </footer>
  );
};

export default CustomFooter;