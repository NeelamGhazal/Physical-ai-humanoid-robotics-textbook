import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Learn Physical AI',
    Svg: require('../../static/img/robot-arm.svg').default,
    description: (
      <>
        Comprehensive coverage of Physical AI concepts, from basic robotics
        principles to advanced embodied AI techniques.
      </>
    ),
  },
  {
    title: 'Four Core Modules',
    Svg: require('../../static/img/modules.svg').default,
    description: (
      <>
        Master ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language Actions
        through structured learning paths.
      </>
    ),
  },
  {
    title: 'Interactive Learning',
    Svg: require('../../static/img/interactive.svg').default,
    description: (
      <>
        Engage with interactive code examples, quizzes, and hands-on exercises
        to reinforce your understanding.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}