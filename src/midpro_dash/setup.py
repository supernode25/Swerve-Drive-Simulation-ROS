from setuptools import setup
package_name = 'midpro_dash'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='midpro', maintainer_email='dev@example.com',
      description='Live driving dashboard (PyQt5 + pyqtgraph)', license='Proprietary',
      entry_points={'console_scripts': ['speed_dash = midpro_dash.speed_dash:main',
                                      'run_regression = midpro_dash.regression:main',
                                      'compare_regression = midpro_dash.compare:main',
                                      'run_variants = midpro_dash.run_variants:main',
                                      'goal_footprint = midpro_dash.goal_footprint:main']})
