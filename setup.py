#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup()
package_info['packages'] = ['gazetracking']
package_info['package_dir'] = {'':'src'}
package_info['install_requires'] = []

setup(**package_info)
