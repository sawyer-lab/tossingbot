from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# This looks for a folder named 'tossingbot' inside 'src'
d = generate_distutils_setup(
    packages=['tossingbot'],
    package_dir={'': 'src'}
)

setup(**d)