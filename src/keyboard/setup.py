from setuptools import setup

setup(
    name='keyboard',
    version='0.0.1',
    packages=['keyboard'],
    package_dir={'': 'src'},
    install_requires=[],
    scripts=['src/keyboard/keyboard.py'],  # Add your Python script here
)
