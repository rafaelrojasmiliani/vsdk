try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup


setup(
    name='vsdk',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    install_requires=[
        'numpy >= 1.6.2',
        'scipy >= 0.10.0'
    ],
    packages=[
        'vsdk'
    ],
)
