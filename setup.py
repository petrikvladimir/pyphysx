import os
import re
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        # required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cmake_args = ['-DPHYSX_LIBRARY_OUTPUT_DIRECTORY=' + extdir, '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            if sys.maxsize > 2 ** 32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''), self.distribution.get_version())

        if env.get('PYPHYSX_COV'):
            cmake_args += ['-DENABLE_COVERAGE=On']

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.', '--target', 'pyphysx'] + build_args, cwd=self.build_temp)


setup(
    name='pyphysx',
    version='0.2.4',
    author='Vladimir Petrik',
    author_email='vladimir.petrik@cvut.cz',
    description='PyPhysX - python wrapper for PhysX Nvidia simulator.',
    long_description='',
    ext_modules=[CMakeExtension('pyphysx')],
    cmdclass=dict(build_ext=CMakeBuild),
    install_requires=['conan==1.59.0', 'numpy', 'imageio', 'imageio_ffmpeg', 'trimesh', 'networkx',
                      'numba', 'numpy_quaternion', 'matplotlib', 'scipy', 'anytree', 'pyrender', 'meshcat',
                      'pycollada'],
    setup_requires=['pytest-runner', 'conan==1.59.0', 'numpy', 'imageio', 'imageio_ffmpeg', 'trimesh', 'networkx',
                    'numba', 'numpy_quaternion', 'matplotlib', 'scipy', 'anytree', 'pyrender', 'meshcat',
                    'pycollada'],
    tests_require=['pytest'],
    packages=['pyphysx_render', 'pyphysx_utils'],
    zip_safe=False,
)
