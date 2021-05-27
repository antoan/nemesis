from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize



setup(
    ext_modules = cythonize([Extension("ThunderBorgCythonMock",
                            sources = ["ThunderBorgCythonMock.pyx"],
			                language="c++",
                            #include_dirs = [np.get_include()],
                            #libraries = ["dgesvd"]
                            #extra_compile_args = ["-I."],
                            #extra_link_args = ["-L./usr/lib/liblapack.dylib"]
                            )])

) 
