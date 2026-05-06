# -*- coding: utf-8 -*-
import os
import sys
import glob
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

                                                                     
PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}


                                                            
                                                                      
                                                    
class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

                                                                
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

                                                    
        cfg = "Release"

                                                                        
                                                   
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

                                                                      
                                                                              
                      
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}".format(extdir),
            "-DPYTHON_EXECUTABLE={}".format(sys.executable),
            "-DEXAMPLE_VERSION_INFO={}".format(self.distribution.get_version()),
            "-DCMAKE_BUILD_TYPE={}".format(cfg),                                  
                                      
            "-DBUILD_PYTHON_BINDINGS=ON",
        ]
        build_args = []

        if self.compiler.compiler_type != "msvc":
                                                                          
                                                                             
                                                                               
                                                                            
                    
            if not cmake_generator:
                pass
                                           

        else:
                                                             
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})

                                                                                
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})

                                                                              
                                                                       
                             
            if not single_config and not contains_arch:
                cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]

                                                                             
            if not single_config:
                cmake_args += [
                    "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}".format(cfg.upper(), extdir)
                ]
                build_args += ["--config", cfg]

                                                                            
                                
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
                                                                               
                                                                                 
            if hasattr(self, "parallel") and self.parallel:
                                   
                build_args += ["-j{}".format(self.parallel)]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", "."] + build_args, cwd=self.build_temp
        )

                                                                             
                                                                                  
setup(
    name="pygicp",
    version="0.0.1",
    author="k.koide",
    author_email="k.koide@aist.go.jp",
    description="A collection of GICP-based point cloud registration algorithms",
    long_description="",
    ext_modules=[CMakeExtension("pygicp")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)
