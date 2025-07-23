from conan import ConanFile

class MainProject(ConanFile):
    python_requires = "conan_template/[~5]@robotkernel/stable"
    python_requires_extend = "conan_template.RobotkernelConanFile"

    name = "module_el6751"
    url = "https://rmc-github.robotic.dlr.de/robotkernel/module_el6751.git"
    description = ""
    exports_sources = ["*", "!.gitignore"]
    
    def source(self):
        self.run(f"sed 's|PACKAGE_VERSION|{self.version}|' configure.ac.in > configure.ac")

    def requirements(self):
        self.requires("robotkernel/[~6]@robotkernel/unstable")
