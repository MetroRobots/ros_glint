from .util import roscompile


@roscompile
def check_python_dependencies(package):
    run_depends = package.source_code.get_external_python_dependencies()
    package.package_xml.add_packages(set(), run_depends, prefer_depend_tag=False)
