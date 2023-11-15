from .util import get_config, roscompile


@roscompile
def check_python_dependencies(package):
    run_depends = package.source_code.get_external_python_dependencies()
    package.package_xml.add_packages(set(), run_depends, prefer_depend_tag=False)


@roscompile
def update_people(package, config=None):
    if config is None:
        config = get_config()
    for d in config.get('replace_rules', []):
        package.package_xml.update_people(d['to']['name'], d['to']['email'],
                                          d['from'].get('name', None), d['from'].get('email', None))


@roscompile
def update_license(package, config=None):
    if config is None:
        config = get_config()
    if 'default_license' not in config or 'TODO' not in package.package_xml.get_license():
        return

    package.package_xml.set_license(config['default_license'])
