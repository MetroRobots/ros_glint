![ROS Glint animated logo](logo.gif)

### Make your ROS code sparkle!

ROS Glint is a linter for ROS 1 and ROS 2 packages.
It is a pure Python version of the ROS 1 package [roscompile](https://github.com/DLu/roscompile/blob/main/roscompile/README.md).

# Running from the Command Line

Installing the Python package installs the `glint_ros` command. (Note: There's 5 million commands that start with `ros`. [citation needed] The command is `glint_ros` for ease of access/tab completion)

The command will find all ROS packages in a the current directory and attempt to run the "glinters" on them all.

```
usage: glint_ros [-h] [-f FOLDER] [-y] [-s] [linter ...]

positional arguments:
  linter                By default, run all linters. If any are specified here, only those
                        specified are run.

options:
  -h, --help            show this help message and exit
  -f FOLDER, --folder FOLDER
                        The folder to search for ROS packages in. Defaults to the current
                        directory.
  -y, --yes-to-all      Non-interactive mode that accepts all suggestions.
  -s, --skip-ros-load   Avoid loading ROS resources, useful in scripting environments.
```

# Acknowledgements
 * ROS Glint logo by [https://glowtxt.com/](https://glowtxt.com/)
