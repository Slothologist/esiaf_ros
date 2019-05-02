# Esiaf
Esiaf (spoken "easy ah eff") is a library intended to enable ROS nodes to easily interchange audio.
This includes the raw audio signal as well as meta-information.

To simplify usage, a node using esiaf just needs to declare what type of audio it requires (eg.  sample- and bit-rate) or produces and esiaf will handle resampling between the nodes.

As such, esiaf is tailored for the use in a larger speech and sound recognition pipeline, to enable a highly modular design.


## Disclaimer

Esaif is in development and currently not recommended for use.
Scope, interfaces, messages, basic principles are still subject to change.

## Requirements

- ROS

- ROS MoveIt! (basically just for some convenience functions regarding python bindings)

- soxr (the SOX resample library)

- a somewhat recent gcc

- [Boost.NumPy](https://github.com/ndarray/Boost.NumPy) needs to be installed alongside the normal boost library.
On ROS kinetic this typically means that you need to compile Boost.NumPy from source (as numpy was not merged till 1.63 and kinetic comes with boost 1.58).
You will need to make sure that the numpy headers get installed to /usr/include instead of the default /usr/local/include.
You can easily accomplish this by invoking cmake with -DCMAKE_INSTALL_PREFIX=/usr.
However, the libboost_numpy.so may also need to be moved to the boost library directory.