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

- soxr (the SOX resample library)

- a somewhat recent gcc