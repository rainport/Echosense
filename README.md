Echosense is a Unity plugin for spatial audio-based passive echolocation support, along with a standalone application for testing and training purposes. 

It was originally designed to enable navigation for low-vision and blind individuals, by providing an intuitive audio embedding of the scene geometry.

To build the plugin, run 

cmake ..

from within the /NativePlugin/build directory. Then, copy geometry_processor.bundle over to /Assets/Plugins.

You will also need to install libmysofa on your machine, available here: https://github.com/hoene/libmysofa.

The following parameters of the plugin can be tuned from within Unity:

$\sigma$
