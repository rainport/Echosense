Echosense is a Unity plugin for spatial audio-based passive echolocation support, along with a standalone application for testing and training purposes. 

It was originally designed to enable navigation for low-vision and blind individuals, by providing an intuitive audio embedding of the scene geometry.

To build the plugin, run cmake .. from within the /NativePlugin/build directory. Then, copy geometry_processor.bundle over to /Assets/Plugins.

You will also need to install [libmysofa](https://github.com/hoene/libmysofa) on your machine.

To use the plugin, add the script geometry_processor.cs as a component to an object in Unity.

The following parameters of the plugin can be tuned from within Unity:

$\text{max_distance}$ - maximum range at which objects will be detected
$\sigma$ - the width of the Gaussian pulse
$\text{speed_of_sound}$ - speed of sound in the medium
$pulse_repetition_frequency$ - frequency at which new pulses are emitted, generating echoes
