Echosense is a Unity plugin for spatial audio-based passive echolocation support, along with a standalone application for testing and training purposes. 

It was originally designed to enable navigation for low-vision and blind individuals, by providing an intuitive audio embedding of the scene geometry.

To build the plugin, run cmake .. from within the /NativePlugin/build directory. Then, copy geometry_processor.bundle over to /Assets/Plugins.

You will also need to install [libmysofa](https://github.com/hoene/libmysofa) on your machine.

To use the plugin, add the script geometry_processor.cs as a component to an object in Unity.

The following parameters of the plugin can be tuned from within Unity:

Max Distance - maximum range at which objects will be detected  

$\sigma$ - the width of the Gaussian pulse  

Speed of Sound - speed of sound in the medium  

Pulse Repetition Frequency - frequency at which new pulses are emitted and echoes are generated
