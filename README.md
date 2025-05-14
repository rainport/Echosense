Echosense is a Unity plugin for spatial audio-based passive echolocation support, along with a standalone application for testing and training purposes. 

It was originally designed to enable navigation for low-vision and blind individuals by providing an intuitive audio embedding of the scene geometry.

To build the plugin, run cmake from within the /NativePlugin/build directory. Then copy geometry_processor.bundle over to /Assets/Plugins. You will also need to install [libmysofa](https://github.com/hoene/libmysofa).

To use the plugin, add the script geometry_processor.cs as a component to an object.
  
The following plugin parameters can be tuned:

Max Distance - max range at which objects will be detected  
$\sigma$ - width of Gaussian pulse  
Speed of Sound - speed of sound in medium  
Pulse Repetition Frequency - frequency of echo generation
