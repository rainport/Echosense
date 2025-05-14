Echosense is a Unity plugin for spatial audio-based passive echolocation support, along with a standalone application for testing and training purposes. It was originally designed to enable navigation for low-vision and blind individuals by providing an intuitive audio embedding of the scene geometry.  

To build the plugin, run cmake from within the /NativePlugin/build directory. Then copy geometry_processor.bundle over to /Assets/Plugins. You will also need to install [libmysofa](https://github.com/hoene/libmysofa).  

To use it, add the script geometry_processor.cs as a component to a Unity object.  
  
The following plugin parameters can be tuned in Unity:  
Max Distance - max range at which objects will be detected  
$\sigma$ - width of Gaussian pulses  
Pulse Repetition Frequency  
Speed of Sound  

https://github.com/user-attachments/assets/2b7e8c11-cca8-41db-a907-6e9297ef33cb

