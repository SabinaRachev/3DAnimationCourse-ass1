# EngineForAnimationCourse
Graphic Engine based on Libigl


The following files were the ones that we had modified to support the simplification algorithm:
1. Sandbox.cpp / Sandbox.h 
2. inputManager.cpp 
3. Viewer.cpp /Viewer.h - added a vitual function that is called initData in order to support simplification for loaded objects
4. ImGuiMenu - added the command viewer->initData() to the function that handles loading objects.

in order to run the project:
1.clone the git repository 
3. Run Cmake gui. choose the project folder and destination folder for the cpp project files. 
4. Search for "texture_image"on viewerdata.cpp and replace "snake1" with the correct path of your directory. 
5. Confirm correct configurations paths.
6. Copy configuration.txt from tutorial/sandBox to build/tutorial/sandBox sandBox as a startup project and compile the project (it could take few minutes);   
7. Build the project
