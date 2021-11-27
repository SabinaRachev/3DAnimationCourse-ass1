# EngineForAnimationCourse
Graphic Engine based on Libigl


The following files were the ones that we had modified to support the simplification algorithm:
1. Sandbox.cpp / Sandbox.h 
2. inputManager.cpp 
3. Viewer.cpp /Viewer.h - added a vitual function that is called initData in order to support simplification for loaded objects
4. ImGuiMenu - added the command viewer->initData() to the function that handles loading objects.

