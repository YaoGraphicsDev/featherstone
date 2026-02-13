### Building Bullet (Collision Only)

1. Configure and generate the Bullet solution using CMake GUI.  
   Disable all unnecessary components.

2. Set `CMAKE_INSTALL_PREFIX` to the desired installation directory.

3. Ensure `CMAKE_DEBUG_POSTFIX` is set to a valid value so that you can later tell debug and release build products apart.

4. Click **Configure** and **Generate**.

5. Open the generated solution in Visual Studio.  
   For both **LinearMath** and **BulletCollision** projects:

   - Go to: `Properties → C/C++ → Code Generation → Runtime Library`
   - For **Debug**, set the runtime library to `/MDd`, then build the **INSTALL** target.
   - For **Release**, set the runtime library to `/MD`, then build the **INSTALL** target.

6. Collect the generated Debug and Release libraries from the installation directory.  
   Also copy the `include` directory.
