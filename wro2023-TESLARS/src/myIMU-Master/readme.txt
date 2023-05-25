Arduino Compatible Cross Platform C++ Library Project : For more information see http://www.visualmicro.com

This project works exactly the same way as an Arduino library should work. Code should be in the \src folder, code in deep sub folders below the \src folder is also supported.

The \src folder, if it exists, will be added as a compiler -I include path, otherwise the library folder will be a compiler -I include path.

Very old Arduino libraries have code in the library folder and private code in the \utility sub folder. They should be converted to this new format using \src and library.properties

Add this project to any solution that contains an Arduino project and #include <headers.h> in code as you would any normal Arduino library headers. 

To enable intellisense and to support live build discovery outside of the "standard" Arduino library locations, ensure that the library is added as a shared project reference to the master Arduino project. To do this, right click the master project "References" node and then click "Add Reference". A window will open and the library will appear on the "Shared Projects" tab. Click the checkbox next to the library name to add the reference. If this library is moved then the reference to it must be removed/re-added from any arduino projects that use it.

VS2017 has a bug, workround: After moving existing source code within a "library or shared project", close and re-open the solution.

Visual Studio will display intellisense for libraries based on the platform/board that has been specified for the currently active "Startup Project" of the current solution.

Adding a shared library project reference for an incorrect architetcure (incorrect board selection) will result in intellisense and/or compile errors.

IMPORTANT: The arduino.cc Library Rules must be followed when adding code or restructing libraries.


blog: http://www.visualmicro.com/post/2017/01/16/Arduino-Cross-Platform-Library-Development.aspx