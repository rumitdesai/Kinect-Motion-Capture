# Kinect Motion Capture

![Alt text](https://github.com/rumitdesai/Kinect_Motion_Capture/blob/master/User%20interface.png)

This project has been totally buid using:
- Microsoft visual studio 2013 community
- Kinect for windows (v2) SDK
- Qt 5.4
- OpenGL (graphics library)
- Eigen (Linear algebra library)
- pugi (XML processing library(C++))


## Installation
1. Install Kinect for windows (v2) SDK: https://www.microsoft.com/en-us/download/details.aspx?id=44561
1. Install Qt 5.4 from https://www.qt.io/download/
1. Install visual studio add-in 1.2.5 for Qt5 https://www.qt.io/download/
1. For OpenGL setup http://in2gpu.com/2014/10/15/setting-up-opengl-with-visual-studio/
1. Install pugiXML library http://pugixml.org/

## Project setup
1. Create a new project in visual studio 2013 `File->New->Project->Qt5 Projects(left pane)->Qt Application`, and select an appropriate name for project
1. `git clone` https://github.com/rumitdesai/Kinect_Motion_Capture.git inside your project folder
1. Move all the header**(.h)** files, source files**(.cpp)** files, **Object** folder and **Project Dependencies** folder to the folder which has project file(.vcxproj) of the newly created project
1. Add all the header files**(.h)** and **(.cpp)** files to your project insider visual studio `(Right click Header Files->Add->Existing Item), (Right click Source Files->Add->Existing Item)`

## Library linking
In order to run this project succesfully we have to link **Kinect v2 SDK**, and the **Project Dependencies** folder provided with this repository.
- Right click on project name under solution tab in visual studio, and select **properties** from the menu which will open **project properties** dialog box
- Select **All Configurations** under **Configuration:** on top left corner of dialog box
- Go to **VC++ Directories** under  **Configuration Properties** as you will see on left pane
    * Go to **Include Directories** and add **C:\Kinect_Motion_Capture\Project Dependencies\64 bit\Include** and **C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc**
    * Go to **Library Directories** and add **C:\Kinect_Motion_Capture\Project Dependencies\64 bit\libs**, **C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Lib\x64**
- Go to **Linker** on left pane
    * Inside **Linker** go to **Additional Library Directories** and add **C:\Kinect_Motion_Capture\Project Dependencies\64 bit\libs**, **C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Lib\x64**
    * Expand **Linker** and go to **Input**. In that look for **Additional Dependencies** and type in **glu32.lib**, **opengl32.lib** and **C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Lib\x64\Kinect20.lib**, notice here do not forget to add **Kinect20.lib** at the end of path

Make sure the project is built in x64. To check that click **BUILD** from menu bar and select **Configuration Manager**, check the **Active solution platform**. 