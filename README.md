# TreeGen-3D-From-Image-to-Model

## TODO:
1. Add BOOST lib to CMAKE if possible. Since didn't add it succesfully, I currently added through Visual Studio

(Project properties → C/C++ → General → Additional Include Directories, and add a path to the boost library root (in my case E:\Upenn\cis660\IPML2d\L-system inference\boost_1_84_0)

2. in ImportImageCmd.cpp(the main logic), try move helper variables(at the very top) into private member variables

3. buildGraph() has some bug when detecting Edge, check & compare print message with QT project

4. Continue finish buildNaryTree() base on QT projects example (Main Part)

5. organnize helper function into another file

* Added files include: ImportImagecmd(main plugin logic), tree_structure, tutrle(adjusted), BOOST lib, udgcd_cycle_detector.hpp

* in ImportImagecmd.cpp doIt() is the main function, the main helper functions include(4 buttons in QT):  

MStatus parseBoundingBoxData(const std::string& filepath); (Button1)
void buildGraph(); (Button2)
void removeCycles(); (Button3)
void extractMinimalSpanningTree(); (Button3)
void buildNaryTree(); (Button4)

*  tutrle.cpp and .h I adjusted some codes, becuase we don't have qmath from qt (adjusted M_PI implmentation, and Changed qrand() to std::rand())

* tested: QT Project generated grammar can work in project 2


## Reference:
This project is inspired by the innovative approaches discussed in VCC's 2020 research on IPML. The study emphasizes the integration of IP (Intellectual Property) management with machine learning techniques to enhance the efficiency and effectiveness of IP-related tasks. For more details, visit [VCC Research on IPML](https://vcc.tech/research/2020/IPML).
