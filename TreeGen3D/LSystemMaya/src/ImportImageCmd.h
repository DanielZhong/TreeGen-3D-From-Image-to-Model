#ifndef IMPORTIMAGECMD_H
#define IMPORTIMAGECMD_H

#include <maya/MPxCommand.h>
#define MNoPluginEntry
#define MNoVersionString
#include <maya/MFnPlugin.h>
#undef MNoPluginEntry
#undef MNoVersionString
#include <maya/MArgList.h>
#include <maya/MSyntax.h>

class ImportImageCmd : public MPxCommand {
public:
    ImportImageCmd();
    virtual ~ImportImageCmd();
    MStatus doIt(const MArgList& args);
    static void* creator();
    static MSyntax newSyntax();

    MStatus parseBoundingBoxData(const std::string& filepath);
};

#endif // IMPORTIMAGECMD_H
