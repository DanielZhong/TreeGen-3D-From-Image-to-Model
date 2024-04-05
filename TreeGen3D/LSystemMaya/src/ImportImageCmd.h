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
#include "tree_structure.h"
#include "turtle.h"
#include "udgcd_cycle_detector.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/range/irange.hpp>

// Qt
#include <QGLWidget>
#include <QPaintEvent>
#include <QtPrintSupport/QPrinter>
#include <QtSvg>

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> UndirectedGraph;
typedef boost::graph_traits<UndirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits<UndirectedGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<UndirectedGraph>::edge_descriptor edge_descriptor;

class ImportImageCmd : public MPxCommand {
public:
    ImportImageCmd();
    virtual ~ImportImageCmd();
    MStatus doIt(const MArgList& args);
    static void* creator();
    static MSyntax newSyntax();

    MStatus parseBoundingBoxData(const std::string& filepath);
    void buildGraph();
    void removeCycles();
    void extractMinimalSpanningTree();
    void buildNaryTree();
private:
	
};

#endif // IMPORTIMAGECMD_H
