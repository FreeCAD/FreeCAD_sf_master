#include <App/Application.h>
#include <App/Document.h>
#include <Base/Precision.h>
#include "Mod/Part/App/FeaturePartBox.h"
#include "Mod/Part/App/FeaturePartFuse.h"
#include "Mod/Part/App/FeatureFillet.h"
#include <BRepGProp.hxx>
#include "Base/Interpreter.h"
#include <boost/algorithm/string/regex.hpp>
#include <boost/format.hpp>


namespace PartTestHelpers
{

double getVolume(TopoDS_Shape shape);

std::vector<Part::FilletElement>
_getFilletEdges(std::vector<int> edges, double startRadius, double endRadius);

class PartTestHelperClass
{
public:
    App::Document* _doc;
    std::string _docName;
    std::array<Part::Box*, 6> _boxes;
    void createTestDoc();
};

const double minimalDistance = Base::Precision::Confusion() * 1000;

void executePython(std::vector<std::string> python);

void rectangle(double height, double width, char *name);
}  // namespace PartTestHelpers
