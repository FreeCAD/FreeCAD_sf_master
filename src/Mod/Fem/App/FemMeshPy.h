
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef FEM_FEMMESHPY_H
#define FEM_FEMMESHPY_H

#include <App/ComplexGeoDataPy.h>
#include <Mod/Fem/App/FemMesh.h>
#include <string>

namespace Fem
{

//===========================================================================
// FemMeshPy - Python wrapper
//===========================================================================

/** The python export class for FemMesh
 */
class FemExport FemMeshPy : public Data::ComplexGeoDataPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;};

public:
    FemMeshPy(FemMesh *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~FemMeshPy();

    typedef FemMesh* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the setShape() method
    static PyObject * staticCallback_setShape (PyObject *self, PyObject *args);
    /// implementer for the setShape() method
    PyObject*  setShape(PyObject *args);
    /// callback for the compute() method
    static PyObject * staticCallback_compute (PyObject *self, PyObject *args);
    /// implementer for the compute() method
    PyObject*  compute(PyObject *args);
    /// callback for the addHypothesis() method
    static PyObject * staticCallback_addHypothesis (PyObject *self, PyObject *args);
    /// implementer for the addHypothesis() method
    PyObject*  addHypothesis(PyObject *args);
    /// callback for the setStanardHypotheses() method
    static PyObject * staticCallback_setStanardHypotheses (PyObject *self, PyObject *args);
    /// implementer for the setStanardHypotheses() method
    PyObject*  setStanardHypotheses(PyObject *args);
    /// callback for the addNode() method
    static PyObject * staticCallback_addNode (PyObject *self, PyObject *args);
    /// implementer for the addNode() method
    PyObject*  addNode(PyObject *args);
    /// callback for the addEdge() method
    static PyObject * staticCallback_addEdge (PyObject *self, PyObject *args);
    /// implementer for the addEdge() method
    PyObject*  addEdge(PyObject *args);
    /// callback for the addFace() method
    static PyObject * staticCallback_addFace (PyObject *self, PyObject *args);
    /// implementer for the addFace() method
    PyObject*  addFace(PyObject *args);
    /// callback for the addQuad() method
    static PyObject * staticCallback_addQuad (PyObject *self, PyObject *args);
    /// implementer for the addQuad() method
    PyObject*  addQuad(PyObject *args);
    /// callback for the addVolume() method
    static PyObject * staticCallback_addVolume (PyObject *self, PyObject *args);
    /// implementer for the addVolume() method
    PyObject*  addVolume(PyObject *args);
    /// callback for the read() method
    static PyObject * staticCallback_read (PyObject *self, PyObject *args);
    /// implementer for the read() method
    PyObject*  read(PyObject *args);
    /// callback for the write() method
    static PyObject * staticCallback_write (PyObject *self, PyObject *args);
    /// implementer for the write() method
    PyObject*  write(PyObject *args);
    /// callback for the writeABAQUS() method
    static PyObject * staticCallback_writeABAQUS (PyObject *self, PyObject *args);
    /// implementer for the writeABAQUS() method
    PyObject*  writeABAQUS(PyObject *args);
    /// callback for the setTransform() method
    static PyObject * staticCallback_setTransform (PyObject *self, PyObject *args);
    /// implementer for the setTransform() method
    PyObject*  setTransform(PyObject *args);
    /// callback for the copy() method
    static PyObject * staticCallback_copy (PyObject *self, PyObject *args);
    /// implementer for the copy() method
    PyObject*  copy(PyObject *args);
    /// callback for the getVolumesByFace() method
    static PyObject * staticCallback_getVolumesByFace (PyObject *self, PyObject *args);
    /// implementer for the getVolumesByFace() method
    PyObject*  getVolumesByFace(PyObject *args);
    /// callback for the getccxVolumesByFace() method
    static PyObject * staticCallback_getccxVolumesByFace (PyObject *self, PyObject *args);
    /// implementer for the getccxVolumesByFace() method
    PyObject*  getccxVolumesByFace(PyObject *args);
    /// callback for the getNodeById() method
    static PyObject * staticCallback_getNodeById (PyObject *self, PyObject *args);
    /// implementer for the getNodeById() method
    PyObject*  getNodeById(PyObject *args);
    /// callback for the getNodesBySolid() method
    static PyObject * staticCallback_getNodesBySolid (PyObject *self, PyObject *args);
    /// implementer for the getNodesBySolid() method
    PyObject*  getNodesBySolid(PyObject *args);
    /// callback for the getNodesByFace() method
    static PyObject * staticCallback_getNodesByFace (PyObject *self, PyObject *args);
    /// implementer for the getNodesByFace() method
    PyObject*  getNodesByFace(PyObject *args);
    /// callback for the getNodesByEdge() method
    static PyObject * staticCallback_getNodesByEdge (PyObject *self, PyObject *args);
    /// implementer for the getNodesByEdge() method
    PyObject*  getNodesByEdge(PyObject *args);
    /// callback for the getNodesByVertex() method
    static PyObject * staticCallback_getNodesByVertex (PyObject *self, PyObject *args);
    /// implementer for the getNodesByVertex() method
    PyObject*  getNodesByVertex(PyObject *args);
    /// callback for the getElementNodes() method
    static PyObject * staticCallback_getElementNodes (PyObject *self, PyObject *args);
    /// implementer for the getElementNodes() method
    PyObject*  getElementNodes(PyObject *args);
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Nodes attribute
    static PyObject * staticCallback_getNodes (PyObject *self, void *closure);
    /// getter for the Nodes attribute
    Py::Dict getNodes(void) const;
    /// setter callback for the Nodes attribute
    static int staticCallback_setNodes (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Nodes is read only!
    ///getter callback for the NodeCount attribute
    static PyObject * staticCallback_getNodeCount (PyObject *self, void *closure);
    /// getter for the NodeCount attribute
    Py::Int getNodeCount(void) const;
    /// setter callback for the NodeCount attribute
    static int staticCallback_setNodeCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  NodeCount is read only!
    ///getter callback for the Edges attribute
    static PyObject * staticCallback_getEdges (PyObject *self, void *closure);
    /// getter for the Edges attribute
    Py::Tuple getEdges(void) const;
    /// setter callback for the Edges attribute
    static int staticCallback_setEdges (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Edges is read only!
    ///getter callback for the EdgeCount attribute
    static PyObject * staticCallback_getEdgeCount (PyObject *self, void *closure);
    /// getter for the EdgeCount attribute
    Py::Int getEdgeCount(void) const;
    /// setter callback for the EdgeCount attribute
    static int staticCallback_setEdgeCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  EdgeCount is read only!
    ///getter callback for the Faces attribute
    static PyObject * staticCallback_getFaces (PyObject *self, void *closure);
    /// getter for the Faces attribute
    Py::Tuple getFaces(void) const;
    /// setter callback for the Faces attribute
    static int staticCallback_setFaces (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Faces is read only!
    ///getter callback for the FaceCount attribute
    static PyObject * staticCallback_getFaceCount (PyObject *self, void *closure);
    /// getter for the FaceCount attribute
    Py::Int getFaceCount(void) const;
    /// setter callback for the FaceCount attribute
    static int staticCallback_setFaceCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  FaceCount is read only!
    ///getter callback for the TriangleCount attribute
    static PyObject * staticCallback_getTriangleCount (PyObject *self, void *closure);
    /// getter for the TriangleCount attribute
    Py::Int getTriangleCount(void) const;
    /// setter callback for the TriangleCount attribute
    static int staticCallback_setTriangleCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  TriangleCount is read only!
    ///getter callback for the QuadrangleCount attribute
    static PyObject * staticCallback_getQuadrangleCount (PyObject *self, void *closure);
    /// getter for the QuadrangleCount attribute
    Py::Int getQuadrangleCount(void) const;
    /// setter callback for the QuadrangleCount attribute
    static int staticCallback_setQuadrangleCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  QuadrangleCount is read only!
    ///getter callback for the PolygonCount attribute
    static PyObject * staticCallback_getPolygonCount (PyObject *self, void *closure);
    /// getter for the PolygonCount attribute
    Py::Int getPolygonCount(void) const;
    /// setter callback for the PolygonCount attribute
    static int staticCallback_setPolygonCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  PolygonCount is read only!
    ///getter callback for the Volumes attribute
    static PyObject * staticCallback_getVolumes (PyObject *self, void *closure);
    /// getter for the Volumes attribute
    Py::Tuple getVolumes(void) const;
    /// setter callback for the Volumes attribute
    static int staticCallback_setVolumes (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Volumes is read only!
    ///getter callback for the VolumeCount attribute
    static PyObject * staticCallback_getVolumeCount (PyObject *self, void *closure);
    /// getter for the VolumeCount attribute
    Py::Int getVolumeCount(void) const;
    /// setter callback for the VolumeCount attribute
    static int staticCallback_setVolumeCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  VolumeCount is read only!
    ///getter callback for the TetraCount attribute
    static PyObject * staticCallback_getTetraCount (PyObject *self, void *closure);
    /// getter for the TetraCount attribute
    Py::Int getTetraCount(void) const;
    /// setter callback for the TetraCount attribute
    static int staticCallback_setTetraCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  TetraCount is read only!
    ///getter callback for the HexaCount attribute
    static PyObject * staticCallback_getHexaCount (PyObject *self, void *closure);
    /// getter for the HexaCount attribute
    Py::Int getHexaCount(void) const;
    /// setter callback for the HexaCount attribute
    static int staticCallback_setHexaCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  HexaCount is read only!
    ///getter callback for the PyramidCount attribute
    static PyObject * staticCallback_getPyramidCount (PyObject *self, void *closure);
    /// getter for the PyramidCount attribute
    Py::Int getPyramidCount(void) const;
    /// setter callback for the PyramidCount attribute
    static int staticCallback_setPyramidCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  PyramidCount is read only!
    ///getter callback for the PrismCount attribute
    static PyObject * staticCallback_getPrismCount (PyObject *self, void *closure);
    /// getter for the PrismCount attribute
    Py::Int getPrismCount(void) const;
    /// setter callback for the PrismCount attribute
    static int staticCallback_setPrismCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  PrismCount is read only!
    ///getter callback for the PolyhedronCount attribute
    static PyObject * staticCallback_getPolyhedronCount (PyObject *self, void *closure);
    /// getter for the PolyhedronCount attribute
    Py::Int getPolyhedronCount(void) const;
    /// setter callback for the PolyhedronCount attribute
    static int staticCallback_setPolyhedronCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  PolyhedronCount is read only!
    ///getter callback for the SubMeshCount attribute
    static PyObject * staticCallback_getSubMeshCount (PyObject *self, void *closure);
    /// getter for the SubMeshCount attribute
    Py::Int getSubMeshCount(void) const;
    /// setter callback for the SubMeshCount attribute
    static int staticCallback_setSubMeshCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  SubMeshCount is read only!
    ///getter callback for the GroupCount attribute
    static PyObject * staticCallback_getGroupCount (PyObject *self, void *closure);
    /// getter for the GroupCount attribute
    Py::Int getGroupCount(void) const;
    /// setter callback for the GroupCount attribute
    static int staticCallback_setGroupCount (PyObject *self, PyObject *value, void *closure);
    // no setter method,  GroupCount is read only!
    ///getter callback for the Volume attribute
    static PyObject * staticCallback_getVolume (PyObject *self, void *closure);
    /// getter for the Volume attribute
    Py::Object getVolume(void) const;
    /// setter callback for the Volume attribute
    static int staticCallback_setVolume (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Volume is read only!
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    FemMesh *getFemMeshPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Fem

#endif  // FEM_FEMMESHPY_H


