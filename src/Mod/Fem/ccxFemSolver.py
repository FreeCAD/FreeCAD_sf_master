#***************************************************************************
#*                                                                         *
#*   Copyright (c) 2015 - Przemo Firszt <przemo@firszt.eu>  
#*   Copyright (c) 2015 - Qingfeng Xia <qingfeng.xia@  eng.ox.ac.uk> *
#*                                                                         *
#*   This program is free software; you can redistribute it and/or modify  *
#*   it under the terms of the GNU Lesser General Public License (LGPL)    *
#*   as published by the Free Software Foundation; either version 2 of     *
#*   the License, or (at your option) any later version.                   *
#*   for detail see the LICENCE text file.                                 *
#*                                                                         *
#*   This program is distributed in the hope that it will be useful,       *
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#*   GNU Library General Public License for more details.                  *
#*                                                                         *
#*   You should have received a copy of the GNU Library General Public     *
#*   License along with this program; if not, write to the Free Software   *
#*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
#*   USA                                                                   *
#*                                                                         *
#***************************************************************************

from PreImport import *
   
import os.path
from CaeSolver import CaeSolver

class Solver(CaeSolver):
    """ Using OpenFoam solver derived from base class:  CaeSolver
    This class focus on mesh and boundary condition setup for FEM
    """
    
    def __init__(self,analysis):
        #
        if analysis==None:
            if FreeCAD.GuiUp:
                self.analysis=FemGui.getActiveAnalysis()
            else:
                self.analysis=None # can  be set later
        else:
            self.analysis=analysis
        print "type of FemGui.getActiveAnalysis() = ",type(self.analysis)#debug
        
        super(self.__class__, self).__init__()  #
        
        self.module="Fem" #python module name
        self.category="Fem" 
        self.name="CalculiX"
        self.minVersion=(2,0,0) #
        self.known_analysis_types = ["static", "frequency"] #specific solver type
        
        self.parallel=False
        #self.writer: write case  and convert mesh and boundary condition to native solver's format
        #self.reader: convert the native result to FreeCAD supported for rendering,
        self.prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/"+self.catogory) #set in tool->parameter editor
        
            
        super(self.__class__, self).setup_solver() #
        #solver specific setup following the general setup
        self.set_analysis_type()
        self.set_eigenmode_parameters()
        
        
    def check_prerequisites(self):
        """analysis and solver check"""
        message = super(self.__class__, self).check_prerequisites()
        
        if not self.fixed_constraints:
            message += "No fixed-constraint nodes defined in the Analysis\n"
        if self.analysis_type == "static":
            if not (self.force_constraints or self.pressure_constraints):
                message += "No force-constraint or pressure-constraint defined in the Analysis\n"
        if self.beam_sections:
            has_no_references = False
            for b in self.beam_sections:
                if len(b['Object'].References) == 0:
                    if has_no_references is True:
                        message += "More than one BeamSection has empty References list (Only one empty References list is allowed!).\n"
                    has_no_references = True
        if self.shell_thicknesses:
            has_no_references = False
            for s in self.shell_thicknesses:
                if len(s['Object'].References) == 0:
                    if has_no_references is True:
                        message += "More than one ShellThickness has empty References list (Only one empty References list is allowed!).\n"
                    has_no_references = True
                    
        return message
        
    def set_analysis_type(self, analysis_type=None):
        if analysis_type is None:
            self.analysis_type = "static"
        else:
            self.analysis_type = analysis_type
            
    ## sets eigenmode parameters for CalculiX frequency analysis
    #  @param self The python object self
    #  @param number number of eigenmodes that wll be calculated, default 10
    #  @param limit_low lower value of requested eigenfrequency range, default 0.0
    #  @param limit_high higher value of requested eigenfrequency range, default 1000000.0
    def set_eigenmode_parameters(self, number=10, limit_low=0.0, limit_high=1000000.0):
        self.eigenmode_parameters = (number, limit_low, limit_high)


    #####################################################
    def write_case_file(self):
        """ analysis should be an object contains {solver, mesh, material, constraint}
        ccxInpWriter, API needs simplification
        """
        self.update_objects() # where self.mesh is set
        message=self.check_prerequisites() #if not None showMessage
        if message:
            QtGui.QMessageBox.critical(None, "Missing prerequisite", message)
            return
        
        import ccxInpWriter as iw
        import sys
        self.case_file_name = "" 
        try:
            inp_writer = iw.inp_writer(self.analysis, self.mesh, self.material,
                                       self.fixed_constraints,
                                       self.force_constraints, self.pressure_constraints,
                                       self.beam_sections, self.shell_thicknesses,
                                       self.analysis_type, self.eigenmode_parameters,
                                       self.working_dir)
            #FreeCAD.Console.PrintMessage("Debuginfo: built inp_writer, to call write")
            self.case_file_name = inp_writer.write_calculix_input_file()
            #FreeCAD.Console.PrintMessage("Debuginfo: write_calculix_input_file() return")
            self.solver_command_string=self.solver_binary +' -i '+os.path.basename(self.case_file_name)
        except:
            FreeCAD.Console.PrintMessage("Unexpected error when writing CalculiX input file:", sys.exc_info()[0])
            raise
            
    def start_ext_editor(self, ext_editor_path, filename):
        if not hasattr(self, "ext_editor_process"):
            self.ext_editor_process = QtCore.QProcess()
        if self.ext_editor_process.state() != QtCore.QProcess.Running:
            self.ext_editor_process.start(ext_editor_path, [filename])

    def editSolverInputFile(self):
        print 'editCalculixInputFile {}'.format(self.case_file_name)
        if self.prefs.GetBool("UseInternalEditor", True):
            FemGui.open(self.case_file_name)
        else:
            ext_editor_path = self.prefs.GetString("ExternalEditorPath", "")
            if ext_editor_path:
                self.start_ext_editor(ext_editor_path, self.case_file_name)
            else:
                print "External editor is not defined in FEM preferences. Falling back to internal editor"
                FemGui.open(self.case_file_name) #got grammer highlighter for inp file
                
    #####################################################
    def setup_solver_binary_path(self, solver_name=None):
        """ if not specify, setup binary solver name from prefs a default one
        """
        if not solver_name:
            solver_name= self.prefs.GetString("solverBinaryPath",'ccx') #<FemToCae> need to  be changed later
            from platform import system
            if system() == "Windows":
                solver_binary = FreeCAD.getHomePath() + "\\bin\\"+solver_name+".exe"
            else: #"Linux" or  "MACOSX" 
                solver_binary = solver_name
        else:
            solver_binary = solver_name
        self.solver_binary = solver_binary
        
    def set_solver_env(self):
        """solver specific env variable setup, this 
        """           
        if self.parallel:
            import multiprocessing
            self._ont_backup = os.environ.get('OMP_NUM_THREADS') #OPEM-MPI
            if not self._ont_backup:
                self.ont_backup = ""
            self._env = os.putenv('OMP_NUM_THREADS', str(multiprocessing.cpu_count()))
        else:
            self._env=None #
        
    def unset_solver_env(self):
        if self.parallel:
            os.putenv('OMP_NUM_THREADS', self._ont_backup)

    ##########################################################
    def load_results(self):
        import ccxFrdReader #
        import os
        self.results_present = False
        result_file=os.path.splitext(self.case_file_name)[0] + '.frd'
        if os.path.isfile(result_file):
            ccxFrdReader.importFrd(result_file, self.analysis)
            for m in self.analysis.Member:
                if m.isDerivedFrom("Fem::FemResultObject"):
                    self.result_object = m
            if self.result_object is not None:
                self.results_present = True
        else:
            raise Exception('FEM: No results found at {}!'.format(result_file))

    def use_results(self, results_name=None):
        for m in self.analysis.Member:
            if m.isDerivedFrom("Fem::FemResultObject") and m.Name == results_name:
                self.result_object = m
                break
        if not self.result_object:
            raise ("{} doesn't exist".format(results_name))
    ## returns minimum, average and maximum value for provided result type
    #  @param self The python object self
    #  @result_type Type of FEM result, allowed U1, U2, U3, Uabs, Sabs and None
    def get_stats(self, result_type):
        stats = (0.0, 0.0, 0.0)
        for m in self.analysis.Member:
            if m.isDerivedFrom("Fem::FemResultObject"):
                match = {"U1": (m.Stats[0], m.Stats[1], m.Stats[2]),
                         "U2": (m.Stats[3], m.Stats[4], m.Stats[5]),
                         "U3": (m.Stats[6], m.Stats[7], m.Stats[8]),
                         "Uabs": (m.Stats[9], m.Stats[10], m.Stats[11]),
                         "Sabs": (m.Stats[12], m.Stats[13], m.Stats[14]),
                         "None": (0.0, 0.0, 0.0)}
                stats = match[result_type]
        return stats
        
        
    #########################################################
    def show_result(self, result_type="Sabs", limit=None):
        """ solver specific result field
        Sabs (Stress), Uabs (Displacement), U1, U2,U3 (displacement vector)
        """
        self.update_objects()
        if result_type == "None":
            self.reset_mesh_color()
            return
        if self.result_object:
            if result_type == "Sabs":
                values = self.result_object.StressValues
            elif result_type == "Uabs":
                values = self.result_object.DisplacementLengths
            else:
                match = {"U1": 0, "U2": 1, "U3": 2}
                d = zip(*self.result_object.DisplacementVectors)
                values = list(d[match[result_type]])
            self.show_scalar_color_by_with_cutoff(values, limit)
    
    def show_scalar_color_by_with_cutoff(self, values, limit=None):
        """ This is solver dependent, can be moved 
        """
        if limit:
            filtered_values = []
            for v in values:
                if v > limit:
                    filtered_values.append(limit)
                else:
                    filtered_values.append(v)
        else:
            filtered_values = values
        self.mesh.ViewObject.setNodeColorByScalars(self.result_object.ElementNumbers, filtered_values)
    
    def show_displacement(self, displacement_factor=0.0):
        self.mesh.ViewObject.setNodeDisplacementByVectors(self.result_object.ElementNumbers,
                                                          self.result_object.DisplacementVectors)
        self.mesh.ViewObject.applyDisplacement(displacement_factor)
        

    def update_objects(self):
        """
        """
        # [{'Object':material}, {}, ...]
        # [{'Object':fixed_constraints, 'NodeSupports':bool}, {}, ...]
        # [{'Object':force_constraints, 'NodeLoad':value}, {}, ...
        # [{'Object':pressure_constraints, 'xxxxxxxx':value}, {}, ...]
        # [{'Object':beam_sections, 'xxxxxxxx':value}, {}, ...]
        # [{'Object':shell_thicknesses, 'xxxxxxxx':value}, {}, ...]
        self.mesh = None
        self.material = []
        self.fixed_constraints = []
        self.force_constraints = []
        self.pressure_constraints = []
        self.beam_sections = []
        self.shell_thicknesses = []

        for m in self.analysis.Member:
            if m.isDerivedFrom("Fem::FemMeshObject"):
                self.mesh = m
            elif m.isDerivedFrom("App::MaterialObjectPython"):
                material_dict = {}
                material_dict['Object'] = m
                self.material.append(material_dict)
            elif m.isDerivedFrom("Fem::ConstraintFixed"):
                fixed_constraint_dict = {}
                fixed_constraint_dict['Object'] = m
                self.fixed_constraints.append(fixed_constraint_dict)
            elif m.isDerivedFrom("Fem::ConstraintForce"):
                force_constraint_dict = {}
                force_constraint_dict['Object'] = m
                self.force_constraints.append(force_constraint_dict)
            elif m.isDerivedFrom("Fem::ConstraintPressure"):
                PressureObjectDict = {}
                PressureObjectDict['Object'] = m
                self.pressure_constraints.append(PressureObjectDict)
            elif hasattr(m, "Proxy") and m.Proxy.Type == 'FemBeamSection':
                beam_section_dict = {}
                beam_section_dict['Object'] = m
                self.beam_sections.append(beam_section_dict)
            elif hasattr(m, "Proxy") and m.Proxy.Type == 'FemShellThickness':
                shell_thickness_dict = {}
                shell_thickness_dict['Object'] = m
                self.shell_thicknesses.append(shell_thickness_dict)
    
    ################################################
    # Removes all result objects,  define only one API  reset_all() to clean analysis
    #  @param self The python object self
    def purge_results(self):
        if self.results_present:
            for m in self.analysis.Member:
                if (m.isDerivedFrom('Fem::FemResultObject')):  # this CFD module still using FEM object to render 
                    FreeCAD.ActiveDocument.removeObject(m.Name)
            self.results_present = False
    
    ## Resets mesh deformation, 
    #  @param self The python object self
    def reset_mesh_deformation(self):
        if self.results_present and self.mesh:
            self.mesh.ViewObject.applyDisplacement(0.0)
    
    ## Resets mesh color
    #  @param self The python object self
    def reset_mesh_color(self):
        if self.results_present and self.mesh:
            self.mesh.ViewObject.NodeColor = {}
            self.mesh.ViewObject.ElementColor = {}
            self.mesh.ViewObject.setNodeColorByScalars()




    