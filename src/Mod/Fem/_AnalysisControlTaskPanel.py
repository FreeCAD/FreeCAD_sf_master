#***************************************************************************
#*                                                                         *
#*   Copyright (c) 2013-2015 - Juergen Riegel <FreeCAD@juergen-riegel.net> *
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

__title__ = "Job Control Task Panel"
__author__ = "Juergen Riegel"
__url__ = "http://www.freecadweb.org"

from FemTools import FemTools
import ccxFrdReader

import FreeCAD
import os
import time

if FreeCAD.GuiUp:
    import FreeCADGui
    import FemGui
    from PySide import QtCore, QtGui
    from PySide.QtCore import Qt
    from PySide.QtGui import QApplication


class _AnalysisControlTaskPanel:
    def __init__(self, analysis_object):
        self.form = FreeCADGui.PySideUic.loadUi(FreeCAD.getHomePath() + "Mod/Fem/MechanicalAnalysis.ui")
        self.fem_prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Fem")
        
        self.analysis_object = analysis_object
        self.solver_object=self.analysis_object.getSolver()
        self.solver_python_object=self.solver_object.Proxy #??? how to get the pure python object?
        self.working_dir = self.solver_object.getProperty("WorkingDir") #???
        self.SolverProcess = QtCore.QProcess()
        self.Timer = QtCore.QTimer()
        self.Timer.start(300)

        # to be moved into SolverObjectPythonProxy class
        if self.solver_object.Name="Calculix":
            ccx_binary = self.fem_prefs.GetString("ccxBinaryPath", "")
            if ccx_binary:
                self.CalculixBinary = ccx_binary
                print "Using ccx binary path from FEM preferences: {}".format(ccx_binary)
            else:
                from platform import system
                if system() == 'Linux':
                    self.CalculixBinary = 'ccx'
                elif system() == 'Windows':
                    self.CalculixBinary = FreeCAD.getHomePath() + 'bin/ccx.exe'
                else:
                    self.CalculixBinary = 'ccx'
            self.fem_prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Fem") # why twice? refresh?
            
        self.fem_console_message = ''

        #Connect Signals and Slots
        QtCore.QObject.connect(self.form.tb_choose_working_dir, QtCore.SIGNAL("clicked()"), self.choose_working_dir)
        QtCore.QObject.connect(self.form.pushButton_write, QtCore.SIGNAL("clicked()"), self.write_input_file_handler)
        QtCore.QObject.connect(self.form.pushButton_edit, QtCore.SIGNAL("clicked()"), self.editCaseInputFile)
        QtCore.QObject.connect(self.form.pushButton_generate, QtCore.SIGNAL("clicked()"), self.runSolverProcess)

        QtCore.QObject.connect(self.SolverProcess, QtCore.SIGNAL("started()"), self.solverProcessStarted)
        QtCore.QObject.connect(self.SolverProcess, QtCore.SIGNAL("stateChanged(QProcess::ProcessState)"), self.solverProcessStateChanged)
        QtCore.QObject.connect(self.SolverProcess, QtCore.SIGNAL("error(QProcess::ProcessError)"), self.solverProcessError)
        QtCore.QObject.connect(self.SolverProcess, QtCore.SIGNAL("finished(int)"), self.solverProcessFinished)

        QtCore.QObject.connect(self.Timer, QtCore.SIGNAL("timeout()"), self.updateText)

        self.update()

    def femConsoleMessage(self, message="", color="#000000"):
        self.fem_console_message = self.fem_console_message + '<font color="#0000FF">{0:4.1f}:</font> <font color="{1}">{2}</font><br>'.\
            format(time.time() - self.Start, color, message.encode('utf-8', 'replace'))
        self.form.textEdit_Output.setText(self.fem_console_message)
        self.form.textEdit_Output.moveCursor(QtGui.QTextCursor.End)

    def printSolverProcessStdout(self):
        out = self.SolverProcess.readAllStandardOutput()
        if out.isEmpty():
            self.femConsoleMessage("Solver stdout is empty", "#FF0000")
        else:
            try:
                out = unicode(out, 'utf-8', 'replace')
                rx = QtCore.QRegExp("\\*ERROR.*\\n\\n")
                rx.setMinimal(True)
                pos = rx.indexIn(out)
                while not pos < 0:
                    match = rx.cap(0)
                    FreeCAD.Console.PrintError(match.strip().replace('\n', ' ') + '\n')
                    pos = rx.indexIn(out, pos + 1)
                out = os.linesep.join([s for s in out.splitlines() if s])
                self.femConsoleMessage(out.replace('\n', '<br>'))
            except UnicodeDecodeError:
                self.femConsoleMessage("Error converting stdout from Solver", "#FF0000")

    def updateText(self):
        if(self.SolverProcess.state() == QtCore.QProcess.ProcessState.Running):
            self.form.label_Time.setText('Time: {0:4.1f}: '.format(time.time() - self.Start))

    def solverProcessError(self, error):
        print "Error()", error
        self.femConsoleMessage("Solver execute error: {}".format(error), "#FF0000")

    def solverProcessStarted(self):
        print "SolverProcessStarted()"
        print self.SolverProcess.state()
        self.form.pushButton_generate.setText("Break Solver process")

    def solverProcessStateChanged(self, newState):
        if (newState == QtCore.QProcess.ProcessState.Starting):
                self.femConsoleMessage("Starting Solver...")
        if (newState == QtCore.QProcess.ProcessState.Running):
                self.femConsoleMessage("Solver is running...")
        if (newState == QtCore.QProcess.ProcessState.NotRunning):
                self.femConsoleMessage("Solver stopped.")

    def solverProcessFinished(self, exitCode):
        print "solverProcessFinished()", exitCode
        print self.SolverProcess.state()

        # Restore previous cwd
        QtCore.QDir.setCurrent(self.cwd)

        self.printSolverProcessStdout()
        self.Timer.stop()

        self.femConsoleMessage("External solver process is done!", "#00AA00")
        self.form.pushButton_generate.setText("Re-run Solver")
        
        if self.solver_object.Name="Calculix":
            print "Loading results...."
            self.femConsoleMessage("Loading result sets...")
            self.form.label_Time.setText('Time: {0:4.1f}: '.format(time.time() - self.Start))
            fea = FemTools()
            fea.reset_all()
            frd_result_file = os.path.splitext(self.inp_file_name)[0] + '.frd'
            if os.path.isfile(frd_result_file):
                QApplication.setOverrideCursor(Qt.WaitCursor)
                ccxFrdReader.importFrd(frd_result_file, FemGui.getActiveAnalysis())
                
                self.femConsoleMessage("Loading results done!", "#00AA00")
            else:
                self.femConsoleMessage("Loading results failed! Results file doesn\'t exist", "#FF0000")
            self.form.label_Time.setText('Time: {0:4.1f}: '.format(time.time() - self.Start))
            
        elif self.solver_object.Name="OpenFOAM":
            self.femConsoleMessage("Please use external view to view the result", "#00AA00")
            #activate the viewExternal command? 
        else:
            print "do nothing "
        QApplication.restoreOverrideCursor()
        

    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Close)

    def update(self):
        'fills the widgets'
        self.form.le_working_dir.setText(self.working_dir)
        return

    def accept(self):
        FreeCADGui.Control.closeDialog()

    def reject(self):
        FreeCADGui.Control.closeDialog()

    def choose_working_dir(self):
        self.fem_prefs = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Fem")
        self.working_dir = QtGui.QFileDialog.getExistingDirectory(None,
                                                                  'Choose Solver working directory',
                                                                  self.fem_prefs.GetString("WorkingDir", '/tmp'))
        if self.working_dir:
            self.fem_prefs.SetString("WorkingDir", str(self.working_dir))
            self.form.le_working_dir.setText(self.working_dir)
            #Todo: also save into DocumentObject property for persistence

    def write_input_file_handler(self):
        QApplication.restoreOverrideCursor()
        if self.solver_object.Name="Calculix":
            if self.check_prerequisites_helper():
                QApplication.setOverrideCursor(Qt.WaitCursor)
                self.inp_file_name = ""
                fea = FemTools()
                fea.update_objects()
                fea.write_inp_file()
                if fea.inp_file_name != "":
                    self.inp_file_name = fea.inp_file_name
                    self.femConsoleMessage("Write completed.")
                    self.form.pushButton_edit.setEnabled(True)
                    self.form.pushButton_generate.setEnabled(True)
                else:
                    self.femConsoleMessage("Write .inp file failed!", "#FF0000")
                QApplication.restoreOverrideCursor()
        elif self.solver_object.Name="OpenFOAM":
            self.femConsoleMessage("{} case writer is called".format(self.solver_object.Name))
            #
            self.femConsoleMessage("Write {} case is completed.".format(self.solver_object.Name))
        else:
            self.femConsoleMessage("Error: Solver {} is not valid!".format(self.solver_object.Name), "#AAOO00")

    def check_prerequisites_helper(self):
        self.Start = time.time()
        self.femConsoleMessage("Check dependencies...")
        self.form.label_Time.setText('Time: {0:4.1f}: '.format(time.time() - self.Start))
        
        if self.solver_object.Name="Calculix":
            fea = FemTools()
            fea.update_objects()
            message = fea.check_prerequisites()
        
        if message != "":
            QtGui.QMessageBox.critical(None, "Missing prerequisit(s)", message)
            return False
        return True

    def start_ext_editor(self, ext_editor_path, filename):
        if not hasattr(self, "ext_editor_process"):
            self.ext_editor_process = QtCore.QProcess() #???
        if self.ext_editor_process.state() != QtCore.QProcess.Running:
            self.ext_editor_process.start(ext_editor_path, [filename])

    def editSolverInputFile(self):
        if self.solver_object.Name="Calculix":
            print 'editCalculixInputFile {}'.format(self.inp_file_name)
            if self.fem_prefs.GetBool("UseInternalEditor", True):
                FemGui.open(self.inp_file_name)
            else:
                ext_editor_path = self.fem_prefs.GetString("ExternalEditorPath", "")
                if ext_editor_path:
                    self.start_ext_editor(ext_editor_path, self.inp_file_name)
                else:
                    print "External editor is not defined in FEM preferences. Falling back to internal editor"
                    FemGui.open(self.inp_file_name)
        else:
            self.femConsoleMessage("Edit case input file in FreeCAD is not implemented!")

    def runSolverProcess(self):
        print 'run solver process'
        self.Start = time.time()
        self.femConsoleMessage("Run {}...".format(self.solver_object.Name))
        
        if self.solver_object.Name="Calculix":
            # run Calculix
            self.femConsoleMessage("Solver binary: {}".format(self.CalculixBinary))
            print 'run Calculix at: ', self.CalculixBinary, ' with: ', os.path.splitext(self.inp_file_name)[0]
            # change cwd because ccx may crash if directory has no write permission
            # there is also a limit of the length of file names so jump to the document directory
            self.cwd = QtCore.QDir.currentPath()
            fi = QtCore.QFileInfo(self.inp_file_name)
            QtCore.QDir.setCurrent(fi.path())
            self.SolverProcess.start(self.CalculixBinary, ['-i', fi.baseName()])
            
        elif self.solver_object.Name="OpenFOAM":
            self.femConsoleMessage("Run OpenFoam...")
        else:
            pass

        QApplication.restoreOverrideCursor()
