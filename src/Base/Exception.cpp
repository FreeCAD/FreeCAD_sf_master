/***************************************************************************
 *   (c) Jürgen Riegel (juergen.riegel@web.de) 2002                        *   
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License (LGPL)   *
 *   as published by the Free Software Foundation; either version 2 of     *
 *   the License, or (at your option) any later version.                   *
 *   for detail see the LICENCE text file.                                 *
 *                                                                         *
 *   FreeCAD is distributed in the hope that it will be useful,            *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        * 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with FreeCAD; if not, write to the Free Software        * 
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
 *   USA                                                                   *
 *                                                                         *
 *   Juergen Riegel 2002                                                   *
 ***************************************************************************/


#include "PreCompiled.h"


#include "Exception.h"
#include "Console.h"

using namespace Base;


TYPESYSTEM_SOURCE(Base::Exception,Base::BaseClass);


Exception::Exception(void)
{
  _sErrMsg = "FreeCAD Exception";
}

Exception::Exception(const Exception &inst)
: BaseClass(),_sErrMsg(inst._sErrMsg), _file(inst._file), _line(inst._line), _function(inst._function)
{
}


Exception::Exception(const char * sMessage)
 : _sErrMsg(sMessage)
{
}

Exception::Exception(const std::string& sMessage)
 : _sErrMsg(sMessage)
{
}

Exception::Exception(const std::string & file, const int line, const std::string & function)
: _file(file), _line(std::to_string(line)), _function(function)
{
}

Exception::Exception(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: _sErrMsg(sMessage), _file(file), _line(std::to_string(line)), _function(function)
{
}

Exception &Exception::operator=(const Exception &inst)
{
  _sErrMsg = inst._sErrMsg;
  return *this;
}

const char* Exception::what(void) const throw()
{
    return _sErrMsg.c_str();
}

void Exception::ReportException (void) const
{
    Console().Error("Exception (%s): %s \n",Console().Time(),report().c_str());
}

std::string Exception::report() const
{
    std::string str = "";

    if(!_sErrMsg.empty())
        str+= (_sErrMsg + " ");
    
    if(!_function.empty()) {
        str+="in ";
        str+=_function;
        str+= " ";
    }
    
    if(!_file.empty() && !_line.empty()) {
        // strip absolute path
        std::size_t pos = _file.find("src");
        
        if (pos!=std::string::npos) {
            str+="in ";
            str+= _file.substr(pos);
            str+= ":";
            str+=_line;
        }
    }
    
    return str;
}

// ---------------------------------------------------------

AbortException::AbortException(const char * sMessage)
  : Exception( sMessage )
{
}

AbortException::AbortException()
{
    _sErrMsg = "Aborted operation";
}

AbortException::AbortException(const AbortException &inst)
 : Exception(inst)
{
}

AbortException::AbortException(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
 : Exception(sMessage, file, line, function)
{}

AbortException::AbortException(const std::string & file, const int line, const std::string & function)
 : Exception(file, line, function)
{}

const char* AbortException::what() const throw()
{
    return Exception::what();
}

// ---------------------------------------------------------

XMLBaseException::XMLBaseException(const char * sMessage)
  : Exception(sMessage)
{
}

XMLBaseException::XMLBaseException(const std::string& sMessage)
  : Exception(sMessage)
{
}

XMLBaseException::XMLBaseException(const XMLBaseException &inst)
 : Exception(inst)
{
}

XMLBaseException::XMLBaseException(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

XMLParseException::XMLParseException(const char * sMessage)
  : Exception(sMessage)
{
}

XMLParseException::XMLParseException(const std::string& sMessage)
 : Exception(sMessage)
{
}

XMLParseException::XMLParseException()
{
    _sErrMsg = "XML parse exception";
}

XMLParseException::XMLParseException(const XMLParseException &inst)
 : Exception(inst)
{
}

XMLParseException::XMLParseException(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}


XMLParseException::XMLParseException(const std::string & file, const int line, const std::string & function)
: Exception(file, line, function)
{}

const char* XMLParseException::what() const throw()
{
    return Exception::what();
}

// ---------------------------------------------------------

FileException::FileException(const char * sMessage, const char * sFileName)
  : Exception( sMessage ),file(sFileName)
{
    if (sFileName) {
        _sErrMsgAndFileName = _sErrMsg + ": ";
        _sErrMsgAndFileName += sFileName;
    }
}

FileException::FileException(const char * sMessage, const FileInfo& File)
  : Exception( sMessage ),file(File)
{
    _sErrMsgAndFileName = _sErrMsg + ": ";
    _sErrMsgAndFileName += File.fileName();
}

FileException::FileException()
  : Exception( "Unknown file exception happened" )
{
}

FileException::FileException(const FileException &inst)
  : Exception( inst._sErrMsg.c_str() ),file(inst.file)
{
}

FileException::FileException(const std::string& sMessage, const char * sFileName, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function),file(sFileName)
{
    if (sFileName) {
        _sErrMsgAndFileName = _sErrMsg + ": ";
        _sErrMsgAndFileName += sFileName;
    }
}

FileException::FileException(const std::string& sMessage, const FileInfo& File, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function),file(File)
{
    _sErrMsgAndFileName = _sErrMsg + ": ";
    _sErrMsgAndFileName += File.fileName();
}

FileException::FileException(const std::string & file, const int line, const std::string & function)
: Exception("Unknown file exception happened",file, line, function)
{
    _sErrMsgAndFileName = _sErrMsg;
}

std::string FileException::getFileName() const
{
    return file.fileName();
}

const char* FileException::what() const throw()
{
    return _sErrMsgAndFileName.c_str();
}

// ---------------------------------------------------------

FileSystemError::FileSystemError(const char * sMessage)
  : Exception(sMessage)
{
}

FileSystemError::FileSystemError(const std::string& sMessage)
  : Exception(sMessage)
{
}

FileSystemError::FileSystemError(const FileSystemError &inst)
 : Exception(inst)
{
}

FileSystemError::FileSystemError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}


// ---------------------------------------------------------

BadFormatError::BadFormatError(const char * sMessage)
  : Exception(sMessage)
{
}

BadFormatError::BadFormatError(const std::string& sMessage)
  : Exception(sMessage)
{
}

BadFormatError::BadFormatError(const BadFormatError &inst)
 : Exception(inst)
{
}

BadFormatError::BadFormatError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

MemoryException::MemoryException()
{
    _sErrMsg = "Not enough memory available";
}

MemoryException::MemoryException(const MemoryException &inst)
#if defined (__GNUC__)
 : std::bad_alloc(), Exception(inst) 
#else
 : Exception(inst)
#endif
{
}

MemoryException::MemoryException(const std::string & file, const int line, const std::string & function)
: Exception(file, line, function)
{
    _sErrMsg = "Not enough memory available";
}

#if defined (__GNUC__)
const char* MemoryException::what() const throw()
{
    // call what() of Exception, not of std::bad_alloc
    return Exception::what();
}
#endif

// ---------------------------------------------------------

AccessViolation::AccessViolation()
{
    _sErrMsg = "Access violation";
}

AccessViolation::AccessViolation(const char * sMessage)
  : Exception(sMessage)
{
}

AccessViolation::AccessViolation(const std::string& sMessage)
  : Exception(sMessage)
{
}

AccessViolation::AccessViolation(const AccessViolation &inst)
 : Exception(inst)
{
}

AccessViolation::AccessViolation(const std::string & file, const int line, const std::string & function)
: Exception(file, line, function)
{
    _sErrMsg = "Access violation";
}

AccessViolation::AccessViolation(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}
// ---------------------------------------------------------

AbnormalProgramTermination::AbnormalProgramTermination()
{
    _sErrMsg = "Abnormal program termination";
}

AbnormalProgramTermination::AbnormalProgramTermination(const char * sMessage)
  : Exception(sMessage)
{
}

AbnormalProgramTermination::AbnormalProgramTermination(const std::string& sMessage)
  : Exception(sMessage)
{
}

AbnormalProgramTermination::AbnormalProgramTermination(const AbnormalProgramTermination &inst)
 : Exception(inst)
{
}

AbnormalProgramTermination::AbnormalProgramTermination(const std::string & file, const int line, const std::string & function)
: Exception(file, line, function)
{
    _sErrMsg = "Abnormal program termination";
}

AbnormalProgramTermination::AbnormalProgramTermination(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

UnknownProgramOption::UnknownProgramOption(const char * sMessage)
  : Exception(sMessage)
{
}

UnknownProgramOption::UnknownProgramOption(const std::string& sMessage)
  : Exception(sMessage)
{
}

UnknownProgramOption::UnknownProgramOption(const UnknownProgramOption &inst)
 : Exception(inst)
{
}

UnknownProgramOption::UnknownProgramOption(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

ProgramInformation::ProgramInformation(const char * sMessage)
  : Exception(sMessage)
{
}

ProgramInformation::ProgramInformation(const std::string& sMessage)
  : Exception(sMessage)
{
}

ProgramInformation::ProgramInformation(const ProgramInformation &inst)
 : Exception(inst)
{
}

ProgramInformation::ProgramInformation(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}


// ---------------------------------------------------------

TypeError::TypeError(const char * sMessage)
  : Exception(sMessage)
{
}

TypeError::TypeError(const std::string& sMessage)
  : Exception(sMessage)
{
}

TypeError::TypeError(const TypeError &inst)
 : Exception(inst)
{
}

TypeError::TypeError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

ValueError::ValueError(const char * sMessage)
  : Exception(sMessage)
{
}

ValueError::ValueError(const std::string& sMessage)
  : Exception(sMessage)
{
}

ValueError::ValueError(const ValueError &inst)
 : Exception(inst)
{
}

ValueError::ValueError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

IndexError::IndexError(const char * sMessage)
  : Exception(sMessage)
{
}

IndexError::IndexError(const std::string& sMessage)
  : Exception(sMessage)
{
}

IndexError::IndexError(const IndexError &inst)
 : Exception(inst)
{
}

IndexError::IndexError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

AttributeError::AttributeError(const char * sMessage)
  : Exception(sMessage)
{
}

AttributeError::AttributeError(const std::string& sMessage)
  : Exception(sMessage)
{
}

AttributeError::AttributeError(const AttributeError &inst)
 : Exception(inst)
{
}

AttributeError::AttributeError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

RuntimeError::RuntimeError(const char * sMessage)
  : Exception(sMessage)
{
}

RuntimeError::RuntimeError(const std::string& sMessage)
  : Exception(sMessage)
{
}

RuntimeError::RuntimeError(const RuntimeError &inst)
 : Exception(inst)
{
}

RuntimeError::RuntimeError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

NotImplementedError::NotImplementedError(const char * sMessage)
  : Exception(sMessage)
{
}

NotImplementedError::NotImplementedError(const std::string& sMessage)
  : Exception(sMessage)
{
}

NotImplementedError::NotImplementedError(const NotImplementedError &inst)
 : Exception(inst)
{
}

NotImplementedError::NotImplementedError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

DivisionByZeroError::DivisionByZeroError(const char * sMessage)
  : Exception(sMessage)
{
}

DivisionByZeroError::DivisionByZeroError(const std::string& sMessage)
  : Exception(sMessage)
{
}

DivisionByZeroError::DivisionByZeroError(const DivisionByZeroError &inst)
 : Exception(inst)
{
}

DivisionByZeroError::DivisionByZeroError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

ReferencesError::ReferencesError(const char * sMessage)
  : Exception(sMessage)
{
}

ReferencesError::ReferencesError(const std::string& sMessage)
  : Exception(sMessage)
{
}

ReferencesError::ReferencesError(const ReferencesError &inst)
 : Exception(inst)
{
}

// ---------------------------------------------------------

ExpressionError::ExpressionError(const char * sMessage)
  : Exception(sMessage)
{
}

ExpressionError::ExpressionError(const std::string& sMessage)
  : Exception(sMessage)
{
}

ExpressionError::ExpressionError(const ExpressionError &inst)
 : Exception(inst)
{
}

ExpressionError::ExpressionError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

ParserError::ParserError(const char * sMessage)
  : Exception(sMessage)
{
}

ParserError::ParserError(const std::string& sMessage)
  : Exception(sMessage)
{
}

ParserError::ParserError(const ParserError &inst)
 : Exception(inst)
{
}

ParserError::ParserError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

UnicodeError::UnicodeError(const char * sMessage)
  : Exception(sMessage)
{
}

UnicodeError::UnicodeError(const std::string& sMessage)
  : Exception(sMessage)
{
}

UnicodeError::UnicodeError(const UnicodeError &inst)
 : Exception(inst)
{
}

UnicodeError::UnicodeError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}
// ---------------------------------------------------------

OverflowError::OverflowError(const char * sMessage)
  : Exception(sMessage)
{
}

OverflowError::OverflowError(const std::string& sMessage)
  : Exception(sMessage)
{
}

OverflowError::OverflowError(const OverflowError &inst)
 : Exception(inst)
{
}

OverflowError::OverflowError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

UnderflowError::UnderflowError(const char * sMessage)
  : Exception(sMessage)
{
}

UnderflowError::UnderflowError(const std::string& sMessage)
  : Exception(sMessage)
{
}

UnderflowError::UnderflowError(const UnderflowError &inst)
 : Exception(inst)
{
}

UnderflowError::UnderflowError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------

UnitsMismatchError::UnitsMismatchError(const char * sMessage)
  : Exception(sMessage)
{
}

UnitsMismatchError::UnitsMismatchError(const std::string& sMessage)
  : Exception(sMessage)
{
}

UnitsMismatchError::UnitsMismatchError(const UnitsMismatchError &inst)
 : Exception(inst)
{
}

UnitsMismatchError::UnitsMismatchError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}

// ---------------------------------------------------------
 
CADKernelError::CADKernelError(const char * sMessage)
: Exception(sMessage)
{
}

CADKernelError::CADKernelError(const std::string& sMessage)
: Exception(sMessage)
{
}

CADKernelError::CADKernelError(const CADKernelError &inst)
: Exception(inst)
{
}

CADKernelError::CADKernelError(const std::string& sMessage, const std::string & file, const int line, const std::string & function)
: Exception(sMessage, file, line, function)
{}


// ---------------------------------------------------------

#if defined(__GNUC__) && defined (FC_OS_LINUX)
#include <stdexcept>
#include <iostream>

SignalException::SignalException()
{
    memset (&new_action, 0, sizeof (new_action));
    new_action.sa_handler = throw_signal;
    sigemptyset (&new_action.sa_mask);
    new_action.sa_flags = 0;
    ok = (sigaction (SIGSEGV, &new_action, &old_action) < 0);
#ifdef _DEBUG
    std::cout << "Set new signal handler" << std::endl;
#endif
}

SignalException::~SignalException()
{
    sigaction (SIGSEGV, &old_action, NULL);
#ifdef _DEBUG
    std::cout << "Restore old signal handler" << std::endl;
#endif
}

void SignalException::throw_signal(int signum)
{
    std::cerr << "SIGSEGV signal raised: " << signum << std::endl;
    throw std::runtime_error ("throw_signal");
}
#endif

