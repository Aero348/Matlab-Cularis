/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       FGBuoyantForces.cpp
 Authors:      Anders Gidenstam, Jon S. Berndt
 Date started: 01/21/08
 Purpose:      Encapsulates the buoyant forces

 ------------- Copyright (C) 2008 - 2009  Anders Gidenstam        -------------
 ------------- Copyright (C) 2008  Jon S. Berndt (jsb@hal-pc.org) -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 Place - Suite 330, Boston, MA  02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be found on
 the world wide web at http://www.gnu.org.

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------

HISTORY
--------------------------------------------------------------------------------
01/21/08   JSB   Created

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "FGBuoyantForces.h"
#include "FGMassBalance.h"
#include <input_output/FGPropertyManager.h>  // Need?

namespace JSBSim {

static const char *IdSrc = "$Id: FGBuoyantForces.cpp,v 1.7 2009/06/09 03:23:54 jberndt Exp $";
static const char *IdHdr = ID_BUOYANTFORCES;

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS IMPLEMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

FGBuoyantForces::FGBuoyantForces(FGFDMExec* FDMExec) : FGModel(FDMExec)
{
  Name = "FGBuoyantForces";

  NoneDefined = true;

  vTotalForces.InitMatrix();
  vTotalMoments.InitMatrix();

  gasCellJ.InitMatrix();

  bind();

  Debug(0);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGBuoyantForces::~FGBuoyantForces()
{
  for (unsigned int i=0; i<Cells.size(); i++) delete Cells[i];
  Cells.clear();

  Debug(1);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGBuoyantForces::InitModel(void)
{
  if (!FGModel::InitModel()) return false;

  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGBuoyantForces::Run(void)
{
  if (FGModel::Run()) return true;
  if (FDMExec->Holding()) return false; // if paused don't execute
  if (NoneDefined) return true;

  vTotalForces.InitMatrix();
  vTotalMoments.InitMatrix();

  for (unsigned int i=0; i<Cells.size(); i++) {
    Cells[i]->Calculate(FDMExec->GetDeltaT());
    vTotalForces  += Cells[i]->GetBodyForces();
    vTotalMoments += Cells[i]->GetMoments();
  }

  return false;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

bool FGBuoyantForces::Load(Element *element)
{
  string fname="", file="";
  Element *gas_cell_element;

  Debug(2);

  string separator = "/";

  fname = element->GetAttributeValue("file");
  if (!fname.empty()) {
    file = FDMExec->GetFullAircraftPath() + separator + fname;
    document = LoadXMLDocument(file);
  } else {
    document = element;
  }

  FGModel::Load(element); // Perform base class Load

  gas_cell_element = document->FindElement("gas_cell");
  while (gas_cell_element) {
    NoneDefined = false;
    Cells.push_back(new FGGasCell(FDMExec, gas_cell_element, Cells.size()));
    gas_cell_element = document->FindNextElement("gas_cell");
  }
  
  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double FGBuoyantForces::GetGasMass(void)
{
  double Gw = 0.0;

  for (unsigned int i = 0; i < Cells.size(); i++) {
    Gw += Cells[i]->GetMass();
  }

  return Gw;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const FGColumnVector3& FGBuoyantForces::GetGasMassMoment(void)
{
  vXYZgasCell_arm.InitMatrix();
  for (unsigned int i = 0; i < Cells.size(); i++) {
    vXYZgasCell_arm += Cells[i]->GetMassMoment();
  }
  return vXYZgasCell_arm;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

const FGMatrix33& FGBuoyantForces::GetGasMassInertia(void)
{
  const unsigned int size = Cells.size();
  
  if (size == 0) return gasCellJ;

  gasCellJ = FGMatrix33();

  for (unsigned int i=0; i < size; i++) {
    FGColumnVector3 v = MassBalance->StructuralToBody( Cells[i]->GetXYZ() );
    // Body basis is in FT. 
    const double mass = Cells[i]->GetMass();
    
    // FIXME: Verify that this is the correct way to change between the
    //        coordinate frames.
    gasCellJ += Cells[i]->GetInertia() + 
      FGMatrix33( 0,                - mass*v(1)*v(2), - mass*v(1)*v(3),
                  - mass*v(2)*v(1), 0,                - mass*v(2)*v(3),
                  - mass*v(3)*v(1), - mass*v(3)*v(2), 0 );
  }
  
  return gasCellJ;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string FGBuoyantForces::GetBuoyancyStrings(string delimeter)
{
  string CoeffStrings = "";
/*
  bool firstime = true;
  for (sd = 0; sd < variables.size(); sd++) {
    if (firstime) {
      firstime = false;
    } else {
      CoeffStrings += delimeter;
    }
    CoeffStrings += variables[sd]->GetName();
  }

  for (axis = 0; axis < 6; axis++) {
    for (sd = 0; sd < Coeff[axis].size(); sd++) {
      if (firstime) {
        firstime = false;
      } else {
        CoeffStrings += delimeter;
      }
      CoeffStrings += Coeff[axis][sd]->GetName();
    }
  }
*/
  return CoeffStrings;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string FGBuoyantForces::GetBuoyancyValues(string delimeter)
{
  string SDValues = "";
/*
  bool firstime = true;
  for (sd = 0; sd < variables.size(); sd++) {
    if (firstime) {
      firstime = false;
    } else {
      SDValues += delimeter;
    }
    SDValues += variables[sd]->GetValueAsString();
  }

  for (unsigned int axis = 0; axis < 6; axis++) {
    for (unsigned int sd = 0; sd < Coeff[axis].size(); sd++) {
      if (firstime) {
        firstime = false;
      } else {
        SDValues += delimeter;
      }
      SDValues += Coeff[axis][sd]->GetValueAsString();
    }
  }
*/
  return SDValues;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGBuoyantForces::bind(void)
{
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    The bitmasked value choices are as follows:
//    unset: In this case (the default) JSBSim would only print
//       out the normally expected messages, essentially echoing
//       the config files as they are read. If the environment
//       variable is not set, debug_lvl is set to 1 internally
//    0: This requests JSBSim not to output any messages
//       whatsoever.
//    1: This value explicity requests the normal JSBSim
//       startup messages
//    2: This value asks for a message to be printed out when
//       a class is instantiated
//    4: When this value is set, a message is displayed when a
//       FGModel object executes its Run() method
//    8: When this value is set, various runtime state variables
//       are printed out periodically
//    16: When set various parameters are sanity checked and
//       a message is printed out when they go out of bounds

void FGBuoyantForces::Debug(int from)
{
  if (debug_lvl <= 0) return;

  if (debug_lvl & 1) { // Standard console startup message output
    if (from == 2) { // Loader
      cout << endl << "  Buoyant Forces: " << endl;
    }
  }
  if (debug_lvl & 2 ) { // Instantiation/Destruction notification
    if (from == 0) cout << "Instantiated: FGBuoyantForces" << endl;
    if (from == 1) cout << "Destroyed:    FGBuoyantForces" << endl;
  }
  if (debug_lvl & 4 ) { // Run() method entry print for FGModel-derived objects
  }
  if (debug_lvl & 8 ) { // Runtime state variables
  }
  if (debug_lvl & 16) { // Sanity checking
  }
  if (debug_lvl & 64) {
    if (from == 0) { // Constructor
      cout << IdSrc << endl;
      cout << IdHdr << endl;
    }
  }
}

} // namespace JSBSim
