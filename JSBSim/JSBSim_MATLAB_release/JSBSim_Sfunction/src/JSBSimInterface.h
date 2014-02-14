#ifndef JSBSIMINTERFACE_HEADER_H
#define JSBSIMINTERFACE_HEADER_H

#include "mex.h"
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGAuxiliary.h>
#include <models/FGPropulsion.h>
#include <models/FGFCS.h>

using namespace JSBSim;

class JSBSimInterface
{
public:
	FGFDMExec *fdmExec;
	JSBSimInterface(FGFDMExec *, double dt);
	~JSBSimInterface(void);
	/// Open an aircraft model from Matlab
	bool Open(string prop);
	/// Get a property from the catalog
	bool GetPropertyValue(const mxArray *prhs1, double& value);
	/// Set a property in the catalog
	bool SetPropertyValue(const mxArray *prhs1, const mxArray *prhs2);
	/// Set a property in the catalog
	bool SetPropertyValue(const string prop, const double value);
	/// Enables a number of commonly used settings
	bool EasySetValue(const string prop, const double value);
	/// Check if the given string is present in the catalog
	bool QueryJSBSimProperty(string prop);
	/// Print the aircraft catalog
	void PrintCatalog(void);
	bool IsAircraftLoaded(){return _ac_model_loaded;}
	/// Set an initial state
	/*
		*prhs1 is a Matlab structure of strings/values couples, 
			e.g. ['u','v','w', 'p','q','r', 'phi','tht','psi', 'lat','lon','h' ]
			[ 80,0,0, 0,0,0, 0,2,0, 36,44,1000 ]
			where
			(*rhs1)(4).name = 'p'; (*rhs1)(1).value = 80;
	*/
	bool ResetToInitialCondition(void);
	bool Init(const mxArray *prhs1);
	/// put the 16 dotted quantities into statedot:
	/*
	dot of (u,v,w,p,q,r,q1,q2,q3,q4,x,y,z,phi,theta,psi)
	*/
	//bool GetEngNum(real_T *num_of_eng);
	//bool GetEngType(string* engtype);

	// Wrapper functions to the FGFDMExec class
	bool RunFDMExec() {return fdmExec->Run();}
	bool RunPropagate() {return propagate->Run();}
	bool RunAuxiliary() {return auxiliary->Run();}
	bool RunPropulsion() {return propulsion->Run();}
	bool RunFCS() {return fcs->Run();}

	enum JIVerbosityLevel {eSilent=0, eVerbose, eVeryVerbose, eDebug} verbosityLevel;

	// Set verbosity level
	bool SetVerbosity(const mxArray *prhs1);
	void SetVerbosity(const JIVerbosityLevel vl) {verbosityLevel = vl;}
	
	// Additional Functions added for SFunction
	void SetMultiplier(double multiple){ x_times = multiple;}
	double	GetMultiplier(){return x_times;}
	//double JSBSimInterface::GetEulerDot(int i);
	bool JSBSimInterface::SetEuler(int i, double value);
	bool UpdateDerivatives(double *u_ptr, double *dx_ptr, double *x_ptr, double *fc_ptr, double *p_ptr, double *c_ptr, double *a_ptr);
	bool UpdateStates(double *u_ptr, double *dx_ptr, double *x_ptr, double *fc_ptr, double *p_ptr, double *c_ptr, double *a_ptr);

	
private:
	
	FGPropagate *propagate;
	FGAuxiliary *auxiliary;
	vector<string> catalog;
	FGInitialCondition *ic;
	FGAerodynamics *aerodynamics;
	FGPropulsion *propulsion;
	FGFCS *fcs;

	
	bool _ac_model_loaded;
	double dT;
	double _u, _v, _w;
	double _p, _q, _r;
	double _alpha, _beta;
	double _phi, _theta, _psi, _gamma;

	double _stheta, _sphi, _spsi;
	double _ctheta, _cphi, _cpsi;

	double _udot, _vdot, _wdot, _pdot, _qdot, _rdot;
	double _q1dot, _q2dot, _q3dot, _q4dot;
	double _xdot, _ydot, _zdot;
	double _phidot, _thetadot, _psidot;
	double _alphadot,_betadot,_hdot;
	double	x_times;
	

};
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#endif
