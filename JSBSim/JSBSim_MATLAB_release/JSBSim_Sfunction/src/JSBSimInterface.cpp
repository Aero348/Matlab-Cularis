#include "StdAfx.h"
#include "JSBSimInterface.h"
#include <models/FGAircraft.h>
#include <FGState.h>
#include <math/FGQuaternion.h>

JSBSimInterface::JSBSimInterface(FGFDMExec *fdmex, double dt)
{
	mexPrintf("\n\tJSBSimInterface is loading\n");
	_ac_model_loaded = false;
	fdmExec = fdmex;
	fdmExec->GetState()->Setdt(dt);
	mexPrintf("\tSimulation dt set to %f\n",fdmExec->GetState()->Getdt());
	propagate = fdmExec->GetPropagate();
	auxiliary = fdmExec->GetAuxiliary();
	aerodynamics = fdmExec->GetAerodynamics();
	propulsion = fdmExec->GetPropulsion();
	fcs = fdmExec->GetFCS();
	ic = new FGInitialCondition(fdmExec);
	verbosityLevel = eSilent;
	catalog = fdmExec->GetPropertyCatalog();

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
JSBSimInterface::~JSBSimInterface(void)
{
	fdmExec = 0L;
	delete ic;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::Open(string prop)
{
	if (!fdmExec) return 0;

	/*char buf[128];
	mwSize buflen;
	buflen = mxGetNumberOfElements(prhs) + 1;
	mxGetString(prhs, buf, buflen);
	*/
	string acName = prop;
    string rootDir = "JSBSim/data/";

	//mexEvalString("plot(sin(0:.1:pi))");

	if ( fdmExec->GetAircraft()->GetAircraftName() != ""  )
	{
		if ( verbosityLevel == eVerbose )
		{
			mexPrintf("\n\tERROR: another aircraft is already loaded ('%s').\n", fdmExec->GetAircraft()->GetAircraftName().c_str());
			mexPrintf("\n\tTo load a new aircraft, clear the mex file and start up again.\n");
		}
		return 0;
	}

	// JSBSim stuff

	if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tSetting up JSBSim with standard 'aircraft', 'engine', and 'system' paths.\n");

    fdmExec->SetAircraftPath (rootDir + "aircraft");
    fdmExec->SetEnginePath   (rootDir + "engine"  );
    fdmExec->SetSystemsPath  (rootDir + "systems" );

	if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tLoading aircraft '%s' ...\n",acName.c_str());

    if ( ! fdmExec->LoadModel( rootDir + "aircraft",
                               rootDir + "engine",
                               rootDir + "systems",
                               acName)) 
	{
		if ( verbosityLevel == eVerbose )
			mexPrintf("\n\tERROR: JSBSim could not load the aircraft model.\n");
		return 0;
    }
	_ac_model_loaded = true;
	// Print AC name
	if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tModel %s loaded.\n", fdmExec->GetModelName().c_str() );

//***********************************************************************
	// populate aircraft catalog
	   fdmExec->PrintPropertyCatalog();

	if ( verbosityLevel == eVeryVerbose )
	{
		mexPrintf("\n\tAttempting to print AC property catalog.\n");
		PrintCatalog();
		//for (unsigned i=0; i<catalog.size(); i++) 
			//mexPrintf("%s\n",catalog[i].c_str());
	}
//***********************************************************************/
	//
	
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::GetPropertyValue(const mxArray *prhs1, double& value)
{
	if (!fdmExec) return 0;
	//if (!IsAircraftLoaded()) return 0;

	char buf[128];
	mwSize buflen;
	buflen = mxGetNumberOfElements(prhs1) + 1;
	mxGetString(prhs1, buf, buflen);
	string prop = "";
	prop = string(buf);
	if ( !QueryJSBSimProperty(prop) )
	{
		if ( verbosityLevel == eVerbose )
			mexPrintf("\n\tERROR: JSBSim could not find the property '%s' in the aircraft catalog.\n",prop.c_str());
		return 0;
	}
	value = fdmExec->GetPropertyValue(prop);
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::SetPropertyValue(const mxArray *prhs1, const mxArray *prhs2)
{
	if (!fdmExec) return 0;
	//if (!IsAircraftLoaded()) return 0;

	char buf[128];
	mwSize buflen;
	buflen = mxGetNumberOfElements(prhs1) + 1;
	mxGetString(prhs1, buf, buflen);
	string prop = "";
	prop = string(buf);
	double value = *mxGetPr(prhs2);

	if (!EasySetValue(prop,value)) // first check if an easy way of setting is implemented
	{
		if ( !QueryJSBSimProperty(prop) ) // then try to set the full-path property, e.g. '/fcs/elevator-cmd-norm'
		{
			if ( verbosityLevel == eDebug )
				mexPrintf("\n\tERROR: JSBSim could not find the property '%s' in the aircraft catalog.\n",prop.c_str());
			return 1;
		}
		fdmExec->SetPropertyValue(prop, value);
	}
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::SetPropertyValue(const string prop, const double value)
{
		mxArray *p1 = mxCreateString(prop.c_str());
		mxArray *p2 = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(p2) = value;
		return SetPropertyValue(p1,p2);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::EasySetValue(const string prop,  double value)
{
	if (prop == "u-fps")
	{
		propagate->SetUVW(1,value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: true flight speed (ft/s) = %f\n",auxiliary->GetVt());
		
		return 1;
	}
	else if (prop == "v-fps")
	{
		propagate->SetUVW(2,value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: v (ft/s) = %f\n",auxiliary->GetAeroUVW(2));
		return 1;
	}
	else if (prop == "w-fps")
	{
		propagate->SetUVW(3,value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: w (ft/s) = %f\n",auxiliary->GetAeroUVW(3));
		return 1;
	}
	else if (prop == "p-rad_sec")
	{
		propagate->SetPQR(1,value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: roll rate (rad/s) = %f\n",propagate->GetPQR(1));
		return 1;
	}
	else if (prop == "q-rad_sec")
	{
		propagate->SetPQR(2,value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: pitch rate (rad/s) = %f\n",propagate->GetPQR(2));
		return 1;
	}
	else if (prop == "r-rad_sec")
	{
		propagate->SetPQR(3,value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: yaw rate (rad/s) = %f\n",propagate->GetPQR(3));
		return 1;
	}
	else if (prop == "h-sl-ft")
	{
		propagate->SetAltitudeASL(value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: altitude over sea level (ft) = %f\n",propagate->GetAltitudeASL());
		return 1;
	}
	else if (prop == "long-gc-deg")
	{
		propagate->SetLongitudeDeg(value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: geocentric longitude (deg) = %f\n",propagate->GetLongitudeDeg());
		return 1;
	}
	else if (prop == "lat-gc-deg")
	{
		propagate->SetLatitudeDeg(value);
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: geocentric latitude (deg) = %f\n",propagate->GetLatitudeDeg());
		return 1;
	}
	else if (prop == "phi-rad")
	{
		FGQuaternion Quat( value, propagate->GetEuler(2), propagate->GetEuler(3) );
		/*FGQuaternion Quat( value, .2, propagate->GetEuler(3) );*/
		Quat.Normalize();
		FGPropagate::VehicleState* vstate = propagate->GetVState(); //changed to pointer in FGPropagate
		vstate->vQtrn = Quat; // reflect changes made to FGPropagate
		propagate->SetVState(vstate);
		propagate->Run(); // vVel => gamma
		auxiliary->Run(); // alpha, beta, gamma
		if ( verbosityLevel == eVerbose ) 
			mexPrintf("\tEasy-set: phi -> quaternion = (%f,%f,%f,%f)\n",
				propagate->GetVState()->vQtrn(1),propagate->GetVState()->vQtrn(2),propagate->GetVState()->vQtrn(3),propagate->GetVState()->vQtrn(4)),
		
		mexPrintf("\tEasy-set: alpha (deg) = %f,\n\tbeta (deg) = %f,\n\tgamma (deg) = %f\n",
			auxiliary->Getalpha()*180./M_PI,auxiliary->Getbeta()*180./M_PI,auxiliary->GetGamma()*180./M_PI);
		
		return 1;
	}
	else if (prop == "theta-rad")
	{
		FGQuaternion Quat( propagate->GetEuler(1), value, propagate->GetEuler(3) );
		Quat.Normalize();
		FGPropagate::VehicleState* vstate = propagate->GetVState();
		vstate->vQtrn = Quat;
		propagate->SetVState(vstate);
		propagate->Run(); // vVel => gamma
		auxiliary->Run(); // alpha, beta, gamma
		if ( verbosityLevel == eVerbose ) 
			mexPrintf("\tEasy-set: theta -> quaternion = (%f,%f,%f,%f)\n",
				propagate->GetVState()->vQtrn(1),propagate->GetVState()->vQtrn(2),propagate->GetVState()->vQtrn(3),propagate->GetVState()->vQtrn(4)),
		
		mexPrintf("\tEasy-set: alpha (deg) = %f,\n\tbeta (deg) = %f,\n\tgamma (deg) = %f\n",
			auxiliary->Getalpha()*180./M_PI,auxiliary->Getbeta()*180./M_PI,auxiliary->GetGamma()*180./M_PI);
		
		return 1;
	}
	else if (prop == "psi-rad")
	{
		FGQuaternion Quat( propagate->GetEuler(1), propagate->GetEuler(2), value );
		Quat.Normalize();
		FGPropagate::VehicleState* vstate = propagate->GetVState();
		vstate->vQtrn = Quat;
		propagate->SetVState(vstate);
		propagate->Run(); // vVel => gamma
		auxiliary->Run(); // alpha, beta, gamma
		if ( verbosityLevel == eVerbose )  
			mexPrintf("\tEasy-set: psi -> quaternion = (%f,%f,%f,%f)\n",
				propagate->GetVState()->vQtrn(1),propagate->GetVState()->vQtrn(2),propagate->GetVState()->vQtrn(3),propagate->GetVState()->vQtrn(4)),
		
		mexPrintf("\tEasy-set: alpha (deg) = %f,\n\tbeta (deg) = %f,\n\tgamma (deg) = %f\n",
			auxiliary->Getalpha()*180./M_PI,auxiliary->Getbeta()*180./M_PI,auxiliary->GetGamma()*180./M_PI);
		
		return 1;
	}
		
	else if (prop == "fcs/throttle-cmd-norm")
	{
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++){
			fdmExec->GetFCS()->SetThrottleCmd(i, value);//control the throttle cmd
			fdmExec->GetFCS()->SetThrottlePos(i, value);//control the throttle position
		}
		//ensure that the engines have spooled up completely
		for(unsigned i=0;i<2000;i++){
			fdmExec->GetFCS()->Run();
			propagate->Run();
			auxiliary->Run();
		}
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: throttle pos norm (for all throttles) = %f\n",fdmExec->GetFCS()->GetThrottlePos(0));
		return 1;
	}
	
	else if (prop == "elevator-cmd-norm")
	{
		fdmExec->GetFCS()->SetDeCmd(value);
		fdmExec->GetFCS()->SetDePos(2, value);
		fdmExec->GetFCS()->Run();
		propagate->Run();
		auxiliary->Run();
		
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: elevator pos rads = %f\n",fdmExec->GetFCS()->GetDePos());
		return 1;
	}
	else if (prop == "aileron-cmd-norm")
	{
		fdmExec->GetFCS()->SetDaCmd(value);
		fdmExec->GetFCS()->SetDaLPos(2, value);
		int neg_value = -value;
		fdmExec->GetFCS()->SetDaRPos(2, neg_value);
		fdmExec->GetFCS()->Run();
		propagate->Run();
		auxiliary->Run();
		
		if ( verbosityLevel == eVerbose ){
			mexPrintf("\tEasy-set: left aileron pos rads = %f\n",fdmExec->GetFCS()->GetDaLPos());
		    mexPrintf("\tEasy-set: right aileron pos rads = %f\n",fdmExec->GetFCS()->GetDaRPos());
		}
		return 1;
	}
	else if (prop == "rudder-cmd-norm")
	{
		fdmExec->GetFCS()->SetDrCmd(value);
		fdmExec->GetFCS()->SetDrPos(2, value);
		fdmExec->GetFCS()->Run();
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: rudder pos rads = %f\n",fdmExec->GetFCS()->GetDrPos());
		return 1;
	}
	
	else if (prop == "fcs/mixture-cmd-norm")
	{
		
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++){
			fdmExec->GetFCS()-> SetMixtureCmd(i, value);//control the mixture cmd
			//fdmExec->GetFCS()->SetMixturePos(i, value);//control the mixture position
		}
		
		//fdmExec->GetFCS()->SetMixtureCmd(0, value);//control the mixture cmd
		//if ( verbosityLevel == eVerbose )
			//mexPrintf("\tEasy-set: mixture pos norm (for all engines) = %f\n",fdmExec->GetFCS()->GetMixturePos(0));
		return 1;
	}
	
	else if (prop == "set-running")
	{
		//mexPrintf("\tEasy-set: Set Running Called\n");
		bool isrunning = false;
		double engine_type;
		if (value > 0) isrunning = true;
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++)
		{
			fdmExec->GetPropulsion()->GetEngine(i)->SetRunning(isrunning);
			//fdmExec->GetPropulsion()->GetEngine(i)->SetRPM()
			engine_type = fdmExec->GetPropulsion()->GetEngine(i)->GetType();
			if(engine_type == 2){
				fdmExec->GetPropulsion()->GetEngine(i)->InitRunning();
			}
			fdmExec->Run();
			fdmExec->GetPropulsion()->GetSteadyState();
		}
		//fdmExec->GetState()->Setdt(0.00833);
		//fdmExec->GetPropulsion()->GetSteadyState();
		//propagate->Run();
		//auxiliary->Run();
		/*
		int engineStartCount = 0;
		bool engine_started = false;
		//while ( (Propulsion->GetActiveEngine() < 0) && (engineStartCount < 100) )
		while ( !engine_started && (engineStartCount < 10) )
			{
				//cout << "\t\t\t\tattempt: " << engineStartCount << endl;
				//cout << "\t\t\t\tactive engine:" << Propulsion->GetActiveEngine() << endl;
				//fdmExec->GetPropulsion()->GetEngine(1)->InitRunning();
				engine_started = fdmExec->GetPropulsion()->GetEngine(0)->GetRunning();
				fdmExec->Run();
				//Propulsion->Run();
				engineStartCount++;
			}
		*/
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: %d engine(s) running = %d\n",fdmExec->GetPropulsion()->GetNumEngines(),(int)isrunning);
		return 1;
	}
	else if (prop == "flaps-cmd-norm")
	{
		fdmExec->GetFCS()->SetDfCmd(value);
		fdmExec->GetFCS()->SetDfPos(2, value);
		fdmExec->GetFCS()->Run();
		propagate->Run();
		auxiliary->Run();
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: Flap pos norm = %f\n",fdmExec->GetFCS()->GetDfPos(2));
		return 1;
	}
	else if (prop == "gear-cmd-norm")
	{
		fdmExec->GetFCS()->SetGearPos(value);
		fdmExec->GetFCS()->Run();
		propagate->Run();
		auxiliary->Run();
		//ensure that the engines have spooled up completely
		
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tEasy-set: Gear pos norm = %f\n",fdmExec->GetFCS()->GetGearPos());
		return 1;
	}
	//Created a JSBSim "internal" property "multiplier" 
	else if (prop == "multiplier")
	{
		SetMultiplier(value);
		//fdmExec->GetFCS()->Run();
		//propagate->Run();
		//auxiliary->Run();
		if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tJSBSim running at %f times the speed of Simulink\n",GetMultiplier());

		return 1;
	}

	else if (prop == "rate_rot")
	{
		fdmExec->SetPropertyValue("simulation/integrator/rate/rotational", value);
		if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tJSBSim integrator rotational rate = %f\n",fdmExec->GetPropertyValue("simulation/integrator/rate/rotational"));
		return 1;
	}
	else if (prop == "rate_trans")
	{
		fdmExec->SetPropertyValue("simulation/integrator/rate/translational", value);
		if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tJSBSim integrator translational rate = %f\n",fdmExec->GetPropertyValue("simulation/integrator/rate/translational"));
		return 1;
	}
	else if (prop == "pos_rot")
	{
		fdmExec->SetPropertyValue("simulation/integrator/position/rotational",value);
		if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tJSBSim integrator rotational position = %f\n",fdmExec->GetPropertyValue("simulation/integrator/position/rotational"));
		return 1;
	}
	else if (prop == "pos_trans")
	{
		fdmExec->SetPropertyValue("simulation/integrator/position/translational", value);
		if ( verbosityLevel == eVerbose )
		mexPrintf("\n\tJSBSim integrator translational position = %f\n",fdmExec->GetPropertyValue("simulation/integrator/position/translational"));
		return 1;
	}
	return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::QueryJSBSimProperty(string prop)
{
	// mexPrintf("catalog size: %d\n",catalog.size());
	for (unsigned i=0; i<catalog.size(); i++)
	{
		// mexPrintf("__%s__\n",catalog[i].c_str());
		if (catalog[i]==prop) return 1;
	}
	return 0;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void JSBSimInterface::PrintCatalog()
{
	catalog = fdmExec->GetPropertyCatalog();
	
		mexPrintf("-- Property catalog for current aircraft %s:\n",fdmExec->GetModelName().c_str());
		for (unsigned i=0; i<catalog.size(); i++)
			mexPrintf("\t%s\n",catalog[i].c_str());
		mexPrintf("-- end of catalog\n");

	return;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::ResetToInitialCondition()
{
	//mexEvalString("clearSF");
	fdmExec->GetState()->Setsim_time(0.0);	
	//fdmExec->GetIC()->ResetIC(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	//fdmExec->GetPropagate()->ResetPQRdot();
	//fdmExec->GetPropagate()->ResetUVWdot();
	//fdmExec->GetPropagate()->ResetQtrndot();
	//fdmExec->GetPropagate()->ResetLocationdot();
	//fdmExec->GetPropagate()->InitModel();
	fdmExec->ResetToInitialConditions();
	mexPrintf("\n\tAircraft states are reset to IC\n");
	return 1;
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::Init(const mxArray *prhs1)
{
	// Inspired by "refbook.c"
	// The argument prhs1 is pointer to a Matlab structure with two fields: name, value.
	// The Matlab user is forced to build such a structure first, then to pass it to the
	// mex-file. Example:
	//    >> ic(1).name = 'u'; ic(1).value = 80; (ft/s)
	//    >> ic(2).name = 'v'; ic(1).value =  0; (ft/s)
	//    >> ic(3).name = 'w'; ic(1).value =  0; (ft/s)
	//    >> ic(4).name = 'p'; ic(1).value =  0; (rad/s) % etc ...
	//    >> MexJSBSim('init', ic)

	//*************************************************
	// Set dt=0 first
	
	fdmExec->GetState()->SuspendIntegration();
	
	//*************************************************

	bool success = 1;

	const char **fnames;       /* pointers to field names */
	const mwSize *dims;
	mxArray    *tmp;
	char       *pdata=NULL;
	int        ifield, nfields;
	mxClassID  *classIDflags;
	mwIndex    jstruct;
	mwSize     NStructElems;
	mwSize     ndim;

    // get input arguments
    nfields = mxGetNumberOfFields(prhs1);
    NStructElems = mxGetNumberOfElements(prhs1);
    // allocate memory  for storing classIDflags
    classIDflags = (mxClassID*)mxCalloc(nfields, sizeof(mxClassID));

    // check empty field, proper data type, and data type consistency;
	// and get classID for each field (see "refbook.c")
    for(ifield=0; ifield<nfields; ifield++) 
	{
		for(jstruct = 0; jstruct < NStructElems; jstruct++) 
		{
			tmp = mxGetFieldByNumber(prhs1, jstruct, ifield);
			if(tmp == NULL) 
			{
				if ( verbosityLevel == eVeryVerbose )
				{
					mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
					mexErrMsgTxt("Above field is empty!");
				}
				return 0;
			} 
			if(jstruct==0) 
			{
				if( (!mxIsChar(tmp) && !mxIsNumeric(tmp)) || mxIsSparse(tmp)) 
				{
					if ( verbosityLevel == eVeryVerbose )
					{
						mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
						mexErrMsgTxt("Above field must have either string or numeric non-sparse data.");
					}
					return 0;
				}
				classIDflags[ifield]=mxGetClassID(tmp); 
			} 
			else 
			{
				if (mxGetClassID(tmp) != classIDflags[ifield]) 
				{
					if ( verbosityLevel == eVeryVerbose )
					{
						mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
						mexErrMsgTxt("Inconsistent data type in above field!"); 
					}
					return 0;
				} 
				else if(!mxIsChar(tmp) && 
					  ((mxIsComplex(tmp) || mxGetNumberOfElements(tmp)!=1)))
				{
					if ( verbosityLevel == eVeryVerbose )
					{
						mexPrintf("%s%d\t%s%d\n", "FIELD: ", ifield+1, "STRUCT INDEX :", jstruct+1);
						mexErrMsgTxt("Numeric data in above field must be scalar and noncomplex!"); 
					}
					return 0;
				}
			}
		}
    }
    /* allocate memory  for storing pointers */
    fnames = (const char **)mxCalloc(nfields, sizeof(*fnames));
    /* get field name pointers */
    for (ifield=0; ifield< nfields; ifield++)
	{
		fnames[ifield] = mxGetFieldNameByNumber(prhs1,ifield);
    }
	// At this point we have extracted from prhs1 the vector of 
	// field names fnames of nfields elements.
	// nfields is the number of fields in the passed Matlab struct (ic).
	// It may have more fields, but the first two must be "name" and "value"
	// The structure possesses generally a number of NStructElems elements.

    ndim = mxGetNumberOfDimensions(prhs1);
    dims = mxGetDimensions(prhs1);

	// loop on the element of the structure
	for (jstruct=0; jstruct<NStructElems; jstruct++) 
	{
		string prop = "";
		double value = -99.;

		// scan the fields
		// the first two must be "name" and "value"
		for(ifield=0; ifield<2; ifield++) // for(ifield=0; ifield<nfields; ifield++) // nfields=>2
		{
			tmp = mxGetFieldByNumber(prhs1,jstruct,ifield);
			if( mxIsChar(tmp) ) //  && (fnames[ifield]=="name") the "name" field
			{
				// mxSetCell(fout, jstruct, mxDuplicateArray(tmp));
				char buf[128];
				mwSize buflen;
				buflen = mxGetNumberOfElements(tmp) + 1;
				mxGetString(tmp, buf, buflen);
				prop = string(buf);
				//mexPrintf("field name: %s\n",prop.c_str());
			}
			else  // the "value" field
			{
				value = *mxGetPr(tmp);
				//mexPrintf("field value %f\n",value);
			}
		}
		//----------------------------------------------------
		// now we have a string in prop and a double in value
		// we got to set the property value accordingly
		//----------------------------------------------------
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\n\tProperty name: '%s'; to be set to value: %f\n",prop.c_str(),value);

		//----------------------------------------------------
		// Note: the time step is set to zero at this point, so that all calls 
		//       to propagate->Run() will not advance the vehicle state in time
		//----------------------------------------------------
		// Now pass prop and value to the member function
		success = success && SetPropertyValue(prop,value); // EasySet called here
		if ( verbosityLevel == eVeryVerbose )
			mexPrintf("\n\tsuccess '%d'; \n",(int)success);
    }
	// free memory
    mxFree(classIDflags);
	mxFree((void *)fnames);

	//---------------------------------------------------------------
	// see "FGInitialConditions.h"
	// NOTE:
	
	
	double vt = 
		sqrt(
			fdmExec->GetPropagate()->GetUVW(1) * fdmExec->GetPropagate()->GetUVW(1) + // to do State ??
			fdmExec->GetPropagate()->GetUVW(2) * fdmExec->GetPropagate()->GetUVW(2) +
			fdmExec->GetPropagate()->GetUVW(3) * fdmExec->GetPropagate()->GetUVW(3) );

	//if ( verbosityLevel == eVerbose || verbosityLevel == eDebug)
		//mexPrintf("Vt = %f\n",fdmExec->GetAuxiliary()->GetVt());

	fdmExec->Run();
	fdmExec->GetState()->ResumeIntegration();
	/*
	// Calculate state derivatives
	fdmExec->GetPropagate()->CalculatePQRdot();      // Angular rate derivative
	fdmExec->GetPropagate()->CalculateUVWdot();      // Translational rate derivative
	fdmExec->GetPropagate()->CalculateQuatdot();     // Angular orientation derivative
	fdmExec->GetPropagate()->CalculateLocationdot(); // Translational position derivative
	*/
	_udot = propagate->GetUVWdot(1);
	_vdot = propagate->GetUVWdot(2);
	_wdot = propagate->GetUVWdot(3);
	_pdot = propagate->GetPQRdot(1);
	_qdot = propagate->GetPQRdot(2);
	_rdot = propagate->GetPQRdot(3);

	if ( verbosityLevel == eVerbose || verbosityLevel == eDebug){
		//fdmExec->GetState()->Setdt(1.0/120);
		mexPrintf("\n\tInitial State derivatives calculated at:\n");
		mexPrintf("\tSimulation dt %f\n",fdmExec->GetState()->Getdt());
		mexPrintf("\tSimulation sim-time %f\n",fdmExec->GetSimTime());	
		mexPrintf("\tVt = %f (ft/s)\n",fdmExec->GetAuxiliary()->GetVt());
		mexPrintf("\t[u_dot, v_dot, w_dot] = [%f, %f, %f] (ft/s/s)\n",
			propagate->GetUVWdot(1),propagate->GetUVWdot(2),propagate->GetUVWdot(3));
		mexPrintf("\t[p_dot, q_dot, r_dot] = [%f, %f, %f] (rad/s/s)\n",
			propagate->GetPQRdot(1),propagate->GetPQRdot(2),propagate->GetPQRdot(3));
	}
	
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tSimulation dt set to %f\n",fdmExec->GetState()->Getdt());
		
		if ( verbosityLevel == eVerbose ){
		//JSBSim generated states prior to integration
		mexPrintf("\tInitial State variables calculated.\n");
		mexPrintf("\tU %f (ft/s)\n",fdmExec->GetPropagate()->GetUVW(1));
		mexPrintf("\tV %f (ft/s)\n",fdmExec->GetPropagate()->GetUVW(2));
		mexPrintf("\tW %f (ft/s)\n",fdmExec->GetPropagate()->GetUVW(3));
		mexPrintf("\tP %f (rad/s)\n",fdmExec->GetPropagate()->GetPQR(1));
		mexPrintf("\tQ %f (rad/s)\n",fdmExec->GetPropagate()->GetPQR(2));
		mexPrintf("\tR %f (rad/s)\n",fdmExec->GetPropagate()->GetPQR(3));
		mexPrintf("\tH %f (ft)\n",fdmExec->GetPropagate()->GetAltitudeASL());
		mexPrintf("\tPhi %f (rad)\n",fdmExec->GetPropagate()->GetEuler(1));
		mexPrintf("\tTheta %f (rad)\n",fdmExec->GetPropagate()->GetEuler(2));
		mexPrintf("\tPsi %f (rad)\n",fdmExec->GetPropagate()->GetEuler(3));
		}
		
	if (!success)
	{
		if ( verbosityLevel == eVerbose )
			mexPrintf("\n\tERROR: One or more or all the required properties could not be initialized.\n");
		return 0;
	}
	else
		return 1;
}
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::GetEngNum(real_T *num_of_eng)
{

	mexPrintf("\JSBSim: engine(s) running = %i\n",fdmExec->GetPropulsion()->GetNumEngines());
	
	return 1;


	if (!Init(prhs1))
	{
		if ( verbosityLevel == eVerbose )
			mexPrintf("\tERROR: could not calculate dotted states correctly.\n");
		return 0;
	}
	statedot[ 0] = _udot;
	statedot[ 1] = _vdot;
	statedot[ 2] = _wdot;
	statedot[ 3] = _pdot;
	statedot[ 4] = _qdot;
	statedot[ 5] = _rdot;
	statedot[ 6] = _q1dot;
	statedot[ 7] = _q2dot;
	statedot[ 8] = _q3dot;
	statedot[ 9] = _q4dot;
	statedot[10] = _xdot;
	statedot[11] = _ydot;
	statedot[12] = _zdot;
	statedot[13] = _phidot;
	statedot[14] = _thetadot;
	statedot[15] = _psidot;
	statedot[16] = _hdot;
	statedot[17] = _alphadot;
	statedot[18] = _betadot;

	return 1;
	
}


bool JSBSimInterface::GetEngType(string* engtype)
{
	//string engine = (string) fdmExec->GetPropulsion()->GetEngine(0)->GetType();
	return 1;
}
*/
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::SetVerbosity(const mxArray *prhs1)
{
	int field; // field index (0)
	mwIndex    jstruct;	// index (0)
	mxArray *tmp; // mxArray pointer

	//mexPrintf("SetVerbosity called.\n");

	if (!fdmExec) return 0;
	// if (!IsAircraftLoaded()) return 0;
	// even when aircraft is null we must be able to set verbosity level
	jstruct = 0;
	field = 0;

	tmp = mxGetFieldByNumber(prhs1,jstruct,field); //point to field number 0

	if ( mxIsChar(tmp) )
	{
		char buf[128];
		mwSize buflen;
		buflen = mxGetNumberOfElements(tmp) + 1;
		mxGetString(tmp, buf, buflen);
		string prop = "";
		prop = string(buf);
		//mexPrintf("Verbosity level set.\n");
		if ( (prop == "silent") ||
			 (prop == "Silent") ||
			 (prop == "SILENT")
			)
		{
			SetVerbosity( (JIVerbosityLevel)0 );
		}
		else if ( (prop == "verbose") ||
			      (prop == "Verbose") ||
			      (prop == "VERBOSE")
			)
		{
			SetVerbosity( (JIVerbosityLevel)1 );
		}
		else if ( (prop == "very verbose") ||
			      (prop == "Very Verbose") ||
			      (prop == "VERY VERBOSE")
			)
		{
			SetVerbosity( (JIVerbosityLevel)2 );
		}
		else if ( (prop == "debug") ||
			      (prop == "Debug") ||
			      (prop == "DEBUG")
			)
		{
			SetVerbosity( (JIVerbosityLevel)3 );
		}
		else 
			return 0;
	}
	else if ( mxIsNumeric(prhs1) )
	{
		double value = 0;
		value = *mxGetPr(prhs1);
		//mexPrintf("%f Verbosity level set in JII.\n", value);
		int ival = (int) value;
		if (ival <= eVeryVerbose)
		{
			SetVerbosity( (JIVerbosityLevel)ival );
		}
		else
			return 0;
	}
	
	return 1;
}

/* 
double JSBSimInterface::GetEulerDot(int i)
{
	double angle;
	angle = fdmExec->GetPropagate()->GetQtrndot(i);// Angle in radians to be converted to euler!!!
		return angle;
}
*/
bool JSBSimInterface::SetEuler(int i, double value)
{
	if ((i>=1) && (i<=3) ){
		  if(i == 1){
			FGQuaternion Quat( value, propagate->GetEuler(2), propagate->GetEuler(3) );
			/*FGQuaternion Quat( value, .2, propagate->GetEuler(3) );*/
			Quat.Normalize();
			FGPropagate::VehicleState* vstate = propagate->GetVState(); //changed to pointer in FGPropagate
			vstate->vQtrn = Quat; // reflect changes made to FGPropagate
			propagate->SetVState(vstate);
			return 1;
		  }
		  if(i == 2){
			FGQuaternion Quat( propagate->GetEuler(1), value, propagate->GetEuler(3) );
			Quat.Normalize();
			FGPropagate::VehicleState* vstate = propagate->GetVState();
			vstate->vQtrn = Quat;
			propagate->SetVState(vstate);
			return 1;
		  }
		  if(i == 3){
			FGQuaternion Quat( propagate->GetEuler(1), propagate->GetEuler(2), value );
			Quat.Normalize();
			FGPropagate::VehicleState* vstate = propagate->GetVState();
			vstate->vQtrn = Quat;
			propagate->SetVState(vstate);
			return 1;
		  }
	  }
	return 0;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// UpdateStates Method for SFunction
bool JSBSimInterface::UpdateStates(double *u_ptr, double *dx_ptr, double *x_ptr, double *fc_ptr, double *p_ptr, double *c_ptr, double *a_ptr)
{
	//mexPrintf("\t Simulation dt %f\n",fdmExec->GetState()->Getdt());
	//mexPrintf("Simulation sim-time %f\n",fdmExec->GetSimTime());
	int engines = (int) fdmExec->GetPropulsion()->GetNumEngines();
	int eng_type = fdmExec->GetPropulsion()->GetEngine(0)->GetType();
	
	/* Receive updated control inputs from MexJSBSimSFun, propagate them through one JSBSim cycle
	 * and retrieve updated states and outputs, and return them to MexJSBSimSFunction.
	 */

	   /* New control inputs from S-Function 
		* Control Input Vector = [throttle aileron elevator rudder mixture set-run flaps gear]
		*/
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++)
			fdmExec->GetFCS()->SetThrottleCmd(i, u_ptr[0]);//control the throttle(s)
		fdmExec->GetFCS()->SetDaCmd(u_ptr[1]);//control the ailerons
		fdmExec->GetFCS()->SetDeCmd(u_ptr[2]);//control the elevators
		fdmExec->GetFCS()->SetDrCmd(u_ptr[3]);//control the rudder(s)
		SetPropertyValue("fcs/mixture-cmd-norm",u_ptr[4]);//control the mixture
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++)
			fdmExec->GetPropulsion()->GetEngine(i)->SetRunning(u_ptr[5]);//set engine(s) to running
		fdmExec->GetFCS()->SetDfCmd(u_ptr[6]);//control the flaps
		fdmExec->GetFCS()->SetGearCmd(u_ptr[7]);//control the gear position

		

		//Run JSBSim x times
		for(int i = 0;i < GetMultiplier();i++){
			fdmExec->Run();
			if ( verbosityLevel == eDebug ){
				mexPrintf("\tCall to Run completed\n");
			}
		}
		
		/* Send newly calculated states to S-Function
		 * State vector will be:
		 * [u-fps v-fps w-fps p-radsec q-radsec r-radsec q1-rad q2-rad q3-rad q4-rad long-deg 
		 *  lat-deg z-ft phi-rad theta-rad psi-rad h-sl-ft alpha-rad beta-rad]
		 */
		x_ptr[0] = fdmExec->GetPropagate()->GetUVW(1); // U fps
		x_ptr[1] = fdmExec->GetPropagate()->GetUVW(2);// V fps
		x_ptr[2] = fdmExec->GetPropagate()->GetUVW(3);// W fps
		x_ptr[3] = fdmExec->GetPropagate()->GetPQR(1);// Roll rate rad/sec
		x_ptr[4] = fdmExec->GetPropagate()->GetPQR(2);//Pitch rate rad/sec
		x_ptr[5] = fdmExec->GetPropagate()->GetPQR(3);//Yaw rate rad/sec
		/*
		fdmExec->GetPropagate()->GetQuaternion();
		x_ptr[6] = propagate->GetVState()->vQtrn(1);// Quaternion 1
		x_ptr[7] = propagate->GetVState()->vQtrn(2);// Quaternion 2
		x_ptr[8] = propagate->GetVState()->vQtrn(3);// Quaternion 3
		x_ptr[9] = propagate->GetVState()->vQtrn(4);// Quaternion 4
		x_ptr[10] = fdmExec->GetPropagate()->GetLongitudeDeg();// Longitude geographic position in deg
		x_ptr[11] = fdmExec->GetPropagate()->GetLatitudeDeg();// Latitude geographic position in deg
		x_ptr[12] = fdmExec->GetPropagate()->GetGeodeticAltitude();// Z geographic position in feet
		*/
		x_ptr[6] = fdmExec->GetPropagate()->GetAltitudeASL();//Altitude in feet
		x_ptr[7] = fdmExec->GetPropagate()->GetLongitudeDeg();// Longitude geographic position in deg
		x_ptr[8] = fdmExec->GetPropagate()->GetLatitudeDeg();// Latitude geographic position in deg
		
		x_ptr[9] = fdmExec->GetPropagate()->GetEuler(1);// Phi (Roll Angle)in radians
		x_ptr[10] = fdmExec->GetPropagate()->GetEuler(2);// Theta (Pitch Angle) in radians
		x_ptr[11] = fdmExec->GetPropagate()->GetEuler(3);// Psi (Yaw Angle) in radians
		/*
		x_ptr[16] = fdmExec->GetPropagate()->GetAltitudeASL();//Altitude in feet
		x_ptr[17] = fdmExec->GetAuxiliary()->Getalpha();// Alpha in radians
		x_ptr[18] = fdmExec->GetAuxiliary()->Getbeta();// Beta in radians
		*/
		/* Flight Controls output vector [throttle left-aileron elevator tvc rudder flap right-aileron speedbrake
		 * spoiler lef gear nosewheel-steering gear-unit-WOW]
		 */
		fc_ptr[0] = fdmExec->GetFCS()->GetThrottlePos(0);//fcs/throttle-pos-norm 
		fc_ptr[1] = fdmExec->GetFCS()->GetDaLPos(0);//fcs/left-aileron-pos-rad 0=rad, 1=deg, 2=norm
		fc_ptr[2] = fdmExec->GetFCS()->GetDePos(0);//fcs/elevator-pos-rad
		fc_ptr[3] = fdmExec->GetPropertyValue("fcs/tvc-pos-rad");//tvc-pos-rad
		fc_ptr[4] = fdmExec->GetFCS()->GetDrPos(0);//fcs/rudder-pos-rad
		fc_ptr[5] = fdmExec->GetFCS()->GetDfPos(0);//fcs/flap-pos-rad
		fc_ptr[6] = fdmExec->GetFCS()->GetDaRPos(0);//fcs/right-aileron-pos-rad
		fc_ptr[7] = fdmExec->GetFCS()->GetDsbPos(0);//fcs/speedbrake-pos-rad
		fc_ptr[8] = fdmExec->GetFCS()->GetDspPos(0);//fcs/spoiler-pos-rad
		fc_ptr[9] = fdmExec->GetPropertyValue("fcs/lef-pos-rad");//lef-pos-rad
		fc_ptr[10] = fdmExec->GetFCS()->GetGearPos();//gear/gear-pos-norm
		fc_ptr[11] = fdmExec->GetFCS()->GetSteerPosDeg(0);//Nose-gear-steering-pos-deg
		fc_ptr[12] = (int)fdmExec->GetPropertyValue("gear/unit/WOW");//Gear-WOW


     switch(eng_type)
	 {
	 case(2)://Piston engines
		switch(engines)
		{
		  case(1):
	   /* Propulsion output vector for piston engined aircraft p_ptr [] This will be sized based on # of engines
		* There are 12 output properties for each engine [RPM thrust mixture fuel-flow advance-ratio engine-power
		* pt vol-eff bsfc torque blade-angle pitch]  Each engine will have these output properties, so for example, if
		* there are 3 engines, then there will be 36 total (3 x 12) outputs.
		*/

		//*********if piston, single engined************
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch
			break;

		//*********if piston, dual engined************
		  case(2):
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch
			

			p_ptr[12] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[13] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[14] = fdmExec->GetPropulsion()->GetEngine(1)->GetMixture();//mixture
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/advance-ratio");//advance-ratio
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/power-hp");//engine/egt-degf
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[21] = fdmExec->GetPropertyValue("propulsion/engine[1]/torque");//engine/thruster/torque
			p_ptr[22] = fdmExec->GetPropertyValue("propulsion/engine[1]/blade-angle");//engine/thruster/rpm
			p_ptr[23] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetPitch();//engine/thruster/pitch
			break;

		  case(3):
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch

			p_ptr[12] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[13] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[14] = fdmExec->GetPropulsion()->GetEngine(1)->GetMixture();//mixture
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/advance-ratio");//advance-ratio
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/power-hp");//engine/egt-degf
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[21] = fdmExec->GetPropertyValue("propulsion/engine[1]/torque");//engine/thruster/torque
			p_ptr[22] = fdmExec->GetPropertyValue("propulsion/engine[1]/blade-angle");//engine/thruster/rpm
			p_ptr[23] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetPitch();//engine/thruster/pitch

			p_ptr[24] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[25] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[26] = fdmExec->GetPropulsion()->GetEngine(2)->GetMixture();//mixture
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/advance-ratio");//advance-ratio
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/power-hp");//engine/egt-degf
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[33] = fdmExec->GetPropertyValue("propulsion/engine[2]/torque");//engine/thruster/torque
			p_ptr[34] = fdmExec->GetPropertyValue("propulsion/engine[2]/blade-angle");//engine/thruster/rpm
			p_ptr[35] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetPitch();//engine/thruster/pitch

			break;
		  case(4):
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch

			p_ptr[12] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[13] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[14] = fdmExec->GetPropulsion()->GetEngine(1)->GetMixture();//mixture
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/advance-ratio");//advance-ratio
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/power-hp");//engine/egt-degf
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[21] = fdmExec->GetPropertyValue("propulsion/engine[1]/torque");//engine/thruster/torque
			p_ptr[22] = fdmExec->GetPropertyValue("propulsion/engine[1]/blade-angle");//engine/thruster/rpm
			p_ptr[23] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetPitch();//engine/thruster/pitch

			p_ptr[24] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[25] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[26] = fdmExec->GetPropulsion()->GetEngine(2)->GetMixture();//mixture
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/advance-ratio");//advance-ratio
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/power-hp");//engine/egt-degf
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[33] = fdmExec->GetPropertyValue("propulsion/engine[2]/torque");//engine/thruster/torque
			p_ptr[34] = fdmExec->GetPropertyValue("propulsion/engine[2]/blade-angle");//engine/thruster/rpm
			p_ptr[35] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetPitch();//engine/thruster/pitch

			p_ptr[36] = fdmExec->GetPropulsion()->GetEngine(3)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[37] = fdmExec->GetPropulsion()->GetEngine(3)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[38] = fdmExec->GetPropulsion()->GetEngine(3)->GetMixture();//mixture
			p_ptr[39] = fdmExec->GetPropulsion()->GetEngine(3)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[40] = fdmExec->GetPropertyValue("propulsion/engine[3]/advance-ratio");//advance-ratio
			p_ptr[41] = fdmExec->GetPropertyValue("propulsion/engine[3]/power-hp");//engine/egt-degf
			p_ptr[42] = fdmExec->GetPropertyValue("propulsion/engine[3]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[43] = fdmExec->GetPropertyValue("propulsion/engine[3]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[44] = fdmExec->GetPropertyValue("propulsion/engine[3]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[45] = fdmExec->GetPropertyValue("propulsion/engine[3]/torque");//engine/thruster/torque
			p_ptr[46] = fdmExec->GetPropertyValue("propulsion/engine[3]/blade-angle");//engine/thruster/rpm
			p_ptr[47] = fdmExec->GetPropulsion()->GetEngine(3)->GetThruster()->GetPitch();//engine/thruster/pitch

			break;
		}
		break;

	   case(3)://Turbine engines
		 switch(engines)
		{
		  case(1):
	   /* Propulsion output vector p_ptr [] This will be sized based on # of engines
		* There are 12 output properties for each engine [RPM thrust mixture fuel-flow advance-ratio engine-power
		* pt vol-eff bsfc torque blade-angle pitch]  Each engine will have these output properties, so for example, if
		* there are 3 engines, then there will be 36 total (3 x 12) outputs.
		*/

			//*********if turbine, single engined************
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");
			break;

		//*********if turbine, dual engined************
		  case(2):
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");
			

			p_ptr[12] = fdmExec->GetPropertyValue("propulsion/engine[1]/thrust-lbs");//Thrust-lbs
			p_ptr[13] = fdmExec->GetPropertyValue("propulsion/engine[1]/n1");
			p_ptr[14] = fdmExec->GetPropertyValue("propulsion/engine[1]/n2");
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_pph();
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/fuel-flow-rate-pps");
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pitch-angle-rad");
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/reverser-angle-rad");
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/yaw-angle-rad");
			p_ptr[21] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/injection_cmd");
			p_ptr[22] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/set-running");
			p_ptr[23] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/fuel_dump");
			break;

		  case(3):
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");

			p_ptr[12] = fdmExec->GetPropertyValue("propulsion/engine[1]/thrust-lbs");//Thrust-lbs
			p_ptr[13] = fdmExec->GetPropertyValue("propulsion/engine[1]/n1");
			p_ptr[14] = fdmExec->GetPropertyValue("propulsion/engine[1]/n2");
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_pph();
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/fuel-flow-rate-pps");
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pitch-angle-rad");
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/reverser-angle-rad");
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/yaw-angle-rad");
			p_ptr[21] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/injection_cmd");
			p_ptr[22] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/set-running");
			p_ptr[23] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/fuel_dump");

			p_ptr[24] = fdmExec->GetPropertyValue("propulsion/engine[2]/thrust-lbs");//Thrust-lbs
			p_ptr[25] = fdmExec->GetPropertyValue("propulsion/engine[2]/n1");
			p_ptr[26] = fdmExec->GetPropertyValue("propulsion/engine[2]/n2");
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_pph();
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/fuel-flow-rate-pps");
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pitch-angle-rad");
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/reverser-angle-rad");
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/yaw-angle-rad");
			p_ptr[33] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/injection_cmd");
			p_ptr[34] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/set-running");
			p_ptr[35] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/fuel_dump");

			break;
		  case(4):
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");

			p_ptr[12] = fdmExec->GetPropertyValue("propulsion/engine[1]/thrust-lbs");//Thrust-lbs
			p_ptr[13] = fdmExec->GetPropertyValue("propulsion/engine[1]/n1");
			p_ptr[14] = fdmExec->GetPropertyValue("propulsion/engine[1]/n2");
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_pph();
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/fuel-flow-rate-pps");
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pitch-angle-rad");
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/reverser-angle-rad");
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/yaw-angle-rad");
			p_ptr[21] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/injection_cmd");
			p_ptr[22] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/set-running");
			p_ptr[23] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/fuel_dump");

			p_ptr[24] = fdmExec->GetPropertyValue("propulsion/engine[2]/thrust-lbs");//Thrust-lbs
			p_ptr[25] = fdmExec->GetPropertyValue("propulsion/engine[2]/n1");
			p_ptr[26] = fdmExec->GetPropertyValue("propulsion/engine[2]/n2");
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_pph();
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/fuel-flow-rate-pps");
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pitch-angle-rad");
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/reverser-angle-rad");
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/yaw-angle-rad");
			p_ptr[33] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/injection_cmd");
			p_ptr[34] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/set-running");
			p_ptr[35] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/fuel_dump");

			p_ptr[36] = fdmExec->GetPropertyValue("propulsion/engine[3]/thrust-lbs");//Thrust-lbs
			p_ptr[37] = fdmExec->GetPropertyValue("propulsion/engine[3]/n1");
			p_ptr[38] = fdmExec->GetPropertyValue("propulsion/engine[3]/n2");
			p_ptr[39] = fdmExec->GetPropulsion()->GetEngine(3)->getFuelFlow_pph();
			p_ptr[40] = fdmExec->GetPropertyValue("propulsion/engine[3]/fuel-flow-rate-pps");
			p_ptr[41] = fdmExec->GetPropertyValue("propulsion/engine[3]/pt-lbs_sqft");
			p_ptr[42] = fdmExec->GetPropertyValue("propulsion/engine[3]/pitch-angle-rad");
			p_ptr[43] = fdmExec->GetPropertyValue("propulsion/engine[3]/reverser-angle-rad");
			p_ptr[44] = fdmExec->GetPropertyValue("propulsion/engine[3]/yaw-angle-rad");
			p_ptr[45] = (int)fdmExec->GetPropertyValue("propulsion/engine[3]/injection_cmd");
			p_ptr[46] = (int)fdmExec->GetPropertyValue("propulsion/engine[3]/set-running");
			p_ptr[47] = (int)fdmExec->GetPropertyValue("propulsion/engine[3]/fuel_dump");

			break;
		}
		break;
		}

		//*********if turbine, dual engined************
		//engine/augmentation  boolean
		//engine//cutoff   boolean
		//engine/thrust_lb
		//engine/egt_degf
		//engine/epr
		//p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
		//engine/n1
		//engine/n2
		//engine/oil-pressure-psi
		//engine/running boolean

		//*********if turbine, quad engined************
		//engine/augmentation  boolean
		//engine//cutoff   boolean
		//engine/thrust_lb
		//engine/egt_degf
		//engine/epr
		//p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
		//engine/n1
		//engine/n2
		//engine/oil-pressure-psi
		//engine/running boolean

		// Calculated Outputs output vector [pilot-g alpha-dot beta-dot mach vc-kts ve-kts climb-rate/]
		c_ptr[0] = fdmExec->GetPropertyValue("accelerations/Nz");//Nz
		c_ptr[1] = fdmExec->GetAuxiliary()->Getalpha();// Alpha in radians
		c_ptr[2] = fdmExec->GetAuxiliary()->Getadot();// Alphadot in radians/sec
		c_ptr[3] = fdmExec->GetAuxiliary()->Getbeta();// Beta in radians
		c_ptr[4] = fdmExec->GetAuxiliary()->Getbdot();// Betadot in radians/sec
		c_ptr[5] = fdmExec->GetAuxiliary()->GetVcalibratedFPS();//Cal airspeed fps
		c_ptr[6] = fdmExec->GetAuxiliary()->GetVcalibratedKTS();//Cal airspeed kts
		c_ptr[7] = fdmExec->GetAuxiliary()->GetVt();//VT fps
		c_ptr[8] = fdmExec->GetAuxiliary()->GetVground();//Vel ground fps
		c_ptr[9] = fdmExec->GetAuxiliary()->GetMach();//Mach
		c_ptr[10] = fdmExec->GetPropagate()->Gethdot();//h-dot-fps

		//State derivatives
		dx_ptr[0] = fdmExec->GetPropagate()->GetUVWdot(1); // U fps
		dx_ptr[1] = fdmExec->GetPropagate()->GetUVWdot(2);// V fps
		dx_ptr[2] = fdmExec->GetPropagate()->GetUVWdot(3);// W fps
		dx_ptr[3] = fdmExec->GetPropagate()->GetPQRdot(1);// Roll rate rad/sec
		dx_ptr[4] = fdmExec->GetPropagate()->GetPQRdot(2);//Pitch rate rad/sec
		dx_ptr[5] = fdmExec->GetPropagate()->GetPQRdot(3);//Yaw rate rad/sec
		
		// Aerodynamic outputs
		a_ptr[0] = fdmExec->GetAerodynamics()->GetvFw(1);// Force X axis lbs-force
		a_ptr[1] = fdmExec->GetAerodynamics()->GetvFw(2);// Force Y axis lbs-force
		a_ptr[2] = fdmExec->GetAerodynamics()->GetvFw(3);// Force Z axis lbs-force
		a_ptr[3] = fdmExec->GetAerodynamics()->GetMoments(1);// Moment about the X axis
		a_ptr[4] = fdmExec->GetAerodynamics()->GetMoments(2);// Moment about the Y axis
		a_ptr[5] = fdmExec->GetAerodynamics()->GetMoments(3);// Moment about the Z axis


		/*JSBSim generated states are printed out to workspace for every simulation cycle for testing*/
	  if ( verbosityLevel == eDebug ){
		mexPrintf("\nDebug Output: JSBSim generated inputs, states are printed out to workspace for every simulation cycle for testing\n");
		mexPrintf("Controls.\n");
		mexPrintf("\tThr Cmd %f \n",fdmExec->GetFCS()->GetThrottleCmd(0));
		mexPrintf("\tThr Pos %f \n",fdmExec->GetFCS()->GetThrottlePos(0));
		mexPrintf("\tDaL Cmd %f \n",fdmExec->GetFCS()->GetDaCmd());
		mexPrintf("\tDaL Pos %f \n",fdmExec->GetFCS()->GetDaLPos(0));
		mexPrintf("\tDe Cmd %f \n",fdmExec->GetFCS()->GetDeCmd());
		mexPrintf("\tDe Pos %f \n",fdmExec->GetFCS()->GetDePos(0));
		mexPrintf("\tDr Cmd %f \n",fdmExec->GetFCS()->GetDrCmd());
		mexPrintf("\tDr Pos %f \n",fdmExec->GetFCS()->GetDrPos(0));
		mexPrintf("States.\n");
		mexPrintf("\tU %f (ft/s/s)\n",fdmExec->GetPropagate()->GetUVW(1));
		mexPrintf("\tV %f (ft/s/s)\n",fdmExec->GetPropagate()->GetUVW(2));
		mexPrintf("\tW %f (ft/s/s)\n",fdmExec->GetPropagate()->GetUVW(3));
		mexPrintf("\tP %f (ft/s/s)\n",fdmExec->GetPropagate()->GetPQR(1));
		mexPrintf("\tQ %f (ft/s/s)\n",fdmExec->GetPropagate()->GetPQR(2));
		mexPrintf("\tR %f (ft/s/s)\n",fdmExec->GetPropagate()->GetPQR(3));
		mexPrintf("\tH %f (ft/s/s)\n",fdmExec->GetPropagate()->GetAltitudeASL());
		mexPrintf("\tPhi %f (ft/s/s)\n",fdmExec->GetPropagate()->GetEuler(1));
		mexPrintf("\tTheta %f (ft/s/s)\n",fdmExec->GetPropagate()->GetEuler(2));
		mexPrintf("\tPsi %f (ft/s/s)\n",fdmExec->GetPropagate()->GetEuler(3));
		mexPrintf("\n");
		mexPrintf("Simulation dt %f\n",fdmExec->GetState()->Getdt());
	    mexPrintf("Simulation sim-time %f\n",fdmExec->GetSimTime());
		mexPrintf("\n");
		mexPrintf("\tState derivatives calculated.\n");

		
		mexPrintf("\t[u_dot, v_dot, w_dot] = [%f, %f, %f] (ft/s/s)\n",
			propagate->GetUVWdot(1),propagate->GetUVWdot(2),propagate->GetUVWdot(3));
		mexPrintf("\t[p_dot, q_dot, r_dot] = [%f, %f, %f] (rad/s/s)\n",
			propagate->GetPQRdot(1),propagate->GetPQRdot(2),propagate->GetPQRdot(3));
		mexPrintf("\n");
		
		
		mexPrintf("Call to UpdateStates completed\n");
		mexPrintf("**********************************************************\n");
		}
		return 1;
}

bool JSBSimInterface::UpdateDerivatives(double *u_ptr, double *dx_ptr, double *x_ptr, double *fc_ptr, double *p_ptr, double *c_ptr, double *a_ptr)
{

	//fdmExec->GetState()->SuspendIntegration(); With integration suspended, no control changes are propagated

	//Set the states in JSBSim for when we are using Simulink integration
		fdmExec->GetPropagate()->SetUVW(1,x_ptr[0]);
		fdmExec->GetPropagate()->SetUVW(2,x_ptr[1]);
		fdmExec->GetPropagate()->SetUVW(3,x_ptr[2]);
		fdmExec->GetPropagate()->SetPQR(1,x_ptr[3]);
		fdmExec->GetPropagate()->SetPQR(2,x_ptr[4]);
		fdmExec->GetPropagate()->SetPQR(3,x_ptr[5]);
		fdmExec->GetPropagate()->SetAltitudeASL(x_ptr[6]);
		fdmExec->GetPropagate()->SetLongitudeDeg(x_ptr[7]);
		fdmExec->GetPropagate()->SetLatitudeDeg(x_ptr[8]);
		//Call functions to set the Quaternoins using Euler angles
		SetEuler(1,x_ptr[9]);
		SetEuler(2,x_ptr[10]);
		SetEuler(3,x_ptr[11]);
	//mexPrintf("\t Simulation dt %f\n",fdmExec->GetState()->Getdt());
	//mexPrintf("Simulation sim-time %f\n",fdmExec->GetSimTime());
	int engines = (int) fdmExec->GetPropulsion()->GetNumEngines();
	int eng_type = fdmExec->GetPropulsion()->GetEngine(0)->GetType();
	
	/* Receive updated control inputs from MexJSBSimSFun, propagate them through one JSBSim cycle
	 * and retrieve updated states and outputs, and return them to MexJSBSimSFunction.
	 */

	   /* New control inputs from S-Function 
		* Control Input Vector = [throttle aileron elevator rudder mixture set-run flaps gear]
		*/
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++)
			fdmExec->GetFCS()->SetThrottleCmd(i, u_ptr[0]);//control the throttle(s)
		fdmExec->GetFCS()->SetDaCmd(u_ptr[1]);//control the ailerons
		fdmExec->GetFCS()->SetDeCmd(u_ptr[2]);//control the elevators
		fdmExec->GetFCS()->SetDrCmd(u_ptr[3]);//control the rudder(s)
		SetPropertyValue("fcs/mixture-cmd-norm",u_ptr[4]);//control the mixture
		for(unsigned i=0;i<fdmExec->GetPropulsion()->GetNumEngines();i++)
			fdmExec->GetPropulsion()->GetEngine(i)->SetRunning(u_ptr[5]);//set engine(s) to running
		fdmExec->GetFCS()->SetDfCmd(u_ptr[6]);//control the flaps
		fdmExec->GetFCS()->SetGearCmd(u_ptr[7]);//control the gear position

		fdmExec->Run();
		//fdmExec->GetState()->ResumeIntegration();
		
		dx_ptr[0] = fdmExec->GetPropagate()->GetUVWdot(1); // U fps
		dx_ptr[1] = fdmExec->GetPropagate()->GetUVWdot(2);// V fps
		dx_ptr[2] = fdmExec->GetPropagate()->GetUVWdot(3);// W fps
		dx_ptr[3] = fdmExec->GetPropagate()->GetPQRdot(1);// Roll rate rad/sec
		dx_ptr[4] = fdmExec->GetPropagate()->GetPQRdot(2);//Pitch rate rad/sec
		dx_ptr[5] = fdmExec->GetPropagate()->GetPQRdot(3);//Yaw rate rad/sec
		//fdmExec->GetPropagate()->GetQuaternion();
		//dx_ptr[6] = propagate->GetVState()->vQtrn(1);// Quaternion 1
		//dx_ptr[7] = propagate->GetVState()->vQtrn(2);// Quaternion 2
		//dx_ptr[8] = propagate->GetVState()->vQtrn(3);// Quaternion 3
		//dx_ptr[9] = propagate->GetVState()->vQtrn(4);// Quaternion 4
		dx_ptr[6] = fdmExec->GetPropagate()->Gethdot();//Altitude in feet
		dx_ptr[7] = fdmExec->GetPropagate()->GetLocationdot(1);// Longitude geographic position in deg
		dx_ptr[8] = fdmExec->GetPropagate()->GetLocationdot(2);// Latitude geographic position in deg
		//dx_ptr[12] = fdmExec->GetPropagate()->GetGeodeticAltitude();// Z geographic position in feet
		//fdmExec->GetPropagate()->GetQtrndot();// Get Quaternoin derivative 
		//dx_ptr[9] = GetEulerDot(1);// Phi (Roll Angle) in radians
		dx_ptr[9] = fdmExec->GetAuxiliary()->GetEulerRates(1);// Phi (Roll Angle) in radians
		//dx_ptr[10] = GetEulerDot(2);// Theta (Pitch Angle) in radians
		dx_ptr[10] = fdmExec->GetAuxiliary()->GetEulerRates(2);// Theta (Pitch Angle) in radians
		//dx_ptr[11] = GetEulerDot(3);// Psi (Yaw Angle) in radians
		dx_ptr[11] = fdmExec->GetAuxiliary()->GetEulerRates(3);// Psi (Yaw Angle) in radians
		
		//dx_ptr[12] = fdmExec->GetAuxiliary()->Getadot();// Alpha in radians
		//dx_ptr[13] = fdmExec->GetAuxiliary()->Getbdot();// Beta in radians
		//fdmExec->Run();
		
		/* Flight Controls output vector [throttle left-aileron elevator tvc rudder flap right-aileron speedbrake
		 * spoiler lef gear nosewheel-steering gear-unit-WOW]
		 */
		fc_ptr[0] = fdmExec->GetFCS()->GetThrottlePos(0);//fcs/throttle-pos-norm 
		fc_ptr[1] = fdmExec->GetFCS()->GetDaLPos(0);//fcs/left-aileron-pos-rad 0=rad, 1=deg, 2=norm
		fc_ptr[2] = fdmExec->GetFCS()->GetDePos(0);//fcs/elevator-pos-rad
		fc_ptr[3] = fdmExec->GetPropertyValue("fcs/tvc-pos-rad");//tvc-pos-rad
		fc_ptr[4] = fdmExec->GetFCS()->GetDrPos(0);//fcs/rudder-pos-rad
		fc_ptr[5] = fdmExec->GetFCS()->GetDfPos(0);//fcs/flap-pos-norm
		fc_ptr[6] = fdmExec->GetFCS()->GetDaRPos(0);//fcs/right-aileron-pos-rad
		fc_ptr[7] = fdmExec->GetFCS()->GetDsbPos(0);//fcs/speedbrake-pos-rad
		fc_ptr[8] = fdmExec->GetFCS()->GetDspPos(0);//fcs/spoiler-pos-rad
		fc_ptr[9] = fdmExec->GetPropertyValue("fcs/lef-pos-rad");//lef-pos-rad
		fc_ptr[10] = fdmExec->GetFCS()->GetGearPos();//gear/gear-pos-norm
		fc_ptr[11] = fdmExec->GetFCS()->GetSteerPosDeg(0);//Nose-gear-steering-pos-deg
		fc_ptr[12] = (int)fdmExec->GetPropertyValue("gear/unit/WOW");//Gear-WOW


     switch(eng_type)
	 {
	 case(2)://Piston engines
		switch(engines)
		{
		  case(1):
	   /* Propulsion output vector for piston engined aircraft p_ptr [] This will be sized based on # of engines
		* There are 12 output properties for each engine [RPM thrust mixture fuel-flow advance-ratio engine-power
		* pt vol-eff bsfc torque blade-angle pitch]  Each engine will have these output properties, so for example, if
		* there are 3 engines, then there will be 36 total (3 x 12) outputs.
		*/

		//*********if piston, single engined************
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch
			break;

		//*********if piston, dual engined************
		  case(2):
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch
			

			p_ptr[12] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[13] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[14] = fdmExec->GetPropulsion()->GetEngine(1)->GetMixture();//mixture
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/advance-ratio");//advance-ratio
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/power-hp");//engine/egt-degf
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[21] = fdmExec->GetPropertyValue("propulsion/engine[1]/torque");//engine/thruster/torque
			p_ptr[22] = fdmExec->GetPropertyValue("propulsion/engine[1]/blade-angle");//engine/thruster/rpm
			p_ptr[23] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetPitch();//engine/thruster/pitch
			break;

		  case(3):
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch

			p_ptr[12] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[13] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[14] = fdmExec->GetPropulsion()->GetEngine(1)->GetMixture();//mixture
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/advance-ratio");//advance-ratio
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/power-hp");//engine/egt-degf
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[21] = fdmExec->GetPropertyValue("propulsion/engine[1]/torque");//engine/thruster/torque
			p_ptr[22] = fdmExec->GetPropertyValue("propulsion/engine[1]/blade-angle");//engine/thruster/rpm
			p_ptr[23] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetPitch();//engine/thruster/pitch

			p_ptr[24] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[25] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[26] = fdmExec->GetPropulsion()->GetEngine(2)->GetMixture();//mixture
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/advance-ratio");//advance-ratio
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/power-hp");//engine/egt-degf
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[33] = fdmExec->GetPropertyValue("propulsion/engine[2]/torque");//engine/thruster/torque
			p_ptr[34] = fdmExec->GetPropertyValue("propulsion/engine[2]/blade-angle");//engine/thruster/rpm
			p_ptr[35] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetPitch();//engine/thruster/pitch

			break;
		  case(4):
			p_ptr[0] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetRPM();//Propellor RPM
			p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetThrust();//Propellor thrust_lb
			p_ptr[2] = fdmExec->GetPropulsion()->GetEngine(0)->GetMixture();//engine mixture
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/advance-ratio");//advance-ratio
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/power-hp");//engine horsepower
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");//engine pt
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/volumetric-efficiency");//engine volumetric efficiency
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/bsfc-lbs_hphr");//engine bsfc in lbs/hp-hour
			p_ptr[9] = fdmExec->GetPropertyValue("propulsion/engine[0]/torque");//Propellor torque
			p_ptr[10] = fdmExec->GetPropertyValue("propulsion/engine[0]/blade-angle");//Propellor blade angle
			p_ptr[11] = fdmExec->GetPropulsion()->GetEngine(0)->GetThruster()->GetPitch();//Propellor pitch

			p_ptr[12] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[13] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[14] = fdmExec->GetPropulsion()->GetEngine(1)->GetMixture();//mixture
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/advance-ratio");//advance-ratio
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/power-hp");//engine/egt-degf
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[21] = fdmExec->GetPropertyValue("propulsion/engine[1]/torque");//engine/thruster/torque
			p_ptr[22] = fdmExec->GetPropertyValue("propulsion/engine[1]/blade-angle");//engine/thruster/rpm
			p_ptr[23] = fdmExec->GetPropulsion()->GetEngine(1)->GetThruster()->GetPitch();//engine/thruster/pitch

			p_ptr[24] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[25] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[26] = fdmExec->GetPropulsion()->GetEngine(2)->GetMixture();//mixture
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/advance-ratio");//advance-ratio
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/power-hp");//engine/egt-degf
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[33] = fdmExec->GetPropertyValue("propulsion/engine[2]/torque");//engine/thruster/torque
			p_ptr[34] = fdmExec->GetPropertyValue("propulsion/engine[2]/blade-angle");//engine/thruster/rpm
			p_ptr[35] = fdmExec->GetPropulsion()->GetEngine(2)->GetThruster()->GetPitch();//engine/thruster/pitch

			p_ptr[36] = fdmExec->GetPropulsion()->GetEngine(3)->GetThruster()->GetRPM();//thrust_lb
			p_ptr[37] = fdmExec->GetPropulsion()->GetEngine(3)->GetThruster()->GetThrust();//thrust_lb
			p_ptr[38] = fdmExec->GetPropulsion()->GetEngine(3)->GetMixture();//mixture
			p_ptr[39] = fdmExec->GetPropulsion()->GetEngine(3)->getFuelFlow_gph();//fuel flow in gph
			p_ptr[40] = fdmExec->GetPropertyValue("propulsion/engine[3]/advance-ratio");//advance-ratio
			p_ptr[41] = fdmExec->GetPropertyValue("propulsion/engine[3]/power-hp");//engine/egt-degf
			p_ptr[42] = fdmExec->GetPropertyValue("propulsion/engine[3]/pt-lbs_sqft");//engine/mp-osi
			p_ptr[43] = fdmExec->GetPropertyValue("propulsion/engine[3]/volumetric-efficiency");//engine/oil-pressure-psi
			p_ptr[44] = fdmExec->GetPropertyValue("propulsion/engine[3]/bsfc-lbs_hphr");//engine/oil-temperature-degf
			p_ptr[45] = fdmExec->GetPropertyValue("propulsion/engine[3]/torque");//engine/thruster/torque
			p_ptr[46] = fdmExec->GetPropertyValue("propulsion/engine[3]/blade-angle");//engine/thruster/rpm
			p_ptr[47] = fdmExec->GetPropulsion()->GetEngine(3)->GetThruster()->GetPitch();//engine/thruster/pitch

			break;
		}
		break;

	   case(3)://Turbine engines
		 switch(engines)
		{
		  case(1):
	   /* Propulsion output vector p_ptr [] This will be sized based on # of engines
		* There are 12 output properties for each engine [RPM thrust mixture fuel-flow advance-ratio engine-power
		* pt vol-eff bsfc torque blade-angle pitch]  Each engine will have these output properties, so for example, if
		* there are 3 engines, then there will be 36 total (3 x 12) outputs.
		*/

			//*********if turbine, single engined************
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");
			break;

		//*********if turbine, dual engined************
		  case(2):
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");
			

			p_ptr[12] = fdmExec->GetPropertyValue("propulsion/engine[1]/thrust-lbs");//Thrust-lbs
			p_ptr[13] = fdmExec->GetPropertyValue("propulsion/engine[1]/n1");
			p_ptr[14] = fdmExec->GetPropertyValue("propulsion/engine[1]/n2");
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_pph();
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/fuel-flow-rate-pps");
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pitch-angle-rad");
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/reverser-angle-rad");
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/yaw-angle-rad");
			p_ptr[21] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/injection_cmd");
			p_ptr[22] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/set-running");
			p_ptr[23] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/fuel_dump");
			break;

		  case(3):
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");

			p_ptr[12] = fdmExec->GetPropertyValue("propulsion/engine[1]/thrust-lbs");//Thrust-lbs
			p_ptr[13] = fdmExec->GetPropertyValue("propulsion/engine[1]/n1");
			p_ptr[14] = fdmExec->GetPropertyValue("propulsion/engine[1]/n2");
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_pph();
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/fuel-flow-rate-pps");
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pitch-angle-rad");
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/reverser-angle-rad");
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/yaw-angle-rad");
			p_ptr[21] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/injection_cmd");
			p_ptr[22] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/set-running");
			p_ptr[23] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/fuel_dump");

			p_ptr[24] = fdmExec->GetPropertyValue("propulsion/engine[2]/thrust-lbs");//Thrust-lbs
			p_ptr[25] = fdmExec->GetPropertyValue("propulsion/engine[2]/n1");
			p_ptr[26] = fdmExec->GetPropertyValue("propulsion/engine[2]/n2");
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_pph();
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/fuel-flow-rate-pps");
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pitch-angle-rad");
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/reverser-angle-rad");
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/yaw-angle-rad");
			p_ptr[33] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/injection_cmd");
			p_ptr[34] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/set-running");
			p_ptr[35] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/fuel_dump");

			break;
		  case(4):
			p_ptr[0] = fdmExec->GetPropertyValue("propulsion/engine[0]/thrust-lbs");//Thrust-lbs
			p_ptr[1] = fdmExec->GetPropertyValue("propulsion/engine[0]/n1");
			p_ptr[2] = fdmExec->GetPropertyValue("propulsion/engine[0]/n2");
			p_ptr[3] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
			p_ptr[4] = fdmExec->GetPropertyValue("propulsion/engine[0]/fuel-flow-rate-pps");
			p_ptr[5] = fdmExec->GetPropertyValue("propulsion/engine[0]/pt-lbs_sqft");
			p_ptr[6] = fdmExec->GetPropertyValue("propulsion/engine[0]/pitch-angle-rad");
			p_ptr[7] = fdmExec->GetPropertyValue("propulsion/engine[0]/reverser-angle-rad");
			p_ptr[8] = fdmExec->GetPropertyValue("propulsion/engine[0]/yaw-angle-rad");
			p_ptr[9] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/injection_cmd");
			p_ptr[10] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/set-running");
			p_ptr[11] = (int)fdmExec->GetPropertyValue("propulsion/engine[0]/fuel_dump");

			p_ptr[12] = fdmExec->GetPropertyValue("propulsion/engine[1]/thrust-lbs");//Thrust-lbs
			p_ptr[13] = fdmExec->GetPropertyValue("propulsion/engine[1]/n1");
			p_ptr[14] = fdmExec->GetPropertyValue("propulsion/engine[1]/n2");
			p_ptr[15] = fdmExec->GetPropulsion()->GetEngine(1)->getFuelFlow_pph();
			p_ptr[16] = fdmExec->GetPropertyValue("propulsion/engine[1]/fuel-flow-rate-pps");
			p_ptr[17] = fdmExec->GetPropertyValue("propulsion/engine[1]/pt-lbs_sqft");
			p_ptr[18] = fdmExec->GetPropertyValue("propulsion/engine[1]/pitch-angle-rad");
			p_ptr[19] = fdmExec->GetPropertyValue("propulsion/engine[1]/reverser-angle-rad");
			p_ptr[20] = fdmExec->GetPropertyValue("propulsion/engine[1]/yaw-angle-rad");
			p_ptr[21] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/injection_cmd");
			p_ptr[22] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/set-running");
			p_ptr[23] = (int)fdmExec->GetPropertyValue("propulsion/engine[1]/fuel_dump");

			p_ptr[24] = fdmExec->GetPropertyValue("propulsion/engine[2]/thrust-lbs");//Thrust-lbs
			p_ptr[25] = fdmExec->GetPropertyValue("propulsion/engine[2]/n1");
			p_ptr[26] = fdmExec->GetPropertyValue("propulsion/engine[2]/n2");
			p_ptr[27] = fdmExec->GetPropulsion()->GetEngine(2)->getFuelFlow_pph();
			p_ptr[28] = fdmExec->GetPropertyValue("propulsion/engine[2]/fuel-flow-rate-pps");
			p_ptr[29] = fdmExec->GetPropertyValue("propulsion/engine[2]/pt-lbs_sqft");
			p_ptr[30] = fdmExec->GetPropertyValue("propulsion/engine[2]/pitch-angle-rad");
			p_ptr[31] = fdmExec->GetPropertyValue("propulsion/engine[2]/reverser-angle-rad");
			p_ptr[32] = fdmExec->GetPropertyValue("propulsion/engine[2]/yaw-angle-rad");
			p_ptr[33] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/injection_cmd");
			p_ptr[34] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/set-running");
			p_ptr[35] = (int)fdmExec->GetPropertyValue("propulsion/engine[2]/fuel_dump");

			p_ptr[36] = fdmExec->GetPropertyValue("propulsion/engine[3]/thrust-lbs");//Thrust-lbs
			p_ptr[37] = fdmExec->GetPropertyValue("propulsion/engine[3]/n1");
			p_ptr[38] = fdmExec->GetPropertyValue("propulsion/engine[3]/n2");
			p_ptr[39] = fdmExec->GetPropulsion()->GetEngine(3)->getFuelFlow_pph();
			p_ptr[40] = fdmExec->GetPropertyValue("propulsion/engine[3]/fuel-flow-rate-pps");
			p_ptr[41] = fdmExec->GetPropertyValue("propulsion/engine[3]/pt-lbs_sqft");
			p_ptr[42] = fdmExec->GetPropertyValue("propulsion/engine[3]/pitch-angle-rad");
			p_ptr[43] = fdmExec->GetPropertyValue("propulsion/engine[3]/reverser-angle-rad");
			p_ptr[44] = fdmExec->GetPropertyValue("propulsion/engine[3]/yaw-angle-rad");
			p_ptr[45] = (int)fdmExec->GetPropertyValue("propulsion/engine[3]/injection_cmd");
			p_ptr[46] = (int)fdmExec->GetPropertyValue("propulsion/engine[3]/set-running");
			p_ptr[47] = (int)fdmExec->GetPropertyValue("propulsion/engine[3]/fuel_dump");

			break;
		}
		break;
		}

		//*********if turbine, dual engined************
		//engine/augmentation  boolean
		//engine//cutoff   boolean
		//engine/thrust_lb
		//engine/egt_degf
		//engine/epr
		//p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
		//engine/n1
		//engine/n2
		//engine/oil-pressure-psi
		//engine/running boolean

		//*********if turbine, quad engined************
		//engine/augmentation  boolean
		//engine//cutoff   boolean
		//engine/thrust_lb
		//engine/egt_degf
		//engine/epr
		//p_ptr[1] = fdmExec->GetPropulsion()->GetEngine(0)->getFuelFlow_pph();
		//engine/n1
		//engine/n2
		//engine/oil-pressure-psi
		//engine/running boolean

		// Calculated Outputs output vector [pilot-g alpha-dot beta-dot mach vc-kts ve-kts climb-rate/]
		c_ptr[0] = fdmExec->GetPropertyValue("accelerations/Nz");//Nz
		c_ptr[1] = fdmExec->GetAuxiliary()->Getalpha();// Alpha in radians
		c_ptr[2] = fdmExec->GetAuxiliary()->Getadot();// Alphadot in radians/sec
		c_ptr[3] = fdmExec->GetAuxiliary()->Getbeta();// Beta in radians
		c_ptr[4] = fdmExec->GetAuxiliary()->Getbdot();// Betadot in radians/sec
		c_ptr[5] = fdmExec->GetAuxiliary()->GetVcalibratedFPS();//Cal airspeed fps
		c_ptr[6] = fdmExec->GetAuxiliary()->GetVcalibratedKTS();//Cal airspeed kts
		c_ptr[7] = fdmExec->GetAuxiliary()->GetVt();//VT fps
		c_ptr[8] = fdmExec->GetAuxiliary()->GetVground();//Vel ground fps
		c_ptr[9] = fdmExec->GetAuxiliary()->GetMach();//Mach
		c_ptr[10] = fdmExec->GetPropagate()->Gethdot();//h-dot-fps

		// Aerodynamic outputs
		a_ptr[0] = fdmExec->GetAerodynamics()->GetvFw(1);// Force X axis lbs-force
		a_ptr[1] = fdmExec->GetAerodynamics()->GetvFw(2);// Force Y axis lbs-force
		a_ptr[2] = fdmExec->GetAerodynamics()->GetvFw(3);// Force Z axis lbs-force
		a_ptr[3] = fdmExec->GetAerodynamics()->GetMoments(1);// Moment about the X axis
		a_ptr[4] = fdmExec->GetAerodynamics()->GetMoments(2);// Moment about the Y axis
		a_ptr[5] = fdmExec->GetAerodynamics()->GetMoments(3);// Moment about the Z axis
		
		/*JSBSim generated states are printed out to workspace for every simulation cycle for testing*/
	  if ( verbosityLevel == eDebug ){
	    mexPrintf("\nDebug Output: JSBSim generated inputs, states are printed out to workspace for every simulation cycle for testing\n");
		mexPrintf("Controls.\n");
		mexPrintf("\tThr Cmd %f \n",fdmExec->GetFCS()->GetThrottleCmd(0));
		mexPrintf("\tThr Pos %f \n",fdmExec->GetFCS()->GetThrottlePos(0));
		mexPrintf("\tDaL Cmd %f \n",fdmExec->GetFCS()->GetDaCmd());
		mexPrintf("\tDaL Pos %f \n",fdmExec->GetFCS()->GetDaLPos(0));
		mexPrintf("\tDe Cmd %f \n",fdmExec->GetFCS()->GetDeCmd());
		mexPrintf("\tDe Pos %f \n",fdmExec->GetFCS()->GetDePos(0));
		mexPrintf("\tDr Cmd %f \n",fdmExec->GetFCS()->GetDrCmd());
		mexPrintf("\tDr Pos %f \n",fdmExec->GetFCS()->GetDrPos(0));
		mexPrintf("States.\n");
		mexPrintf("\tU %f (ft/s/s)\n",fdmExec->GetPropagate()->GetUVW(1));
		mexPrintf("\tV %f (ft/s/s)\n",fdmExec->GetPropagate()->GetUVW(2));
		mexPrintf("\tW %f (ft/s/s)\n",fdmExec->GetPropagate()->GetUVW(3));
		mexPrintf("\tP %f (ft/s/s)\n",fdmExec->GetPropagate()->GetPQR(1));
		mexPrintf("\tQ %f (ft/s/s)\n",fdmExec->GetPropagate()->GetPQR(2));
		mexPrintf("\tR %f (ft/s/s)\n",fdmExec->GetPropagate()->GetPQR(3));
		mexPrintf("\tH %f (ft/s/s)\n",fdmExec->GetPropagate()->GetAltitudeASL());
		mexPrintf("\tPhi %f (ft/s/s)\n",fdmExec->GetPropagate()->GetEuler(1));
		mexPrintf("\tTheta %f (ft/s/s)\n",fdmExec->GetPropagate()->GetEuler(2));
		mexPrintf("\tPsi %f (ft/s/s)\n",fdmExec->GetPropagate()->GetEuler(3));
		
		// Calculate state derivatives
	//fdmExec->GetPropagate()->CalculatePQRdot();      // Angular rate derivative
	//fdmExec->GetPropagate()->CalculateUVWdot();      // Translational rate derivative
	//fdmExec->GetPropagate()->CalculateQuatdot();     // Angular orientation derivative
	//fdmExec->GetPropagate()->CalculateLocationdot(); // Translational position derivative

	_udot = propagate->GetUVWdot(1);
	_vdot = propagate->GetUVWdot(2);
	_wdot = propagate->GetUVWdot(3);
	_pdot = propagate->GetPQRdot(1);
	_qdot = propagate->GetPQRdot(2);
	_rdot = propagate->GetPQRdot(3);

		mexPrintf("\n");
		mexPrintf("\tSimulation dt %f\n",fdmExec->GetState()->Getdt());
	    mexPrintf("\tSimulation sim-time %f\n",fdmExec->GetSimTime());
		mexPrintf("\n");
		mexPrintf("State derivatives calculated.\n");

		
		mexPrintf("\t[u_dot, v_dot, w_dot] = [%f, %f, %f] (ft/s/s)\n",
			propagate->GetUVWdot(1),propagate->GetUVWdot(2),propagate->GetUVWdot(3));
		mexPrintf("\t[p_dot, q_dot, r_dot] = [%f, %f, %f] (rad/s/s)\n",
			propagate->GetPQRdot(1),propagate->GetPQRdot(2),propagate->GetPQRdot(3));
		mexPrintf("\n");
		
		mexPrintf("Call to UpdateDerivatives completed\n");
		mexPrintf("**********************************************************\n");
		}
		return 1;

}