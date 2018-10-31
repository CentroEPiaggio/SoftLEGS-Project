//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°37
// 02/04/2013 :  ND : Bug n°128
//

#include "MBSfun.h"

#if !defined HARDINPUT
MBSdataStruct *loadMBSsizes(const mxArray *s_ptr)
{
	MBSdataStruct *s;
	const mxArray *field_value_ptr;

	s = (MBSdataStruct*) malloc(sizeof(MBSdataStruct));


/* Parameters */

	/* nbody */
	field_value_ptr = mxGetField(s_ptr, 0,  "nbody");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nbody\"\n");
	s->nbody = (int) *mxGetPr(field_value_ptr);
	

	/* njoint */
//	field_value_ptr = mxGetField(s_ptr, 0,  "njoint");
	s->njoint = s->nbody; //njoint = nbody


	/* npt */
	field_value_ptr = mxGetField(s_ptr, 0,  "npt");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"npt\"\n");
	s->npt = (int) *mxGetPr(field_value_ptr);


	/* nqu */
	field_value_ptr = mxGetField(s_ptr, 0,  "nqu");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nqu\"\n");
	s->nqu = (int) *mxGetPr(field_value_ptr);


	/* nqc */
	field_value_ptr = mxGetField(s_ptr, 0,  "nqc");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nqc\"\n");
	s->nqc = (int) *mxGetPr(field_value_ptr);
	

	/* nqa */
	field_value_ptr = mxGetField(s_ptr, 0,  "nqa");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nqa\"\n");
	s->nqa = (int) *mxGetPr(field_value_ptr);
	

	/* nqv */
	field_value_ptr = mxGetField(s_ptr, 0,  "nqv");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nqv\"\n");
	s->nqv = (int) *mxGetPr(field_value_ptr);
 

	/* nhu */
	field_value_ptr = mxGetField(s_ptr, 0,  "nhu");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nhu\"\n");
	s->nhu = (int) *mxGetPr(field_value_ptr);
 

	/* nqlocked */
	field_value_ptr = mxGetField(s_ptr, 0,  "nqlocked");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nqlocked\"\n");
	s->nqlocked = (int) *mxGetPr(field_value_ptr);

	/* nqdriven */
	field_value_ptr = mxGetField(s_ptr, 0,  "nqdriven");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"nqdriven\"\n");
	s->nqdriven = (int) *mxGetPr(field_value_ptr);
 
	/* Nloopc */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nloopc");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nloopc\"\n");
	s->Nloopc = (int) *mxGetPr(field_value_ptr);

	/* Nuserc */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nuserc");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nuserc\"\n");
	s->Nuserc = (int) *mxGetPr(field_value_ptr);

	/* Ncons */
	field_value_ptr = mxGetField(s_ptr, 0,  "Ncons");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Ncons\"\n");
	s->Ncons = (int) *mxGetPr(field_value_ptr);

	/* Nlink */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nlink");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nlink\"\n");
	s->Nlink = (int) *mxGetPr(field_value_ptr);

	/* Nlink3D */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nlink3D");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nlink3D\"\n");
	s->Nlink3D = (int) *mxGetPr(field_value_ptr);

	/* Nsensor */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nsensor");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nsensor\"\n");
	s->Nsensor = (int) *mxGetPr(field_value_ptr);
	
	/* Nxfrc */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nxfrc");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nxfrc\"\n");
	s->Nxfrc = (int) *mxGetPr(field_value_ptr);

	/* Nwheel */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nwheel");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nwheel\"\n");
	s->Nwheel = (int) *mxGetPr(field_value_ptr);

#if !defined SENSORKIN
	/* Nuser_model */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nuser_model");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nuser_model\"\n");
	s->Nuser_model = (int) *mxGetPr(field_value_ptr);
#endif

	/* Nux */
	field_value_ptr = mxGetField(s_ptr, 0,  "Nux");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"Nux\"\n");
	s->Nux = (int) *mxGetPr(field_value_ptr);

	return s;
}

MBSdataStruct * loadMBSdata(const mxArray *s_ptr)
{
	MBSdataStruct *s;
	const mxArray    *field_value_ptr,*field_value_ptr2,*field_value_ptr3;
	mxArray *usm_ptr;
	int i,j, ncol;

	s = loadMBSsizes(s_ptr);

//===  Index parameters  ======================================//

	/* qc */
	if (s->nqc)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "qc");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"qc\"\n");
		s->qc = (int*) calloc(s->nqc+1,sizeof(int));
		s->qc[0] = s->nqc;
		for(i=1;i<=s->nqc;i++) 
			s->qc[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->qc = NULL;
  
	/* qu */
	if (s->nqu)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "qu");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"qu\"\n");
		s->qu = (int*) calloc(s->nqu+1,sizeof(int));
		s->qu[0] = s->nqu;
		for(i=1;i<=s->nqu;i++) 
			s->qu[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->qu = NULL;
	
	/* qa */
	if (s->nqa)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "qa");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"qa\"\n");
		s->qa = (int*) calloc(s->nqa+1,sizeof(int));
		s->qa[0] = s->nqa;
		for(i=1;i<=s->nqa;i++) 
			s->qa[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->qa = NULL;
	
	/* qv */
	if (s->nqv)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "qv");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"qv\"\n");
		s->qv = (int*) calloc(s->nqv+1,sizeof(int));
		s->qv[0] = s->nqv;
		for(i=1;i<=s->nqv;i++) 
			s->qv[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->qv = NULL;

	/* hu */ // JFC : 15/01/2008 : ajout
	if (s->nhu)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "hu");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"hu\"\n");
		s->hu = (int*) calloc(s->nhu+1,sizeof(int));
		s->hu[0] = s->nhu;
		for(i=1;i<=s->nhu;i++) 
			s->hu[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->hu = NULL;
	
	/* qlocked */
	if (s->nqlocked)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "qlocked");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"qlocked\"\n");
		s->qlocked = (int*) calloc(s->nqlocked+1,sizeof(int));
		s->qlocked[0] = s->nqlocked;
		for(i=1;i<=s->nqlocked;i++) 
			s->qlocked[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->qlocked = NULL;

	/* qdriven */
	if (s->nqdriven)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "qdriven");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"qdriven\"\n");
		s->qdriven = (int*) calloc(s->nqdriven+1,sizeof(int));
		s->qdriven[0] = s->nqdriven;
		for(i=1;i<=s->nqdriven;i++) 
			s->qdriven[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->qdriven = NULL;

//===   Initial values   ==========================================================//

	/* q0,qd0,qdd0 *
	field_value_ptr  = mxGetField(s_ptr, 0,  "q0");
	field_value_ptr2 = mxGetField(s_ptr, 0,  "qd0");
	field_value_ptr3 = mxGetField(s_ptr, 0,  "qdd0");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"q0\"\n");
	if (field_value_ptr2 == NULL)
    	mexErrMsgTxt("Could not get field \"qd0\"\n");
	if (field_value_ptr3 == NULL)
		mexErrMsgTxt("Could not get field \"qdd0\"\n");
	s->q0   = (double*) calloc(s->njoint+1,sizeof(double));
	s->qd0  = (double*) calloc(s->njoint+1,sizeof(double));
	s->qdd0 = (double*) calloc(s->njoint+1,sizeof(double));
	s->q0[0]   = (double) s->njoint;
	s->qd0[0]  = (double) s->njoint;
	s->qdd0[0] = (double) s->njoint;
	for(i=1;i<=s->njoint;i++) 
	{
		s->q0[i]   = mxGetPr(field_value_ptr) [i-1];
		s->qd0[i]  = mxGetPr(field_value_ptr2)[i-1];
		s->qdd0[i] = mxGetPr(field_value_ptr3)[i-1];
	}
    
//===   Range values   ==========================================================//

	/* qmin */
	field_value_ptr  = mxGetField(s_ptr, 0,  "qmin");
	if (field_value_ptr != NULL)
	{
		// vérifier la taille de qmin
		// ...
		s->qmin = (double*) calloc(s->njoint+1,sizeof(double));
		s->qmin[0] = (double) s->njoint;
		// copier les valeurs de qmin
		// ...
	}
	else
		s->qmin = NULL;

	/* qmax */
	field_value_ptr  = mxGetField(s_ptr, 0,  "qmax");
	if (field_value_ptr != NULL)
	{
		// vérifier la taille de qmax
		// ...
		s->qmax = (double*) calloc(s->njoint+1,sizeof(double));
		s->qmax[0] = (double) s->njoint;
		// copier les valeurs de qmax
		// ...
	}
	else
		s->qmax = NULL;
    
//===   Work Variables   ==========================================================//

	/* q,qd,qdd */
	field_value_ptr  = mxGetField(s_ptr, 0,  "q");
	field_value_ptr2 = mxGetField(s_ptr, 0,  "qd");
	field_value_ptr3 = mxGetField(s_ptr, 0,  "qdd");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"q\n");
	if (field_value_ptr2 == NULL)
		mexErrMsgTxt("Could not get field \"qd\n");
	if (field_value_ptr3 == NULL)
		mexErrMsgTxt("Could not get field \"qdd\n");
	s->q   = (double*) calloc(s->njoint+1,sizeof(double));
	s->qd  = (double*) calloc(s->njoint+1,sizeof(double));
	s->qdd = (double*) calloc(s->njoint+1,sizeof(double));
	s->q[0]   = (double) s->njoint;
	s->qd[0]  = (double) s->njoint;
	s->qdd[0] = (double) s->njoint;
	for(i=1;i<=s->njoint;i++) 
	{
		s->q[i]   = mxGetPr(field_value_ptr) [i-1];
		s->qd[i]  = mxGetPr(field_value_ptr2)[i-1];
		s->qdd[i] = mxGetPr(field_value_ptr3)[i-1];
	}

	/* frc */
	s->frc[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->frc[i] = (double*) calloc(s->nbody+1,sizeof(double));
		s->frc[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->frc[i][j] = 0.0;
	}

	/* trq */
	s->trq[0] = NULL;
	for(i=1;i<=3;i++)
	{
		s->trq[i] = (double*) calloc(s->nbody+1,sizeof(double)); 
		s->trq[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
			s->trq[i][j] = 0.0;
	}
  
	/* Qq */
	s->Qq = (double*) calloc(s->njoint+1,sizeof(double));
	s->Qq[0] = (double) s->njoint;
	for(i=1;i<=s->njoint;i++) 
		s->Qq[i] = 0.0;

	/* tsim */
	s->tsim = 0.0;

//===   System parameters   ====================================================//
	
	/* dpt */
	s->dpt[0] = NULL;
	if (s->npt)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "dpt");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"dpt\"\n");
		for(i=1;i<=3;i++)
		{
			s->dpt[i] = (double*) calloc(s->npt+1,sizeof(double));
			s->dpt[i][0] = (double) s->npt;
			for(j=1;j<=s->npt;j++)
				s->dpt[i][j] = mxGetPr(field_value_ptr)[(i-1)+3*(j-1)];
		}
	}
	else // je pense que ce cas est impossible mais il faudrait vérifier avant de supprimer
		for(i=1;i<=3;i++)
			s->dpt[i] = NULL;
	
	/* l */
	s->l[0] = NULL;
	field_value_ptr = mxGetField(s_ptr, 0,  "l");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"l\"\n");
	for(i=1;i<=3;i++)
	{
		s->l[i] = (double*) calloc(s->nbody+1,sizeof(double)); 
		s->l[i][0] = (double) s->nbody; 
		for(j=1;j<=s->nbody;j++)
			s->l[i][j] = mxGetPr(field_value_ptr)[(i-1)+3*(j-1)];
	}

	/* m */
	field_value_ptr = mxGetField(s_ptr, 0,  "m");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"m\"\n");
	s->m = (double*) calloc(s->nbody+1,sizeof(double));
	s->m[0] = (double) s->nbody;
	for(i=1;i<=s->nbody;i++) 
		s->m[i] = mxGetPr(field_value_ptr)[i-1];

	/* In */
	s->In[0] = NULL;
	field_value_ptr = mxGetField(s_ptr, 0,  "In");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"In\"\n");
	for(i=1;i<=9;i++)
	{
		s->In[i] = (double*) calloc(s->nbody+1,sizeof(double)); 
		s->In[i][0] = (double) s->nbody;
		for(j=1;j<=s->nbody;j++)
	        s->In[i][j] = mxGetPr(field_value_ptr)[(i-1)+9*(j-1)];
	}
  
	/* g */
	s->g[0] = 0.0;
	field_value_ptr = mxGetField(s_ptr, 0,  "g");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"g\"\n");
	for(i=1;i<=3;i++) 
		s->g[i] = mxGetPr(field_value_ptr)[i-1];

/* Constraint data */
	
	/* lrod */
	field_value_ptr = mxGetField(s_ptr, 0,  "lrod");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"lrod\"\n");
	ncol = mxGetN(field_value_ptr);
	if (ncol)
	{
		s->lrod = (double*) calloc(ncol+1,sizeof(double));
		s->lrod[0] = ncol;
		for(i=1;i<=ncol;i++) 
			s->lrod[i] = mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->lrod = NULL;

	/* NRerr */
	field_value_ptr = mxGetField(s_ptr, 0,  "NRerr");
	if (field_value_ptr == NULL)
		s->NRerr = 1e-9; //default value
	else
		s->NRerr = mxGetScalar(field_value_ptr);


  
/* Link Data */
  
	/* Z,Zd,Fl */
	if (s->Nlink)
	{
		s->Z  = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Zd = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Fl = (double*) calloc(s->Nlink+1,sizeof(double));
		s->Z[0]  = (double) s->Nlink;
		s->Zd[0] = (double) s->Nlink;
		s->Fl[0] = (double) s->Nlink;
		for(i=1;i<=s->Nlink;i++)
		{
			s->Z[i]  = 0.0;
			s->Zd[i] = 0.0;
			s->Fl[i] = 0.0;
		}
	}
	else
	{
		s->Z  = NULL;
		s->Zd = NULL;
		s->Fl = NULL;
	}
    
	/* l3DWr */
	if (s->Nlink3D)
	{
		s->l3DWr = (double**) calloc(s->Nlink3D+1,sizeof(double*));
		s->l3DWr[0] = NULL;
		for(i=1;i<=s->Nlink3D;i++)
		{
			s->l3DWr[i] = (double*) calloc(6+1,sizeof(double));
			for(j=0;j<=6;j++)
				s->l3DWr[i][j] = 0.0;
		}
	}
	else
		s->l3DWr = NULL;
	
/* Ext. Forces Data */
	
	/* xfidpt */
	if (s->Nxfrc)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "xfidpt");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"xfidpt\"\n");
		s->xfidpt = (int*) calloc(s->Nxfrc+1,sizeof(int));
		s->xfidpt[0] = s->Nxfrc;
		ncol = mxGetN(field_value_ptr);
		for(i=1;i<=ncol;i++)
			s->xfidpt[i] =(int) mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->xfidpt = NULL;

	/* SWr */
	if (s->Nxfrc)
	{
		s->SWr = (double**) calloc(s->Nxfrc+1,sizeof(double*));
		s->SWr[0] = NULL;
		for(i=1;i<=s->Nxfrc;i++)
		{
			s->SWr[i] = (double*) calloc(9+1,sizeof(double));
			for(j=0;j<=9;j++)
				s->SWr[i][j] = 0.0;
		}
	}
	else
		s->SWr = NULL;

/* Wheel Data */

	/* rnom */
	if (s->Nwheel)
	{
		field_value_ptr = mxGetField(s_ptr, 0,  "rnom");
		if (field_value_ptr == NULL)
			mexErrMsgTxt("Could not get field \"rnom\"\n");
		s->rnom = (double*) calloc(s->Nwheel+1,sizeof(double));
		s->rnom[0] = (double) s->Nwheel;
		for(i=1;i<=s->Nwheel;i++) 
			s->rnom[i] = mxGetPr(field_value_ptr)[i-1];
	}
	else
		s->rnom = NULL;

#if !defined SENSORKIN
/* User Model */

	/* user_model */
	if (s->Nuser_model)
	{
		usm_ptr = mxGetField(s_ptr, 0,  "user_model");
		if (usm_ptr == NULL)
			mexErrMsgTxt("Could not get field \"user_model\"\n");
		s->user_model = loadUserModel(usm_ptr);
	}

	/* ux,uxd,ux0 */
	if (s->Nux)
	{
		field_value_ptr  = mxGetField(s_ptr, 0,  "ux");
		field_value_ptr2 = mxGetField(s_ptr, 0,  "uxd");
		field_value_ptr3 = mxGetField(s_ptr, 0,  "ux0");
		if ((field_value_ptr == NULL) || (field_value_ptr2 == NULL) || (field_value_ptr3 == NULL))
			mexErrMsgTxt("Could not get field \"ux\",\"uxd\" or \"ux0\"\n");
		s->ux  = (double*) calloc(s->Nux+1,sizeof(double));
		s->uxd = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux0 = (double*) calloc(s->Nux+1,sizeof(double));
		s->ux[0]  = (double) s->Nux;
		s->uxd[0] = (double) s->Nux;
		s->ux0[0] = (double) s->Nux;
		for(i=1;i<=s->Nux;i++) 
		{
			s->ux[i] =  mxGetPr(field_value_ptr)[i-1];
			s->uxd[i] = mxGetPr(field_value_ptr2)[i-1];
			s->ux0[i] = mxGetPr(field_value_ptr3)[i-1];
		}
	}
	else
	{
		s->ux  = NULL;
		s->uxd = NULL;
		s->ux0 = NULL;
	}
//#ifndef CMEX
/* User IO */
	s->user_IO = initUserIO(s);
//#endif

#endif

//===  Other  ========================================================//

	/* qddu */
	if (s->nqu)
	{
		s->qddu = (double*) calloc(s->nqu+1,sizeof(double));
		s->qddu[0] = (double) s->nqu;
	}
	else
		s->qddu = NULL;

	/* DonePart */
	field_value_ptr = mxGetField(s_ptr, 0,  "DonePart");
	if (field_value_ptr == NULL)
		mexErrMsgTxt("Could not get field \"DonePart\"\n");
	s->DonePart =(int) *mxGetPr(field_value_ptr);

	return s;
}
#endif


void checkMBSdata(MBSdataStruct *MBSdata)
{
#ifdef MATLAB_MEX_FILE
	if (MBSdata->DonePart <= 0){ 
#ifndef SENSORKIN
       mexErrMsgTxt("Coordinate partitioning must be performed first.\n");
#else
       mexWarnMsgTxt("Coordinate partitioning should be performed first for sensor kinematics.\n");
#endif
	} 
#endif
 }

/******************************************************************************/

void freeMBSdata(MBSdataStruct *s)	// liberation de memoire
{
	int i;
	
	// Donées géométriques et dynamiques
	if (s->npt)
		for(i=1;i<=3;i++)
			free(s->dpt[i]);
	
	for(i=1;i<=3;i++)
		free(s->l[i]);

	free(s->m);

   	for(i=1;i<=9;i++)
		free(s->In[i]);

	// Infos partitionnement
	if (s->nqlocked)
		free(s->qlocked);
	if (s->nqdriven)
		free(s->qdriven);
	if (s->nqc)
		free(s->qc);
	if (s->nqu)
		free(s->qu);
	if (s->nqa)
		free(s->qa);
	if (s->nqv)
		free(s->qv);

	// Variables articulaires, valeures initiales, limites
	free(s->q);
	free(s->qd);
	free(s->qdd);
//	free(s->q0);
//	free(s->qd0);
//	free(s->qdd0);
	if (s->qmin != NULL)
		free(s->qmin);
	if (s->qmax != NULL)
		free(s->qmax);

	// frc, trq, Qq
   	for(i=1;i<=3;i++)
	{
		free(s->frc[i]);
		free(s->trq[i]);
	}
	free(s->Qq);

	// Constraints
	if (s->lrod != NULL)
		free(s->lrod);

	// Links
	if (s->Nlink)
	{
		free(s->Z);
		free(s->Zd);
		free(s->Fl);
	}
	if (s->Nlink3D)
        for(i=1;i<=s->Nlink3D;i++)			
			free(s->l3DWr[i]);
	
	// Ext. forces
	if (s->Nxfrc)
	{
		free(s->xfidpt);
		for(i=1;i<=s->Nxfrc;i++)
			free(s->SWr[i]);
		free(s->SWr);
	}
	
	// Wheel
	if (s->Nwheel)
		free(s->rnom);

	// User state
	if (s->Nux)
	{
		free(s->ux);
		free(s->uxd);		// MD 21/12/2006: probleme inconnu - JFC: y a pas de raison
		free(s->ux0);
	}

#if !defined SENSORKIN
	// User models
	freeUserModel(s->user_model);
//#ifndef CMEX
	// User IO
	freeUserIO(s->user_IO,s);
//#endif
#endif

	// Other
	if (s->nqu)
		free(s->qddu);

	free(s);
/**/	
}

/******************************************************************************/
#ifdef MATLAB_MEX_FILE
void storeMBSdata(MBSdataStruct *s, char *s_name){

	mxArray    *s_ptr, *field_value_ptr;
	int i,j;

	/* Get pointer to MBSdataStruct */ 
#ifdef OLDVER
	s_ptr = mexGetArray(s_name,"base"); //v6
#else
	s_ptr = mexGetVariable("base", s_name);
#endif
	if (s_ptr == NULL){ 
		mexErrMsgTxt("Could not get structure variable.\n"); 
	} 

//===   Work Variables   ==========================================================//
	/* q */
	field_value_ptr = mxGetField(s_ptr, 0,  "q");
	for(i=1;i<=s->njoint;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->q[i];
	}
	mxSetField(s_ptr, 0, "q", field_value_ptr);
  

	/* qd */
	field_value_ptr = mxGetField(s_ptr, 0,  "qd");
	for(i=1;i<=s->njoint;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->qd[i];
	}
	mxSetField(s_ptr, 0, "qd", field_value_ptr);
  

	/* qdd */
	field_value_ptr = mxGetField(s_ptr, 0,  "qdd");
	for(i=1;i<=s->njoint;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->qdd[i];
	}
	mxSetField(s_ptr, 0, "qdd", field_value_ptr);
  

	/* frc */
	field_value_ptr = mxGetField(s_ptr, 0,  "frc");
	for(i=1;i<=3;i++)
	{
		for(j=1;j<=s->nbody;j++)
		{
			mxGetPr(field_value_ptr)[(i-1)+3*(j-1)] = s->frc[i][j];
		}
	}
	mxSetField(s_ptr, 0, "frc", field_value_ptr);


	/* trq */
	field_value_ptr = mxGetField(s_ptr, 0,  "trq");
	for(i=1;i<=3;i++)
	{
		for(j=1;j<=s->nbody;j++)
		{
			mxGetPr(field_value_ptr)[(i-1)+3*(j-1)] = s->trq[i][j];
		}
	}
	mxSetField(s_ptr, 0, "trq", field_value_ptr);


	/* Qq */
	field_value_ptr = mxGetField(s_ptr, 0,  "Qq");
	for(i=1;i<=s->njoint;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->Qq[i];
	}
	mxSetField(s_ptr, 0, "Qq", field_value_ptr);


	/* Fl */
	//field_value_ptr = mxGetField(s_ptr, 0,  "Fl");
	field_value_ptr = mxCreateDoubleMatrix(1,s->Nlink,mxREAL);
	for(i=1;i<=s->Nlink;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->Fl[i];
	}
	mxSetField(s_ptr, 0, "Fl", field_value_ptr);
	
	/* Z */
	//field_value_ptr = mxGetField(s_ptr, 0,  "Z");
	field_value_ptr = mxCreateDoubleMatrix(1,s->Nlink,mxREAL);
	for(i=1;i<=s->Nlink;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->Z[i];
	}
	mxSetField(s_ptr, 0, "Z", field_value_ptr);

	/* Zd */
	//field_value_ptr = mxGetField(s_ptr, 0,  "Zd");
	field_value_ptr = mxCreateDoubleMatrix(1,s->Nlink,mxREAL);
	for(i=1;i<=s->Nlink;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->Zd[i];
	}
	mxSetField(s_ptr, 0, "Zd", field_value_ptr);

	/* ux */
	field_value_ptr = mxGetField(s_ptr, 0,  "ux");
	for(i=1;i<=s->Nux;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->ux[i];
	}
	mxSetField(s_ptr, 0, "ux", field_value_ptr);

	/* uxd */
	field_value_ptr = mxGetField(s_ptr, 0,  "uxd");
	for(i=1;i<=s->Nux;i++) 
	{
		mxGetPr(field_value_ptr)[i-1] = s->uxd[i];
	}
	mxSetField(s_ptr, 0, "uxd", field_value_ptr);

#ifdef OLDVER
	mexPutArray(s_ptr,"base"); //v6
#else
	mexPutVariable("base", s_name,s_ptr);
#endif
}
#endif
