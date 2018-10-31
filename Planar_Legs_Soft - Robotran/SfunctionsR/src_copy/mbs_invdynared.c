//-------------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2008 by JF Collard
// Last update : 01/10/2008
//-------------------------------
//
// Gestion via Bugzilla :
// 01/10/2008 : JFC : Bug n°41
//


#ifdef INVDYNARED

#include "MBSfun.h"
#include "math.h"
#include "nrfct.h"

int invdynared(LocalDataStruct *lds,MBSdataStruct *s)
{
	int i,j,k;
	int iter=0;
	int nL,nC,nk;

	bool is_actuated;

	double term,d;

	// Expression des variables commandées 
	if (s->nqc>0) user_DrivenJoints(s,s->tsim);

	// Résolution des Contraintes
	if (s->Ncons > 0)
	{
		// résolution géométrique
		iter = mbs_close_geo(s, lds);

		if (iter>=lds->MAX_NR_ITER)
		{
			return -1;
		}

		// résolution cinématique
		mbs_close_kin(s, lds);

		// calcul des accélérations dépendantes (qddv = Bvuc * qdduc + bprim)
		// il pourrait être intéressant de comparer ce résultat avec les accélérations fournies par l'utilisateur
		nL = s->nqv;
		nk = lds->iquc[0];
		for (i=1;i<=nL;i++)
		{
			term = 0.0;
			for(k=1;k<=nk;k++)
			{
				term += lds->Bvuc[i][k]*s->qdd[lds->iquc[k]];
			}
			s->qdd[s->qv[i]] = term + lds->bp[i];
		}

	}//if(s->Ncons > 0)

	// calcul des forces appliquées sur les corps
	for (i=1;i<=s->nbody;i++)
	{
		for (j=1;j<=3;j++)
		{
			s->frc[j][i]=0.0;
			s->trq[j][i]=0.0;
		}
	}

	if (s->Nlink > 0) link(s->frc,s->trq,s->Fl,s->Z,s->Zd,s,s->tsim);

	if (s->Nxfrc > 0) extforces(s->frc,s->trq,s,s->tsim);

	s->Qq = user_JointForces(s,s->tsim);


	// calcul de la dynamique inverse de la structure ouverte
	invdyna(lds->phi,s,s->tsim);

	for (i=1;i<=s->njoint;i++)
	{
		lds->phi[i] = lds->phi[i] - s->Qq[i];
	}

	// calcul des forces articulaires actionnées
	if (s->nqv>0)
	{
		// b = Phiu + Bvu'*Phiv;
		nL = s->nqu;
		nk = s->nqv;
		for (i=1;i<=nL;i++)
		{
			term = 0.0;
			for (k=1;k<=nk;k++)
			{
				term += lds->Bvuc[k][i] * lds->phi[s->qv[k]]; // Bvu' * Phiv
			}
			lds->b[i] = term + lds->phi[lds->iquc[i]]; // + Phiu
		}

		// A = [eye(nqu), Bvu'];
		// induv = [ind_u ind_v];
		// [dummy1,colA] = intersect(induv,ind_a);
		// A = A(:,colA);

		// independent variable part
		nL = nC = s->nqu;
		nk = s->nqa;
		for (i=1;i<=nC;i++) // d'abord parcours des indices des var. indép. pour remplir les premières colonnes
		{
			is_actuated = false; // Cette variable indépendante est-elle actionnée ?
			for (k=1;k<=s->nqa;k++)
			{
				if (s->qa[k] == s->qu[i])
				{
					is_actuated = true;
					break;
				}
			}

			if (is_actuated) // A is part of the identity matrix
			{
				for (j=1;j<=nL;j++)
					lds->A[j][k] = 0.0;
				lds->A[i][k] = 1.0;
			}
		}

		// dependent variable part
		nL = s->nqu;
		nC = s->nqv;
		nk = s->nqa;
		for(i=1;i<=nC;i++) // ensuite, parcours des indices des var. dép. pour remplir les colonnes suivantes
		{
			is_actuated = false; // Cette variable dépendante est-elle actionnée ?
			for(k=1;k<=s->nqa;k++)
			{
				if (s->qa[k] == s->qv[i])
				{
					is_actuated = true;
					break;
				}
			}

			if (is_actuated) // A is part of the Bvu' matrix
			{
				for(j=1;j<=nL;j++)
					lds->A[j][k] = lds->Bvuc[i][j];
			}
		}

		if (s->nqa == s->nqu) // solution of the original equations: Qact(ind_a) = A\b;
		{
			// Décomposition LU de la matrice A
			ludcmp(lds->A,s->nqa,lds->ind_A,&d);
			for (i=1;i<=s->nqa;i++)
			{
				if (fabs(lds->A[i][i])<1e-12)
				{
#ifdef MATLAB_MEX_FILE
					mexWarnMsgTxt("Inverse Dynamics: problem may be singular");
#endif
					break;
				}
			}
			for (i=1;i<=s->nqa;i++)
			{
				lds->Qact[i] = lds->b[i];
			}
			lubksb(lds->A,s->nqa,lds->ind_A,lds->Qact);
		}
		else 
			if (s->nqa > s->nqu) // suractionnement
			// solution of the normal equations: Qact(ind_a) = (A'*A)\(A'*b)
			{
				// Décomposition SVD de la matrice A
				svdcmp(lds->A,s->nqu,s->nqa,lds->w,lds->v);
				/*			
				for(i=1;i<=s->nqa;i++)
				{
					if (fabs(lds->w[i])<1e-12)
					{
#ifdef MATLAB_MEX_FILE
						mexWarnMsgTxt("Inverse Dynamics: at least one singular value is equal to zero");
#endif
						break;
					}
				}
				*/			
				svbksb(lds->A,lds->w,lds->v,s->nqu,s->nqa,lds->b,lds->Qact);		
			}
			else // sous-actionnement => interdit
			{
#ifdef MATLAB_MEX_FILE
				mexErrMsgTxt("Inverse Dynamics: not enough actuated joint ! Choose other actuated joints.");
#endif
			}
/*
		// Lambda = (Jv')\(Phiv-Qact(ind_v));
		// -Jv'
		nL = s->nqv;
		nC = s->nhu;
		for(i=1;i<=nL;i++)
		{
			for(j=1;j<=nC;j++)
			{
				lds->mJvt[i][j] = -lds->Jac[s->hu[j]][s->qv[i]];
			}
		}
		// Décomposition LU de la matrice -Jvt
		ludcmp(lds->mJvt,s->nqv,lds->ind_mJvt,&d);

		for(i=1;i<=s->nhu;i++)
		{
			is_actuated = false;
			for(j=1;j<=s->nqa;j++)
				if (s->qv[i] == s->qa[j]){
					is_actuated = true;
					break;
				}

			if (is_actuated)		
				lds->lambda[i] = lds->phi[s->qv[i]] - lds->Qact[j]; // to be completed with Qact_v
			else
				lds->lambda[i] = lds->phi[s->qv[i]];
				
		}
		lubksb(lds->mJvt,s->nhu,lds->ind_mJvt,lds->lambda);
*/
	}
	else
	{
		// Qact = Phi(ind_a)
		for (i=1;i<=s->nqa;i++)
		{
			lds->Qact[i] = lds->phi[s->qa[i]];
		}
	}

	// Qc
	if (s->nqc > 0)
	{
		for (i=1;i<=s->nqc;i++)
		{
			lds->Qc[i] = lds->phi[s->qc[i]];
			if (s->nqv > 0)
			{
				term = 0.0;
				for (j=1;j<=s->nqv;j++)
				{
					is_actuated = false;
					for(k=1;k<=s->nqa;k++)
						if (s->qv[j] == s->qa[k]){
							is_actuated = true;
							break;
						}

					if (is_actuated)		
						term += lds->Bvuc[j][s->nqu+i] * (lds->phi[s->qv[j]]-lds->Qact[k]); // to be completed with Qact_v
					else
						term += lds->Bvuc[j][s->nqu+i] * (lds->phi[s->qv[j]]);
				}
				lds->Qc[i] += term;
			}
		}
	}

	return iter;
}

#endif
