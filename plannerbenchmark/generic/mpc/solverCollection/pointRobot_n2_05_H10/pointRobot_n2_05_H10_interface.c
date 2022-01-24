/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/pointRobot_n2_05_H10.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "pointRobot_n2_05_H10_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, pointRobot_n2_05_H10_callback_float *data, pointRobot_n2_05_H10_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((pointRobot_n2_05_H10_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void pointRobot_n2_05_H10_casadi2forces(pointRobot_n2_05_H10_float *x,        /* primal vars                                         */
                                 pointRobot_n2_05_H10_float *y,        /* eq. constraint multiplers                           */
                                 pointRobot_n2_05_H10_float *l,        /* ineq. constraint multipliers                        */
                                 pointRobot_n2_05_H10_float *p,        /* parameters                                          */
                                 pointRobot_n2_05_H10_float *f,        /* objective function (scalar)                         */
                                 pointRobot_n2_05_H10_float *nabla_f,  /* gradient of objective function                      */
                                 pointRobot_n2_05_H10_float *c,        /* dynamics                                            */
                                 pointRobot_n2_05_H10_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 pointRobot_n2_05_H10_float *h,        /* inequality constraints                              */
                                 pointRobot_n2_05_H10_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 pointRobot_n2_05_H10_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const pointRobot_n2_05_H10_callback_float *in[4];
    pointRobot_n2_05_H10_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	pointRobot_n2_05_H10_float w[35];
	
    /* temporary storage for CasADi sparse output */
    pointRobot_n2_05_H10_callback_float this_f;
    pointRobot_n2_05_H10_float nabla_f_sparse[7];
    pointRobot_n2_05_H10_float h_sparse[14];
    pointRobot_n2_05_H10_float nabla_h_sparse[24];
    pointRobot_n2_05_H10_float c_sparse[1];
    pointRobot_n2_05_H10_float nabla_c_sparse[1];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 <= stage && stage <= 8))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		pointRobot_n2_05_H10_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = pointRobot_n2_05_H10_objective_0_sparsity_out(1)[0];
			ncol = pointRobot_n2_05_H10_objective_0_sparsity_out(1)[1];
			colind = pointRobot_n2_05_H10_objective_0_sparsity_out(1) + 2;
			row = pointRobot_n2_05_H10_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		pointRobot_n2_05_H10_rktwo_0(x, p, c, nabla_c, pointRobot_n2_05_H10_cdyn_0rd_0, pointRobot_n2_05_H10_cdyn_0, threadID);
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		pointRobot_n2_05_H10_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = pointRobot_n2_05_H10_inequalities_0_sparsity_out(0)[0];
			ncol = pointRobot_n2_05_H10_inequalities_0_sparsity_out(0)[1];
			colind = pointRobot_n2_05_H10_inequalities_0_sparsity_out(0) + 2;
			row = pointRobot_n2_05_H10_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = pointRobot_n2_05_H10_inequalities_0_sparsity_out(1)[0];
			ncol = pointRobot_n2_05_H10_inequalities_0_sparsity_out(1)[1];
			colind = pointRobot_n2_05_H10_inequalities_0_sparsity_out(1) + 2;
			row = pointRobot_n2_05_H10_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((9 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		pointRobot_n2_05_H10_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = pointRobot_n2_05_H10_objective_1_sparsity_out(1)[0];
			ncol = pointRobot_n2_05_H10_objective_1_sparsity_out(1)[1];
			colind = pointRobot_n2_05_H10_objective_1_sparsity_out(1) + 2;
			row = pointRobot_n2_05_H10_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		pointRobot_n2_05_H10_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = pointRobot_n2_05_H10_inequalities_1_sparsity_out(0)[0];
			ncol = pointRobot_n2_05_H10_inequalities_1_sparsity_out(0)[1];
			colind = pointRobot_n2_05_H10_inequalities_1_sparsity_out(0) + 2;
			row = pointRobot_n2_05_H10_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = pointRobot_n2_05_H10_inequalities_1_sparsity_out(1)[0];
			ncol = pointRobot_n2_05_H10_inequalities_1_sparsity_out(1)[1];
			colind = pointRobot_n2_05_H10_inequalities_1_sparsity_out(1) + 2;
			row = pointRobot_n2_05_H10_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((pointRobot_n2_05_H10_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
