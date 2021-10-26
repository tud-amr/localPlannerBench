/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/solver_n5_05_N10.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "solver_n5_05_N10_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, solver_n5_05_N10_callback_float *data, solver_n5_05_N10_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((solver_n5_05_N10_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void solver_n5_05_N10_casadi2forces(solver_n5_05_N10_float *x,        /* primal vars                                         */
                                 solver_n5_05_N10_float *y,        /* eq. constraint multiplers                           */
                                 solver_n5_05_N10_float *l,        /* ineq. constraint multipliers                        */
                                 solver_n5_05_N10_float *p,        /* parameters                                          */
                                 solver_n5_05_N10_float *f,        /* objective function (scalar)                         */
                                 solver_n5_05_N10_float *nabla_f,  /* gradient of objective function                      */
                                 solver_n5_05_N10_float *c,        /* dynamics                                            */
                                 solver_n5_05_N10_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 solver_n5_05_N10_float *h,        /* inequality constraints                              */
                                 solver_n5_05_N10_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 solver_n5_05_N10_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const solver_n5_05_N10_callback_float *in[4];
    solver_n5_05_N10_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	solver_n5_05_N10_float w[261];
	
    /* temporary storage for CasADi sparse output */
    solver_n5_05_N10_callback_float this_f;
    solver_n5_05_N10_float nabla_f_sparse[11];
    solver_n5_05_N10_float h_sparse[45];
    solver_n5_05_N10_float nabla_h_sparse[170];
    solver_n5_05_N10_float c_sparse[1];
    solver_n5_05_N10_float nabla_c_sparse[1];
            
    
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
		solver_n5_05_N10_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = solver_n5_05_N10_objective_0_sparsity_out(1)[0];
			ncol = solver_n5_05_N10_objective_0_sparsity_out(1)[1];
			colind = solver_n5_05_N10_objective_0_sparsity_out(1) + 2;
			row = solver_n5_05_N10_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		solver_n5_05_N10_rktwo_0(x, p, c, nabla_c, solver_n5_05_N10_cdyn_0rd_0, solver_n5_05_N10_cdyn_0, threadID);
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solver_n5_05_N10_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = solver_n5_05_N10_inequalities_0_sparsity_out(0)[0];
			ncol = solver_n5_05_N10_inequalities_0_sparsity_out(0)[1];
			colind = solver_n5_05_N10_inequalities_0_sparsity_out(0) + 2;
			row = solver_n5_05_N10_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = solver_n5_05_N10_inequalities_0_sparsity_out(1)[0];
			ncol = solver_n5_05_N10_inequalities_0_sparsity_out(1)[1];
			colind = solver_n5_05_N10_inequalities_0_sparsity_out(1) + 2;
			row = solver_n5_05_N10_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((9 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solver_n5_05_N10_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = solver_n5_05_N10_objective_1_sparsity_out(1)[0];
			ncol = solver_n5_05_N10_objective_1_sparsity_out(1)[1];
			colind = solver_n5_05_N10_objective_1_sparsity_out(1) + 2;
			row = solver_n5_05_N10_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solver_n5_05_N10_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = solver_n5_05_N10_inequalities_1_sparsity_out(0)[0];
			ncol = solver_n5_05_N10_inequalities_1_sparsity_out(0)[1];
			colind = solver_n5_05_N10_inequalities_1_sparsity_out(0) + 2;
			row = solver_n5_05_N10_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = solver_n5_05_N10_inequalities_1_sparsity_out(1)[0];
			ncol = solver_n5_05_N10_inequalities_1_sparsity_out(1)[1];
			colind = solver_n5_05_N10_inequalities_1_sparsity_out(1) + 2;
			row = solver_n5_05_N10_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((solver_n5_05_N10_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
