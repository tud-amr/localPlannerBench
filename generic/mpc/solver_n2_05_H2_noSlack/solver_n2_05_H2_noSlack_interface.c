/*
 * CasADi to FORCESPRO Template - missing information to be filled in by createCasadi.m 
 * (C) embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.
 *
 * This file is part of the FORCESPRO client, and carries the same license.
 */ 

#ifdef __cplusplus
extern "C" {
#endif
    
#include "include/solver_n2_05_H2_noSlack.h"

#ifndef NULL
#define NULL ((void *) 0)
#endif

#include "solver_n2_05_H2_noSlack_model.h"



/* copies data from sparse matrix into a dense one */
static void sparse2fullcopy(solver_int32_default nrow, solver_int32_default ncol, const solver_int32_default *colidx, const solver_int32_default *row, solver_n2_05_H2_noSlack_callback_float *data, solver_n2_05_H2_noSlack_float *out)
{
    solver_int32_default i, j;
    
    /* copy data into dense matrix */
    for(i=0; i<ncol; i++)
    {
        for(j=colidx[i]; j<colidx[i+1]; j++)
        {
            out[i*nrow + row[j]] = ((solver_n2_05_H2_noSlack_float) data[j]);
        }
    }
}




/* CasADi to FORCESPRO interface */
extern void solver_n2_05_H2_noSlack_casadi2forces(solver_n2_05_H2_noSlack_float *x,        /* primal vars                                         */
                                 solver_n2_05_H2_noSlack_float *y,        /* eq. constraint multiplers                           */
                                 solver_n2_05_H2_noSlack_float *l,        /* ineq. constraint multipliers                        */
                                 solver_n2_05_H2_noSlack_float *p,        /* parameters                                          */
                                 solver_n2_05_H2_noSlack_float *f,        /* objective function (scalar)                         */
                                 solver_n2_05_H2_noSlack_float *nabla_f,  /* gradient of objective function                      */
                                 solver_n2_05_H2_noSlack_float *c,        /* dynamics                                            */
                                 solver_n2_05_H2_noSlack_float *nabla_c,  /* Jacobian of the dynamics (column major)             */
                                 solver_n2_05_H2_noSlack_float *h,        /* inequality constraints                              */
                                 solver_n2_05_H2_noSlack_float *nabla_h,  /* Jacobian of inequality constraints (column major)   */
                                 solver_n2_05_H2_noSlack_float *hess,     /* Hessian (column major)                              */
                                 solver_int32_default stage,     /* stage number (0 indexed)                           */
								 solver_int32_default iteration, /* iteration number of solver                         */
								 solver_int32_default threadID   /* Id of caller thread                                */)
{
    /* CasADi input and output arrays */
    const solver_n2_05_H2_noSlack_callback_float *in[4];
    solver_n2_05_H2_noSlack_callback_float *out[7];
	

	/* Allocate working arrays for CasADi */
	solver_n2_05_H2_noSlack_float w[81];
	
    /* temporary storage for CasADi sparse output */
    solver_n2_05_H2_noSlack_callback_float this_f;
    solver_n2_05_H2_noSlack_float nabla_f_sparse[6];
    solver_n2_05_H2_noSlack_float h_sparse[14];
    solver_n2_05_H2_noSlack_float nabla_h_sparse[24];
    solver_n2_05_H2_noSlack_float c_sparse[1];
    solver_n2_05_H2_noSlack_float nabla_c_sparse[1];
            
    
    /* pointers to row and column info for 
     * column compressed format used by CasADi */
    solver_int32_default nrow, ncol;
    const solver_int32_default *colind, *row;
    
    /* set inputs for CasADi */
    in[0] = x;
    in[1] = p;
    in[2] = l;
    in[3] = y;

	if ((0 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solver_n2_05_H2_noSlack_objective_0(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = solver_n2_05_H2_noSlack_objective_0_sparsity_out(1)[0];
			ncol = solver_n2_05_H2_noSlack_objective_0_sparsity_out(1)[1];
			colind = solver_n2_05_H2_noSlack_objective_0_sparsity_out(1) + 2;
			row = solver_n2_05_H2_noSlack_objective_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		solver_n2_05_H2_noSlack_rktwo_0(x, p, c, nabla_c, solver_n2_05_H2_noSlack_cdyn_0rd_0, solver_n2_05_H2_noSlack_cdyn_0, threadID);
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solver_n2_05_H2_noSlack_inequalities_0(in, out, NULL, w, 0);
		if( h )
		{
			nrow = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(0)[0];
			ncol = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(0)[1];
			colind = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(0) + 2;
			row = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(1)[0];
			ncol = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(1)[1];
			colind = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(1) + 2;
			row = solver_n2_05_H2_noSlack_inequalities_0_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
	if ((1 == stage))
	{
		
		
		out[0] = &this_f;
		out[1] = nabla_f_sparse;
		solver_n2_05_H2_noSlack_objective_1(in, out, NULL, w, 0);
		if( nabla_f )
		{
			nrow = solver_n2_05_H2_noSlack_objective_1_sparsity_out(1)[0];
			ncol = solver_n2_05_H2_noSlack_objective_1_sparsity_out(1)[1];
			colind = solver_n2_05_H2_noSlack_objective_1_sparsity_out(1) + 2;
			row = solver_n2_05_H2_noSlack_objective_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_f_sparse, nabla_f);
		}
		
		out[0] = h_sparse;
		out[1] = nabla_h_sparse;
		solver_n2_05_H2_noSlack_inequalities_1(in, out, NULL, w, 0);
		if( h )
		{
			nrow = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(0)[0];
			ncol = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(0)[1];
			colind = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(0) + 2;
			row = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(0) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, h_sparse, h);
		}
		if( nabla_h )
		{
			nrow = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(1)[0];
			ncol = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(1)[1];
			colind = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(1) + 2;
			row = solver_n2_05_H2_noSlack_inequalities_1_sparsity_out(1) + 2 + (ncol + 1);
			sparse2fullcopy(nrow, ncol, colind, row, nabla_h_sparse, nabla_h);
		}
	}
    
    /* add to objective */
    if (f != NULL)
    {
        *f += ((solver_n2_05_H2_noSlack_float) this_f);
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
