function varargout = integrator(varargin)
    %INTEGRATOR 
    %
    %  Function = INTEGRATOR(char name, char solver, struct:SX dae, struct opts)
    %    Create an ODE/DAE integrator Solves an initial value problem (IVP) coupled
    %  Function = INTEGRATOR(char name, char solver, struct:MX dae, struct opts)
    %
    %> INTEGRATOR(char name, char solver, struct:MX dae, struct opts)
    %------------------------------------------------------------------------
    %
    %
    %
    %> INTEGRATOR(char name, char solver, struct:SX dae, struct opts)
    %------------------------------------------------------------------------
    %
    %
    %Create an ODE/DAE integrator Solves an initial value problem (IVP) coupled
    %to a terminal value problem with differential equation given as an implicit
    %ODE coupled to an algebraic equation and a set of quadratures:
    %
    %
    %
    %::
    %
    %  Initial conditions at t=t0
    %  x(t0)  = x0
    %  q(t0)  = 0
    %  
    %  Forward integration from t=t0 to t=tf
    %  der(x) = function(x, z, p, t)                  Forward ODE
    %  0 = fz(x, z, p, t)                  Forward algebraic equations
    %  der(q) = fq(x, z, p, t)                  Forward quadratures
    %  
    %  Terminal conditions at t=tf
    %  rx(tf)  = rx0
    %  rq(tf)  = 0
    %  
    %  Backward integration from t=tf to t=t0
    %  der(rx) = gx(rx, rz, rp, x, z, p, t)        Backward ODE
    %  0 = gz(rx, rz, rp, x, z, p, t)        Backward algebraic equations
    %  der(rq) = gq(rx, rz, rp, x, z, p, t)        Backward quadratures
    %  
    %  where we assume that both the forward and backwards integrations are index-1
    %  (i.e. dfz/dz, dgz/drz are invertible) and furthermore that
    %  gx, gz and gq have a linear dependency on rx, rz and rp.
    %
    %
    %
    %General information
    %===================
    %
    %
    %
    %>List of available options
    %
    %+-----------------+-----------------+-----------------+-----------------+
    %|       Id        |      Type       |   Description   |     Used in     |
    %+=================+=================+=================+=================+
    %| augmented_optio | OT_DICT         | Options to be   | casadi::Integra |
    %| ns              |                 | passed down to  | tor             |
    %|                 |                 | the augmented   |                 |
    %|                 |                 | integrator, if  |                 |
    %|                 |                 | one is          |                 |
    %|                 |                 | constructed.    |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| common_options  | OT_DICT         | Options for     | casadi::OracleF |
    %|                 |                 | auto-generated  | unction         |
    %|                 |                 | functions       |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| expand          | OT_BOOL         | Replace MX with | casadi::Integra |
    %|                 |                 | SX expressions  | tor             |
    %|                 |                 | in problem      |                 |
    %|                 |                 | formulation     |                 |
    %|                 |                 | [false]         |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| grid            | OT_DOUBLEVECTOR | Time grid       | casadi::Integra |
    %|                 |                 |                 | tor             |
    %+-----------------+-----------------+-----------------+-----------------+
    %| monitor         | OT_STRINGVECTOR | Set of user     | casadi::OracleF |
    %|                 |                 | problem         | unction         |
    %|                 |                 | functions to be |                 |
    %|                 |                 | monitored       |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| number_of_finit | OT_INT          | Number of       | casadi::Integra |
    %| e_elements      |                 | finite elements | tor             |
    %+-----------------+-----------------+-----------------+-----------------+
    %| output_t0       | OT_BOOL         | Output the      | casadi::Integra |
    %|                 |                 | state at the    | tor             |
    %|                 |                 | initial time    |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| print_stats     | OT_BOOL         | Print out       | casadi::Integra |
    %|                 |                 | statistics      | tor             |
    %|                 |                 | after           |                 |
    %|                 |                 | integration     |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| rootfinder      | OT_STRING       | An implicit     | casadi::Integra |
    %|                 |                 | function solver | tor             |
    %+-----------------+-----------------+-----------------+-----------------+
    %| rootfinder_opti | OT_DICT         | Options to be   | casadi::Integra |
    %| ons             |                 | passed to the   | tor             |
    %|                 |                 | NLP Solver      |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| specific_option | OT_DICT         | Options for     | casadi::OracleF |
    %| s               |                 | specific auto-  | unction         |
    %|                 |                 | generated       |                 |
    %|                 |                 | functions,      |                 |
    %|                 |                 | overwriting the |                 |
    %|                 |                 | defaults from   |                 |
    %|                 |                 | common_options. |                 |
    %|                 |                 | Nested          |                 |
    %|                 |                 | dictionary.     |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| t0              | OT_DOUBLE       | Beginning of    | casadi::Integra |
    %|                 |                 | the time        | tor             |
    %|                 |                 | horizon         |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| tf              | OT_DOUBLE       | End of the time | casadi::Integra |
    %|                 |                 | horizon         | tor             |
    %+-----------------+-----------------+-----------------+-----------------+
    %
    %>Input scheme: casadi::IntegratorInput (INTEGRATOR_NUM_IN = 6)
    %
    %+------------------------+------------------------+------------------------+
    %|       Full name        |         Short          |      Description       |
    %+========================+========================+========================+
    %| INTEGRATOR_X0          | x0                     | Differential state at  |
    %|                        |                        | the initial time.      |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_P           | p                      | Parameters.            |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_Z0          | z0                     | Initial guess for the  |
    %|                        |                        | algebraic variable.    |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_RX0         | rx0                    | Backward differential  |
    %|                        |                        | state at the final     |
    %|                        |                        | time.                  |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_RP          | rp                     | Backward parameter     |
    %|                        |                        | vector.                |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_RZ0         | rz0                    | Initial guess for the  |
    %|                        |                        | backwards algebraic    |
    %|                        |                        | variable.              |
    %+------------------------+------------------------+------------------------+
    %
    %>Output scheme: casadi::IntegratorOutput (INTEGRATOR_NUM_OUT = 6)
    %
    %+------------------------+------------------------+------------------------+
    %|       Full name        |         Short          |      Description       |
    %+========================+========================+========================+
    %| INTEGRATOR_XF          | xf                     | Differential state at  |
    %|                        |                        | the final time.        |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_QF          | qf                     | Quadrature state at    |
    %|                        |                        | the final time.        |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_ZF          | zf                     | Algebraic variable at  |
    %|                        |                        | the final time.        |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_RXF         | rxf                    | Backward differential  |
    %|                        |                        | state at the initial   |
    %|                        |                        | time.                  |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_RQF         | rqf                    | Backward quadrature    |
    %|                        |                        | state at the initial   |
    %|                        |                        | time.                  |
    %+------------------------+------------------------+------------------------+
    %| INTEGRATOR_RZF         | rzf                    | Backward algebraic     |
    %|                        |                        | variable at the        |
    %|                        |                        | initial time.          |
    %+------------------------+------------------------+------------------------+
    %
    %List of plugins
    %===============
    %
    %
    %
    %- cvodes
    %
    %- idas
    %
    %- collocation
    %
    %- rk
    %
    %Note: some of the plugins in this list might not be available on your
    %system. Also, there might be extra plugins available to you that are not
    %listed here. You can obtain their documentation with
    %Integrator.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %cvodes
    %------
    %
    %
    %
    %Interface to CVodes from the Sundials suite.
    %
    %A call to evaluate will integrate to the end.
    %
    %You can retrieve the entire state trajectory as follows, after the evaluate
    %call: Call reset. Then call integrate(t_i) and getOuput for a series of
    %times t_i.
    %
    %Note: depending on the dimension and structure of your problem, you may
    %experience a dramatic speed-up by using a sparse linear solver:
    %
    %
    %
    %::
    %
    %     intg.setOption("linear_solver","csparse")
    %     intg.setOption("linear_solver_type","user_defined")
    %
    %
    %
    %>List of available options
    %
    %+------------------------+------------------------+------------------------+
    %|           Id           |          Type          |      Description       |
    %+========================+========================+========================+
    %| abstol                 | OT_DOUBLE              | Absolute tolerence for |
    %|                        |                        | the IVP solution       |
    %+------------------------+------------------------+------------------------+
    %| disable_internal_warni | OT_BOOL                | Disable SUNDIALS       |
    %| ngs                    |                        | internal warning       |
    %|                        |                        | messages               |
    %+------------------------+------------------------+------------------------+
    %| fsens_all_at_once      | OT_BOOL                | Calculate all right    |
    %|                        |                        | hand sides of the      |
    %|                        |                        | sensitivity equations  |
    %|                        |                        | at once                |
    %+------------------------+------------------------+------------------------+
    %| fsens_err_con          | OT_BOOL                | include the forward    |
    %|                        |                        | sensitivities in all   |
    %|                        |                        | error controls         |
    %+------------------------+------------------------+------------------------+
    %| interpolation_type     | OT_STRING              | Type of interpolation  |
    %|                        |                        | for the adjoint        |
    %|                        |                        | sensitivities          |
    %+------------------------+------------------------+------------------------+
    %| linear_multistep_metho | OT_STRING              | Integrator scheme:     |
    %| d                      |                        | BDF|adams              |
    %+------------------------+------------------------+------------------------+
    %| linear_solver          | OT_STRING              | A custom linear solver |
    %|                        |                        | creator function       |
    %|                        |                        | [default: csparse]     |
    %+------------------------+------------------------+------------------------+
    %| linear_solver_options  | OT_DICT                | Options to be passed   |
    %|                        |                        | to the linear solver   |
    %+------------------------+------------------------+------------------------+
    %| max_krylov             | OT_INT                 | Maximum Krylov         |
    %|                        |                        | subspace size          |
    %+------------------------+------------------------+------------------------+
    %| max_multistep_order    | OT_INT                 | Maximum order for the  |
    %|                        |                        | (variable-order)       |
    %|                        |                        | multistep method       |
    %+------------------------+------------------------+------------------------+
    %| max_num_steps          | OT_INT                 | Maximum number of      |
    %|                        |                        | integrator steps       |
    %+------------------------+------------------------+------------------------+
    %| newton_scheme          | OT_STRING              | Linear solver scheme   |
    %|                        |                        | in the Newton method:  |
    %|                        |                        | DIRECT|gmres|bcgstab|t |
    %|                        |                        | fqmr                   |
    %+------------------------+------------------------+------------------------+
    %| nonlinear_solver_itera | OT_STRING              | Nonlinear solver type: |
    %| tion                   |                        | NEWTON|functional      |
    %+------------------------+------------------------+------------------------+
    %| quad_err_con           | OT_BOOL                | Should the quadratures |
    %|                        |                        | affect the step size   |
    %|                        |                        | control                |
    %+------------------------+------------------------+------------------------+
    %| reltol                 | OT_DOUBLE              | Relative tolerence for |
    %|                        |                        | the IVP solution       |
    %+------------------------+------------------------+------------------------+
    %| second_order_correctio | OT_BOOL                | Second order           |
    %| n                      |                        | correction in the      |
    %|                        |                        | augmented system       |
    %|                        |                        | Jacobian [true]        |
    %+------------------------+------------------------+------------------------+
    %| sensitivity_method     | OT_STRING              | Sensitivity method:    |
    %|                        |                        | SIMULTANEOUS|staggered |
    %+------------------------+------------------------+------------------------+
    %| steps_per_checkpoint   | OT_INT                 | Number of steps        |
    %|                        |                        | between two            |
    %|                        |                        | consecutive            |
    %|                        |                        | checkpoints            |
    %+------------------------+------------------------+------------------------+
    %| stop_at_end            | OT_BOOL                | Stop the integrator at |
    %|                        |                        | the end of the         |
    %|                        |                        | interval               |
    %+------------------------+------------------------+------------------------+
    %| use_preconditioner     | OT_BOOL                | Precondition the       |
    %|                        |                        | iterative solver       |
    %|                        |                        | [default: true]        |
    %+------------------------+------------------------+------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %idas
    %----
    %
    %
    %
    %Interface to IDAS from the Sundials suite.
    %
    %Note: depending on the dimension and structure of your problem, you may
    %experience a dramatic speed-up by using a sparse linear solver:
    %
    %
    %
    %::
    %
    %     intg.setOption("linear_solver","csparse")
    %     intg.setOption("linear_solver_type","user_defined")
    %
    %
    %
    %>List of available options
    %
    %+------------------------+------------------------+------------------------+
    %|           Id           |          Type          |      Description       |
    %+========================+========================+========================+
    %| abstol                 | OT_DOUBLE              | Absolute tolerence for |
    %|                        |                        | the IVP solution       |
    %+------------------------+------------------------+------------------------+
    %| abstolv                | OT_DOUBLEVECTOR        | Absolute tolerarance   |
    %|                        |                        | for each component     |
    %+------------------------+------------------------+------------------------+
    %| calc_ic                | OT_BOOL                | Use IDACalcIC to get   |
    %|                        |                        | consistent initial     |
    %|                        |                        | conditions.            |
    %+------------------------+------------------------+------------------------+
    %| calc_icB               | OT_BOOL                | Use IDACalcIC to get   |
    %|                        |                        | consistent initial     |
    %|                        |                        | conditions for         |
    %|                        |                        | backwards system       |
    %|                        |                        | [default: equal to     |
    %|                        |                        | calc_ic].              |
    %+------------------------+------------------------+------------------------+
    %| cj_scaling             | OT_BOOL                | IDAS scaling on cj for |
    %|                        |                        | the user-defined       |
    %|                        |                        | linear solver module   |
    %+------------------------+------------------------+------------------------+
    %| disable_internal_warni | OT_BOOL                | Disable SUNDIALS       |
    %| ngs                    |                        | internal warning       |
    %|                        |                        | messages               |
    %+------------------------+------------------------+------------------------+
    %| first_time             | OT_DOUBLE              | First requested time   |
    %|                        |                        | as a fraction of the   |
    %|                        |                        | time interval          |
    %+------------------------+------------------------+------------------------+
    %| fsens_err_con          | OT_BOOL                | include the forward    |
    %|                        |                        | sensitivities in all   |
    %|                        |                        | error controls         |
    %+------------------------+------------------------+------------------------+
    %| init_xdot              | OT_DOUBLEVECTOR        | Initial values for the |
    %|                        |                        | state derivatives      |
    %+------------------------+------------------------+------------------------+
    %| interpolation_type     | OT_STRING              | Type of interpolation  |
    %|                        |                        | for the adjoint        |
    %|                        |                        | sensitivities          |
    %+------------------------+------------------------+------------------------+
    %| linear_solver          | OT_STRING              | A custom linear solver |
    %|                        |                        | creator function       |
    %|                        |                        | [default: csparse]     |
    %+------------------------+------------------------+------------------------+
    %| linear_solver_options  | OT_DICT                | Options to be passed   |
    %|                        |                        | to the linear solver   |
    %+------------------------+------------------------+------------------------+
    %| max_krylov             | OT_INT                 | Maximum Krylov         |
    %|                        |                        | subspace size          |
    %+------------------------+------------------------+------------------------+
    %| max_multistep_order    | OT_INT                 | Maximum order for the  |
    %|                        |                        | (variable-order)       |
    %|                        |                        | multistep method       |
    %+------------------------+------------------------+------------------------+
    %| max_num_steps          | OT_INT                 | Maximum number of      |
    %|                        |                        | integrator steps       |
    %+------------------------+------------------------+------------------------+
    %| max_step_size          | OT_DOUBLE              | Maximim step size      |
    %+------------------------+------------------------+------------------------+
    %| newton_scheme          | OT_STRING              | Linear solver scheme   |
    %|                        |                        | in the Newton method:  |
    %|                        |                        | DIRECT|gmres|bcgstab|t |
    %|                        |                        | fqmr                   |
    %+------------------------+------------------------+------------------------+
    %| quad_err_con           | OT_BOOL                | Should the quadratures |
    %|                        |                        | affect the step size   |
    %|                        |                        | control                |
    %+------------------------+------------------------+------------------------+
    %| reltol                 | OT_DOUBLE              | Relative tolerence for |
    %|                        |                        | the IVP solution       |
    %+------------------------+------------------------+------------------------+
    %| second_order_correctio | OT_BOOL                | Second order           |
    %| n                      |                        | correction in the      |
    %|                        |                        | augmented system       |
    %|                        |                        | Jacobian [true]        |
    %+------------------------+------------------------+------------------------+
    %| sensitivity_method     | OT_STRING              | Sensitivity method:    |
    %|                        |                        | SIMULTANEOUS|staggered |
    %+------------------------+------------------------+------------------------+
    %| steps_per_checkpoint   | OT_INT                 | Number of steps        |
    %|                        |                        | between two            |
    %|                        |                        | consecutive            |
    %|                        |                        | checkpoints            |
    %+------------------------+------------------------+------------------------+
    %| stop_at_end            | OT_BOOL                | Stop the integrator at |
    %|                        |                        | the end of the         |
    %|                        |                        | interval               |
    %+------------------------+------------------------+------------------------+
    %| suppress_algebraic     | OT_BOOL                | Suppress algebraic     |
    %|                        |                        | variables in the error |
    %|                        |                        | testing                |
    %+------------------------+------------------------+------------------------+
    %| use_preconditioner     | OT_BOOL                | Precondition the       |
    %|                        |                        | iterative solver       |
    %|                        |                        | [default: true]        |
    %+------------------------+------------------------+------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %collocation
    %-----------
    %
    %
    %
    %Fixed-step implicit Runge-Kutta integrator ODE/DAE integrator based on
    %collocation schemes
    %
    %The method is still under development
    %
    %>List of available options
    %
    %+------------------------+------------------------+------------------------+
    %|           Id           |          Type          |      Description       |
    %+========================+========================+========================+
    %| augmented_options      | OT_DICT                | Options to be passed   |
    %|                        |                        | down to the augmented  |
    %|                        |                        | integrator, if one is  |
    %|                        |                        | constructed.           |
    %+------------------------+------------------------+------------------------+
    %| collocation_scheme     | OT_STRING              | Collocation scheme:    |
    %|                        |                        | radau|legendre         |
    %+------------------------+------------------------+------------------------+
    %| expand                 | OT_BOOL                | Replace MX with SX     |
    %|                        |                        | expressions in problem |
    %|                        |                        | formulation [false]    |
    %+------------------------+------------------------+------------------------+
    %| grid                   | OT_DOUBLEVECTOR        | Time grid              |
    %+------------------------+------------------------+------------------------+
    %| interpolation_order    | OT_INT                 | Order of the           |
    %|                        |                        | interpolating          |
    %|                        |                        | polynomials            |
    %+------------------------+------------------------+------------------------+
    %| number_of_finite_eleme | OT_INT                 | Number of finite       |
    %| nts                    |                        | elements               |
    %+------------------------+------------------------+------------------------+
    %| output_t0              | OT_BOOL                | Output the state at    |
    %|                        |                        | the initial time       |
    %+------------------------+------------------------+------------------------+
    %| print_stats            | OT_BOOL                | Print out statistics   |
    %|                        |                        | after integration      |
    %+------------------------+------------------------+------------------------+
    %| rootfinder             | OT_STRING              | An implicit function   |
    %|                        |                        | solver                 |
    %+------------------------+------------------------+------------------------+
    %| rootfinder_options     | OT_DICT                | Options to be passed   |
    %|                        |                        | to the NLP Solver      |
    %+------------------------+------------------------+------------------------+
    %| t0                     | OT_DOUBLE              | Beginning of the time  |
    %|                        |                        | horizon                |
    %+------------------------+------------------------+------------------------+
    %| tf                     | OT_DOUBLE              | End of the time        |
    %|                        |                        | horizon                |
    %+------------------------+------------------------+------------------------+
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %rk --
    %
    %
    %
    %Fixed-step explicit Runge-Kutta integrator for ODEs Currently implements
    %RK4.
    %
    %The method is still under development
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %Joel Andersson
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(844, varargin{:});
end
