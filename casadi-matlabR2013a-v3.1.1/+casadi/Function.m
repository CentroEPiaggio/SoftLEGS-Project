classdef Function < casadi.SharedObject
    %FUNCTION General function.
    %
    %
    %
    %A general function $f$ in casadi can be multi-input, multi-output. Number of
    %inputs: nin n_in() Number of outputs: nout n_out()  We can view this
    %function as a being composed of a ( nin, nout) grid of single-input, single-
    %output primitive functions. Each such primitive function $f_ {i, j}
    %\\forall i \\in [0, nin-1], j \\in [0, nout-1]$ can map as $\\mathbf
    %{R}^{n, m}\\to\\mathbf{R}^{p, q}$, in which n, m, p, q can take
    %different values for every (i, j) pair.  When passing input, you specify
    %which partition $i$ is active. You pass the numbers vectorized, as a vector
    %of size $(n*m)$. When requesting output, you specify which partition $j$ is
    %active. You get the numbers vectorized, as a vector of size $(p*q)$.  To
    %calculate Jacobians, you need to have $(m=1, q=1)$.
    %
    %Write the Jacobian as $J_ {i, j} = \\nabla f_{i, j} = \\frac
    %{\\partial f_{i, j}(\\vec{x})}{\\partial \\vec{x}}$.
    %
    %We have the following relationships for function mapping from a row vector
    %to a row vector:
    %
    %$ \\vec {s}_f = \\nabla f_{i, j} . \\vec{v}$ $ \\vec {s}_a =
    %(\\nabla f_{i, j})^T . \\vec{w}$
    %
    %Some quantities in these formulas must be transposed: input col: transpose $
    %\\vec {v} $ and $\\vec{s}_a$ output col: transpose $ \\vec {w} $ and
    %$\\vec{s}_f$  NOTE: Functions are allowed to modify their input arguments
    %when evaluating: implicitFunction, IDAS solver Further releases may disallow
    %this.
    %
    %Joel Andersson >List of available options
    %
    %+-----------------+-----------------+-----------------+-----------------+
    %|       Id        |      Type       |   Description   |     Used in     |
    %+=================+=================+=================+=================+
    %| ad_weight       | OT_DOUBLE       | Weighting       | casadi::Functio |
    %|                 |                 | factor for      | nInternal       |
    %|                 |                 | derivative calc |                 |
    %|                 |                 | ulation.When    |                 |
    %|                 |                 | there is an     |                 |
    %|                 |                 | option of       |                 |
    %|                 |                 | either using    |                 |
    %|                 |                 | forward or      |                 |
    %|                 |                 | reverse mode    |                 |
    %|                 |                 | directional     |                 |
    %|                 |                 | derivatives,    |                 |
    %|                 |                 | the condition a |                 |
    %|                 |                 | d_weight*nf<=(1 |                 |
    %|                 |                 | -ad_weight)*na  |                 |
    %|                 |                 | is used where   |                 |
    %|                 |                 | nf and na are   |                 |
    %|                 |                 | estimates of    |                 |
    %|                 |                 | the number of   |                 |
    %|                 |                 | forward/reverse |                 |
    %|                 |                 | mode            |                 |
    %|                 |                 | directional     |                 |
    %|                 |                 | derivatives     |                 |
    %|                 |                 | needed. By      |                 |
    %|                 |                 | default,        |                 |
    %|                 |                 | ad_weight is    |                 |
    %|                 |                 | calculated      |                 |
    %|                 |                 | automatically,  |                 |
    %|                 |                 | but this can be |                 |
    %|                 |                 | overridden by   |                 |
    %|                 |                 | setting this    |                 |
    %|                 |                 | option. In      |                 |
    %|                 |                 | particular, 0   |                 |
    %|                 |                 | means forcing   |                 |
    %|                 |                 | forward mode    |                 |
    %|                 |                 | and 1 forcing   |                 |
    %|                 |                 | reverse mode.   |                 |
    %|                 |                 | Leave unset for |                 |
    %|                 |                 | (class          |                 |
    %|                 |                 | specific)       |                 |
    %|                 |                 | heuristics.     |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| ad_weight_sp    | OT_DOUBLE       | Weighting       | casadi::Functio |
    %|                 |                 | factor for      | nInternal       |
    %|                 |                 | sparsity        |                 |
    %|                 |                 | pattern         |                 |
    %|                 |                 | calculation cal |                 |
    %|                 |                 | culation.Overri |                 |
    %|                 |                 | des default     |                 |
    %|                 |                 | behavior. Set   |                 |
    %|                 |                 | to 0 and 1 to   |                 |
    %|                 |                 | force forward   |                 |
    %|                 |                 | and reverse     |                 |
    %|                 |                 | mode            |                 |
    %|                 |                 | respectively.   |                 |
    %|                 |                 | Cf. option      |                 |
    %|                 |                 | "ad_weight".    |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| compiler        | OT_STRING       | Just-in-time    | casadi::Functio |
    %|                 |                 | compiler plugin | nInternal       |
    %|                 |                 | to be used.     |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| derivative_of   | OT_FUNCTION     | The function is | casadi::Functio |
    %|                 |                 | a derivative of | nInternal       |
    %|                 |                 | another         |                 |
    %|                 |                 | function. The   |                 |
    %|                 |                 | type of         |                 |
    %|                 |                 | derivative      |                 |
    %|                 |                 | (directional    |                 |
    %|                 |                 | derivative,     |                 |
    %|                 |                 | Jacobian) is    |                 |
    %|                 |                 | inferred from   |                 |
    %|                 |                 | the function    |                 |
    %|                 |                 | name.           |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| gather_stats    | OT_BOOL         | Deprecated      | casadi::Functio |
    %|                 |                 | option          | nInternal       |
    %|                 |                 | (ignored):      |                 |
    %|                 |                 | Statistics are  |                 |
    %|                 |                 | now always      |                 |
    %|                 |                 | collected.      |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| input_scheme    | OT_STRINGVECTOR | Custom input    | casadi::Functio |
    %|                 |                 | scheme          | nInternal       |
    %+-----------------+-----------------+-----------------+-----------------+
    %| inputs_check    | OT_BOOL         | Throw           | casadi::Functio |
    %|                 |                 | exceptions when | nInternal       |
    %|                 |                 | the numerical   |                 |
    %|                 |                 | values of the   |                 |
    %|                 |                 | inputs don't    |                 |
    %|                 |                 | make sense      |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| jac_penalty     | OT_DOUBLE       | When requested  | casadi::Functio |
    %|                 |                 | for a number of | nInternal       |
    %|                 |                 | forward/reverse |                 |
    %|                 |                 | directions, it  |                 |
    %|                 |                 | may be cheaper  |                 |
    %|                 |                 | to compute      |                 |
    %|                 |                 | first the full  |                 |
    %|                 |                 | jacobian and    |                 |
    %|                 |                 | then multiply   |                 |
    %|                 |                 | with seeds,     |                 |
    %|                 |                 | rather than     |                 |
    %|                 |                 | obtain the      |                 |
    %|                 |                 | requested       |                 |
    %|                 |                 | directions in a |                 |
    %|                 |                 | straightforward |                 |
    %|                 |                 | manner. Casadi  |                 |
    %|                 |                 | uses a          |                 |
    %|                 |                 | heuristic to    |                 |
    %|                 |                 | decide which is |                 |
    %|                 |                 | cheaper. A high |                 |
    %|                 |                 | value of        |                 |
    %|                 |                 | 'jac_penalty'   |                 |
    %|                 |                 | makes it less   |                 |
    %|                 |                 | likely for the  |                 |
    %|                 |                 | heurstic to     |                 |
    %|                 |                 | chose the full  |                 |
    %|                 |                 | Jacobian        |                 |
    %|                 |                 | strategy. The   |                 |
    %|                 |                 | special value   |                 |
    %|                 |                 | -1 indicates    |                 |
    %|                 |                 | never to use    |                 |
    %|                 |                 | the full        |                 |
    %|                 |                 | Jacobian        |                 |
    %|                 |                 | strategy        |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| jit             | OT_BOOL         | Use just-in-    | casadi::Functio |
    %|                 |                 | time compiler   | nInternal       |
    %|                 |                 | to speed up the |                 |
    %|                 |                 | evaluation      |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| jit_options     | OT_DICT         | Options to be   | casadi::Functio |
    %|                 |                 | passed to the   | nInternal       |
    %|                 |                 | jit compiler.   |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| max_num_dir     | OT_INT          | Specify the     | casadi::Functio |
    %|                 |                 | maximum number  | nInternal       |
    %|                 |                 | of directions   |                 |
    %|                 |                 | for derivative  |                 |
    %|                 |                 | functions.      |                 |
    %|                 |                 | Overrules the   |                 |
    %|                 |                 | builtin optimiz |                 |
    %|                 |                 | ed_num_dir.     |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| output_scheme   | OT_STRINGVECTOR | Custom output   | casadi::Functio |
    %|                 |                 | scheme          | nInternal       |
    %+-----------------+-----------------+-----------------+-----------------+
    %| print_time      | OT_BOOL         | print           | casadi::Functio |
    %|                 |                 | information     | nInternal       |
    %|                 |                 | about execution |                 |
    %|                 |                 | time            |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| regularity_chec | OT_BOOL         | Throw           | casadi::Functio |
    %| k               |                 | exceptions when | nInternal       |
    %|                 |                 | NaN or Inf      |                 |
    %|                 |                 | appears during  |                 |
    %|                 |                 | evaluation      |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| user_data       | OT_VOIDPTR      | A user-defined  | casadi::Functio |
    %|                 |                 | field that can  | nInternal       |
    %|                 |                 | be used to      |                 |
    %|                 |                 | identify the    |                 |
    %|                 |                 | function or     |                 |
    %|                 |                 | pass additional |                 |
    %|                 |                 | information     |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %| verbose         | OT_BOOL         | Verbose         | casadi::Functio |
    %|                 |                 | evaluation  for | nInternal       |
    %|                 |                 | debugging       |                 |
    %+-----------------+-----------------+-----------------+-----------------+
    %
    %C++ includes: function.hpp 
    %
  methods
    function delete(self)
      if self.swigPtr
        casadiMEX(748, self);
        self.swigPtr=[];
      end
    end
    function varargout = expand(self,varargin)
    %EXPAND Expand a function to SX.
    %
    %  Function = EXPAND(self)
    %  Function = EXPAND(self, char name, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(749, self, varargin{:});
    end
    function varargout = n_in(self,varargin)
    %N_IN Get the number of function inputs.
    %
    %  int = N_IN(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(750, self, varargin{:});
    end
    function varargout = n_out(self,varargin)
    %N_OUT Get the number of function outputs.
    %
    %  int = N_OUT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(751, self, varargin{:});
    end
    function varargout = size1_in(self,varargin)
    %SIZE1_IN Get input dimension.
    %
    %  int = SIZE1_IN(self, int ind)
    %  int = SIZE1_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(752, self, varargin{:});
    end
    function varargout = size2_in(self,varargin)
    %SIZE2_IN Get input dimension.
    %
    %  int = SIZE2_IN(self, int ind)
    %  int = SIZE2_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(753, self, varargin{:});
    end
    function varargout = size_in(self,varargin)
    %SIZE_IN Get input dimension.
    %
    %  [int,int] = SIZE_IN(self, int ind)
    %  [int,int] = SIZE_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(754, self, varargin{:});
    end
    function varargout = size1_out(self,varargin)
    %SIZE1_OUT Get output dimension.
    %
    %  int = SIZE1_OUT(self, int ind)
    %  int = SIZE1_OUT(self, char oname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(755, self, varargin{:});
    end
    function varargout = size2_out(self,varargin)
    %SIZE2_OUT Get output dimension.
    %
    %  int = SIZE2_OUT(self, int ind)
    %  int = SIZE2_OUT(self, char oname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(756, self, varargin{:});
    end
    function varargout = size_out(self,varargin)
    %SIZE_OUT Get output dimension.
    %
    %  [int,int] = SIZE_OUT(self, int ind)
    %  [int,int] = SIZE_OUT(self, char oname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(757, self, varargin{:});
    end
    function varargout = nnz_in(self,varargin)
    %NNZ_IN Get number of input nonzeros.
    %
    %  int = NNZ_IN(self)
    %  int = NNZ_IN(self, int ind)
    %  int = NNZ_IN(self, char iname)
    %
    %
    %For a particular input or for all of the inputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(758, self, varargin{:});
    end
    function varargout = nnz_out(self,varargin)
    %NNZ_OUT Get number of output nonzeros.
    %
    %  int = NNZ_OUT(self)
    %  int = NNZ_OUT(self, int ind)
    %  int = NNZ_OUT(self, char oname)
    %
    %
    %For a particular output or for all of the outputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(759, self, varargin{:});
    end
    function varargout = numel_in(self,varargin)
    %NUMEL_IN Get number of input elements.
    %
    %  int = NUMEL_IN(self)
    %  int = NUMEL_IN(self, int ind)
    %  int = NUMEL_IN(self, char iname)
    %
    %
    %For a particular input or for all of the inputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(760, self, varargin{:});
    end
    function varargout = numel_out(self,varargin)
    %NUMEL_OUT Get number of output elements.
    %
    %  int = NUMEL_OUT(self)
    %  int = NUMEL_OUT(self, int ind)
    %  int = NUMEL_OUT(self, char oname)
    %
    %
    %For a particular output or for all of the outputs
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(761, self, varargin{:});
    end
    function varargout = name_in(self,varargin)
    %NAME_IN Get input scheme name by index.
    %
    %  [char] = NAME_IN(self)
    %    Get input scheme.
    %  char = NAME_IN(self, int ind)
    %
    %
    %
    %> NAME_IN(self, int ind)
    %------------------------------------------------------------------------
    %
    %
    %Get input scheme name by index.
    %
    %
    %> NAME_IN(self)
    %------------------------------------------------------------------------
    %
    %
    %Get input scheme.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(762, self, varargin{:});
    end
    function varargout = name_out(self,varargin)
    %NAME_OUT Get output scheme name by index.
    %
    %  [char] = NAME_OUT(self)
    %    Get output scheme.
    %  char = NAME_OUT(self, int ind)
    %
    %
    %
    %> NAME_OUT(self, int ind)
    %------------------------------------------------------------------------
    %
    %
    %Get output scheme name by index.
    %
    %
    %> NAME_OUT(self)
    %------------------------------------------------------------------------
    %
    %
    %Get output scheme.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(763, self, varargin{:});
    end
    function varargout = index_in(self,varargin)
    %INDEX_IN Find the index for a string describing a particular entry of an input
    %
    %  int = INDEX_IN(self, char name)
    %
    %scheme.
    %
    %example: schemeEntry("x_opt") -> returns NLPSOL_X if FunctionInternal
    %adheres to SCHEME_NLPINput
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(764, self, varargin{:});
    end
    function varargout = index_out(self,varargin)
    %INDEX_OUT Find the index for a string describing a particular entry of an output
    %
    %  int = INDEX_OUT(self, char name)
    %
    %scheme.
    %
    %example: schemeEntry("x_opt") -> returns NLPSOL_X if FunctionInternal
    %adheres to SCHEME_NLPINput
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(765, self, varargin{:});
    end
    function varargout = default_in(self,varargin)
    %DEFAULT_IN Get default input value (NOTE: constant reference)
    %
    %  double = DEFAULT_IN(self, int ind)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(766, self, varargin{:});
    end
    function varargout = sparsity_in(self,varargin)
    %SPARSITY_IN Get sparsity of a given input.
    %
    %  Sparsity = SPARSITY_IN(self, int ind)
    %  Sparsity = SPARSITY_IN(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(767, self, varargin{:});
    end
    function varargout = sparsity_out(self,varargin)
    %SPARSITY_OUT Get sparsity of a given output.
    %
    %  Sparsity = SPARSITY_OUT(self, int ind)
    %  Sparsity = SPARSITY_OUT(self, char iname)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(768, self, varargin{:});
    end
    function varargout = factory(self,varargin)
    %FACTORY 
    %
    %  Function = FACTORY(self, char name, [char] s_in, [char] s_out, casadi::Function::AuxOut const & aux, struct opts)
    %
    %
      [varargout{1:nargout}] = casadiMEX(769, self, varargin{:});
    end
    function varargout = oracle(self,varargin)
    %ORACLE Get oracle.
    %
    %  Function = ORACLE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(770, self, varargin{:});
    end
    function varargout = wrap(self,varargin)
    %WRAP Wrap in an Function instance consisting of only one MX call.
    %
    %  Function = WRAP(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(771, self, varargin{:});
    end
    function varargout = nl_var(self,varargin)
    %NL_VAR [DEPRECATED] Which variables enter nonlinearly
    %
    %  [bool] = NL_VAR(self, char s_in, [char] s_out)
    %
    %
    %Use which_depends instead.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(772, self, varargin{:});
    end
    function varargout = which_depends(self,varargin)
    %WHICH_DEPENDS Which variables enter with some order.
    %
    %  [bool] = WHICH_DEPENDS(self, char s_in, [char] s_out, int order, bool tr)
    %
    %
    %Parameters:
    %-----------
    %
    %order:  Only 1 (linear) and 2 (nonlinear) allowed
    %
    %tr:  Flip the relationship. Return which expressions contain the variables
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(773, self, varargin{:});
    end
    function varargout = print_dimensions(self,varargin)
    %PRINT_DIMENSIONS Print dimensions of inputs and outputs.
    %
    %  PRINT_DIMENSIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(774, self, varargin{:});
    end
    function varargout = print_options(self,varargin)
    %PRINT_OPTIONS Print options to a stream.
    %
    %  PRINT_OPTIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(775, self, varargin{:});
    end
    function varargout = print_option(self,varargin)
    %PRINT_OPTION Print all information there is to know about a certain option.
    %
    %  PRINT_OPTION(self, char name)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(776, self, varargin{:});
    end
    function varargout = printDimensions(self,varargin)
    %PRINTDIMENSIONS [DEPRECATED] printDimensions has been renamed print_dimensions
    %
    %  PRINTDIMENSIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(777, self, varargin{:});
    end
    function varargout = printOptions(self,varargin)
    %PRINTOPTIONS [DEPRECATED] printOptions has been renamed print_options
    %
    %  PRINTOPTIONS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(778, self, varargin{:});
    end
    function varargout = printOption(self,varargin)
    %PRINTOPTION [DEPRECATED] printOption has been renamed print_option
    %
    %  PRINTOPTION(self, char name)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(779, self, varargin{:});
    end
    function varargout = print_free(self,varargin)
    %PRINT_FREE Print free variables.
    %
    %  PRINT_FREE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(780, self, varargin{:});
    end
    function varargout = jacobian(self,varargin)
    %JACOBIAN Generate a Jacobian function of output oind with respect to input iind.
    %
    %  Function = JACOBIAN(self, int iind, int oind, bool compact, bool symmetric)
    %  Function = JACOBIAN(self, char iind, int oind, bool compact, bool symmetric)
    %  Function = JACOBIAN(self, int iind, char oind, bool compact, bool symmetric)
    %  Function = JACOBIAN(self, char iind, char oind, bool compact, bool symmetric)
    %
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output
    %
    %The default behavior of this class is defined by the derived class. If
    %compact is set to true, only the nonzeros of the input and output
    %expressions are considered. If symmetric is set to true, the Jacobian being
    %calculated is known to be symmetric (usually a Hessian), which can be
    %exploited by the algorithm.
    %
    %The generated Jacobian has one more output than the calling function
    %corresponding to the Jacobian and the same number of inputs.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(781, self, varargin{:});
    end
    function varargout = setJacobian(self,varargin)
    %SETJACOBIAN Set the Jacobian function of output oind with respect to input iind NOTE:
    %
    %  SETJACOBIAN(self, Function jac, int iind, int oind, bool compact)
    %
    %Does not take ownership, only weak references to the Jacobians are kept
    %internally
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(782, self, varargin{:});
    end
    function varargout = gradient(self,varargin)
    %GRADIENT Generate a gradient function of output oind with respect to input iind.
    %
    %  Function = GRADIENT(self, int iind, int oind)
    %  Function = GRADIENT(self, char iind, int oind)
    %  Function = GRADIENT(self, int iind, char oind)
    %  Function = GRADIENT(self, char iind, char oind)
    %
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output
    %
    %The default behavior of this class is defined by the derived class. Note
    %that the output must be scalar. In other cases, use the Jacobian instead.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(783, self, varargin{:});
    end
    function varargout = tangent(self,varargin)
    %TANGENT Generate a tangent function of output oind with respect to input iind.
    %
    %  Function = TANGENT(self, int iind, int oind)
    %  Function = TANGENT(self, char iind, int oind)
    %  Function = TANGENT(self, int iind, char oind)
    %  Function = TANGENT(self, char iind, char oind)
    %
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output
    %
    %The default behavior of this class is defined by the derived class. Note
    %that the input must be scalar. In other cases, use the Jacobian instead.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(784, self, varargin{:});
    end
    function varargout = hessian(self,varargin)
    %HESSIAN Generate a Hessian function of output oind with respect to input iind.
    %
    %  Function = HESSIAN(self, int iind, int oind)
    %  Function = HESSIAN(self, char iind, int oind)
    %  Function = HESSIAN(self, int iind, char oind)
    %  Function = HESSIAN(self, char iind, char oind)
    %
    %
    %Parameters:
    %-----------
    %
    %iind:  The index of the input
    %
    %oind:  The index of the output
    %
    %The generated Hessian has two more outputs than the calling function
    %corresponding to the Hessian and the gradients.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(785, self, varargin{:});
    end
    function varargout = fullJacobian(self,varargin)
    %FULLJACOBIAN Generate a Jacobian function of all the inputs elements with respect to all
    %
    %  Function = FULLJACOBIAN(self)
    %
    %the output elements).
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(786, self, varargin{:});
    end
    function varargout = setFullJacobian(self,varargin)
    %SETFULLJACOBIAN Set the Jacobian of all the input nonzeros with respect to all output
    %
    %  SETFULLJACOBIAN(self, Function jac)
    %
    %nonzeros NOTE: Does not take ownership, only weak references to the Jacobian
    %are kept internally
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(787, self, varargin{:});
    end
    function varargout = call(self,varargin)
    %CALL Evaluate the function symbolically or numerically.
    %
    %  struct:DM = CALL(self, struct:DM arg, bool always_inline, bool never_inline)
    %  {DM} = CALL(self, {DM} arg, bool always_inline, bool never_inline)
    %  {SX} = CALL(self, {SX} arg, bool always_inline, bool never_inline)
    %  struct:SX = CALL(self, struct:SX arg, bool always_inline, bool never_inline)
    %  struct:MX = CALL(self, struct:MX arg, bool always_inline, bool never_inline)
    %  {MX} = CALL(self, {MX} arg, bool always_inline, bool never_inline)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(788, self, varargin{:});
    end
    function varargout = mapsum(self,varargin)
    %MAPSUM Evaluate symbolically in parallel and sum (matrix graph)
    %
    %  {MX} = MAPSUM(self, {MX} arg, char parallelization)
    %
    %
    %Parameters:
    %-----------
    %
    %parallelization:  Type of parallelization used: unroll|serial|openmp
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(789, self, varargin{:});
    end
    function varargout = mapaccum(self,varargin)
    %MAPACCUM Create a mapaccumulated version of this function.
    %
    %  Function = MAPACCUM(self, char name, int n, int n_accum, struct opts)
    %  Function = MAPACCUM(self, char name, int n, [int] accum_in, [int] accum_out, struct opts)
    %  Function = MAPACCUM(self, char name, int n, [char] accum_in, [char] accum_out, struct opts)
    %
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (x, u) -> (x_next , y )
    %  
    %
    %
    %
    %The the mapaccumulated version has the signature:
    %
    %::
    %
    %     F: (x0, U) -> (X , Y )
    %  
    %      with
    %          U: horzcat([u0, u1, ..., u_(N-1)])
    %          X: horzcat([x1, x2, ..., x_N])
    %          Y: horzcat([y0, y1, ..., y_(N-1)])
    %  
    %      and
    %          x1, y0 <- f(x0, u0)
    %          x2, y1 <- f(x1, u1)
    %          ...
    %          x_N, y_(N-1) <- f(x_(N-1), u_(N-1))
    %  
    %
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(790, self, varargin{:});
    end
    function varargout = map(self,varargin)
    %MAP Map with reduction A subset of the inputs are non-repeated and a subset of
    %
    %  Function = MAP(self, int n, char parallelization)
    %    Create a mapped version of this function.
    %  struct:MX = MAP(self, struct:MX arg, char parallelization)
    %    [DEPRECATED] Use map(int) instead
    %  {MX} = MAP(self, {MX} arg, char parallelization)
    %    [DEPRECATED] Use map(int) instead
    %  Function = MAP(self, char name, char parallelization, int n, struct opts)
    %    [DEPRECATED] Old syntax for map
    %  Function = MAP(self, char name, char parallelization, int n, [int] reduce_in, [int] reduce_out, struct opts)
    %  Function = MAP(self, char name, char parallelization, int n, [char] reduce_in, [char] reduce_out, struct opts)
    %
    %the outputs summed up.
    %
    %
    %> MAP(self, struct:MX arg, char parallelization)
    %> MAP(self, {MX} arg, char parallelization)
    %------------------------------------------------------------------------
    %
    %
    %[DEPRECATED] Use map(int) instead
    %
    %
    %> MAP(self, char name, char parallelization, int n, struct opts)
    %------------------------------------------------------------------------
    %
    %
    %[DEPRECATED] Old syntax for map
    %
    %
    %> MAP(self, int n, char parallelization)
    %------------------------------------------------------------------------
    %
    %
    %Create a mapped version of this function.
    %
    %Suppose the function has a signature of:
    %
    %::
    %
    %     f: (a, p) -> ( s )
    %  
    %
    %
    %
    %The the mapped version has the signature:
    %
    %::
    %
    %     F: (A, P) -> (S )
    %  
    %      with
    %          A: horzcat([a0, a1, ..., a_(N-1)])
    %          P: horzcat([p0, p1, ..., p_(N-1)])
    %          S: horzcat([s0, s1, ..., s_(N-1)])
    %      and
    %          s0 <- f(a0, p0)
    %          s1 <- f(a1, p1)
    %          ...
    %          s_(N-1) <- f(a_(N-1), p_(N-1))
    %  
    %
    %
    %
    %Parameters:
    %-----------
    %
    %parallelization:  Type of parallelization used: unroll|serial|openmp
    %
    %
    %> MAP(self, char name, char parallelization, int n, [int] reduce_in, [int] reduce_out, struct opts)
    %> MAP(self, char name, char parallelization, int n, [char] reduce_in, [char] reduce_out, struct opts)
    %------------------------------------------------------------------------
    %
    %
    %Map with reduction A subset of the inputs are non-repeated and a subset of
    %the outputs summed up.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(791, self, varargin{:});
    end
    function varargout = slice(self,varargin)
    %SLICE returns a new function with a selection of inputs/outputs of the original
    %
    %  Function = SLICE(self, char name, [int] order_in, [int] order_out, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(792, self, varargin{:});
    end
    function varargout = kernel_sum(self,varargin)
    %KERNEL_SUM [DEPRECATED] kernel_sum is no longer available
    %
    %  Function = KERNEL_SUM(self, char name, [int,int] size, double r, int n, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(795, self, varargin{:});
    end
    function varargout = derivative(self,varargin)
    %DERIVATIVE [DEPRECATED] Use forward_new and reverse_new instead.
    %
    %  Function = DERIVATIVE(self, int nfwd, int nadj)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(796, self, varargin{:});
    end
    function varargout = forward_new(self,varargin)
    %FORWARD_NEW Get a function that calculates nfwd forward derivatives.
    %
    %  Function = FORWARD_NEW(self, int nfwd)
    %
    %
    %Returns a function with n_in + n_out + n_in inputs and nfwd outputs. The
    %first n_in inputs correspond to nondifferentiated inputs. The next n_out
    %inputs correspond to nondifferentiated outputs. and the last n_in inputs
    %correspond to forward seeds, stacked horizontally The n_out outputs
    %correspond to forward sensitivities, stacked horizontally. * (n_in = n_in(),
    %n_out = n_out())
    %
    %The functions returned are cached, meaning that if called multiple timed
    %with the same value, then multiple references to the same function will be
    %returned.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(797, self, varargin{:});
    end
    function varargout = forward(self,varargin)
    %FORWARD [DEPRECATED] Use forward_new instead
    %
    %  Function = FORWARD(self, int nfwd)
    %  {{DM}} = FORWARD(self, {DM} arg, {DM} res, {{DM}} fseed, bool always_inline, bool never_inline)
    %    [DEPRECATED] Use Function::factory or jtimes
    %  {SX}} = FORWARD(self, {SX} arg, {SX} res, {SX}} fseed, bool always_inline, bool never_inline)
    %    [DEPRECATED] Use Function::factory or jtimes
    %  {{MX}} = FORWARD(self, {MX} arg, {MX} res, {{MX}} fseed, bool always_inline, bool never_inline)
    %    [DEPRECATED] Use Function::factory or jtimes
    %
    %
    %
    %> FORWARD(self, int nfwd)
    %------------------------------------------------------------------------
    %
    %
    %[DEPRECATED] Use forward_new instead
    %
    %
    %> FORWARD(self, {DM} arg, {DM} res, {{DM}} fseed, bool always_inline, bool never_inline)
    %> FORWARD(self, {SX} arg, {SX} res, {SX}} fseed, bool always_inline, bool never_inline)
    %> FORWARD(self, {MX} arg, {MX} res, {{MX}} fseed, bool always_inline, bool never_inline)
    %------------------------------------------------------------------------
    %
    %
    %[DEPRECATED] Use Function::factory or jtimes
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(798, self, varargin{:});
    end
    function varargout = reverse_new(self,varargin)
    %REVERSE_NEW Get a function that calculates nadj adjoint derivatives.
    %
    %  Function = REVERSE_NEW(self, int nadj)
    %
    %
    %Returns a function with n_in + n_out + n_out inputs and n_in outputs. The
    %first n_in inputs correspond to nondifferentiated inputs. The next n_out
    %inputs correspond to nondifferentiated outputs. and the last n_out inputs
    %correspond to adjoint seeds, stacked horizontally The n_in outputs
    %correspond to adjoint sensitivities, stacked horizontally. * (n_in = n_in(),
    %n_out = n_out())
    %
    %(n_in = n_in(), n_out = n_out())
    %
    %The functions returned are cached, meaning that if called multiple timed
    %with the same value, then multiple references to the same function will be
    %returned.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(799, self, varargin{:});
    end
    function varargout = reverse(self,varargin)
    %REVERSE [DEPRECATED] Use reverse_new instead
    %
    %  Function = REVERSE(self, int nfwd)
    %  {{DM}} = REVERSE(self, {DM} arg, {DM} res, {{DM}} aseed, bool always_inline, bool never_inline)
    %    [DEPRECATED] Use Function::factory or jtimes
    %  {SX}} = REVERSE(self, {SX} arg, {SX} res, {SX}} aseed, bool always_inline, bool never_inline)
    %    [DEPRECATED] Use Function::factory or jtimes
    %  {{MX}} = REVERSE(self, {MX} arg, {MX} res, {{MX}} aseed, bool always_inline, bool never_inline)
    %    [DEPRECATED] Use Function::factory or jtimes
    %
    %
    %
    %> REVERSE(self, int nfwd)
    %------------------------------------------------------------------------
    %
    %
    %[DEPRECATED] Use reverse_new instead
    %
    %
    %> REVERSE(self, {DM} arg, {DM} res, {{DM}} aseed, bool always_inline, bool never_inline)
    %> REVERSE(self, {SX} arg, {SX} res, {SX}} aseed, bool always_inline, bool never_inline)
    %> REVERSE(self, {MX} arg, {MX} res, {{MX}} aseed, bool always_inline, bool never_inline)
    %------------------------------------------------------------------------
    %
    %
    %[DEPRECATED] Use Function::factory or jtimes
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(800, self, varargin{:});
    end
    function varargout = sparsity_jac(self,varargin)
    %SPARSITY_JAC Get, if necessary generate, the sparsity of a Jacobian block
    %
    %  Sparsity = SPARSITY_JAC(self, int iind, int oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, char iind, int oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, int iind, char oind, bool compact, bool symmetric)
    %  Sparsity = SPARSITY_JAC(self, char iind, char oind, bool compact, bool symmetric)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(801, self, varargin{:});
    end
    function varargout = set_jac_sparsity(self,varargin)
    %SET_JAC_SPARSITY Generate the sparsity of a Jacobian block
    %
    %  SET_JAC_SPARSITY(self, Sparsity sp, int iind, int oind, bool compact)
    %  SET_JAC_SPARSITY(self, Sparsity sp, int iind, char oind, bool compact)
    %  SET_JAC_SPARSITY(self, Sparsity sp, char iind, int oind, bool compact)
    %  SET_JAC_SPARSITY(self, Sparsity sp, char iind, char oind, bool compact)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(802, self, varargin{:});
    end
    function varargout = generate(self,varargin)
    %GENERATE Export / Generate C code for the function.
    %
    %  char = GENERATE(self, struct opts)
    %  char = GENERATE(self, char fname, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(803, self, varargin{:});
    end
    function varargout = generate_dependencies(self,varargin)
    %GENERATE_DEPENDENCIES Export / Generate C code for the dependency function.
    %
    %  char = GENERATE_DEPENDENCIES(self, char fname, struct opts)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(804, self, varargin{:});
    end
    function varargout = stats(self,varargin)
    %STATS Get all statistics obtained at the end of the last evaluate call.
    %
    %  struct = STATS(self, int mem)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(805, self, varargin{:});
    end
    function varargout = sx_in(self,varargin)
    %SX_IN Get symbolic primitives equivalent to the input expressions There is no
    %
    %  {SX} = SX_IN(self)
    %  SX = SX_IN(self, int iind)
    %  SX = SX_IN(self, char iname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(806, self, varargin{:});
    end
    function varargout = mx_in(self,varargin)
    %MX_IN Get symbolic primitives equivalent to the input expressions There is no
    %
    %  {MX} = MX_IN(self)
    %  MX = MX_IN(self, int ind)
    %  MX = MX_IN(self, char iname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(807, self, varargin{:});
    end
    function varargout = sx_out(self,varargin)
    %SX_OUT Get symbolic primitives equivalent to the output expressions There is no
    %
    %  {SX} = SX_OUT(self)
    %  SX = SX_OUT(self, int oind)
    %  SX = SX_OUT(self, char oname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(808, self, varargin{:});
    end
    function varargout = mx_out(self,varargin)
    %MX_OUT Get symbolic primitives equivalent to the output expressions There is no
    %
    %  {MX} = MX_OUT(self)
    %  MX = MX_OUT(self, int ind)
    %  MX = MX_OUT(self, char oname)
    %
    %guarantee that subsequent calls return unique answers.
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(809, self, varargin{:});
    end
    function varargout = free_sx(self,varargin)
    %FREE_SX Get all the free variables of the function.
    %
    %  {SX} = FREE_SX(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(810, self, varargin{:});
    end
    function varargout = free_mx(self,varargin)
    %FREE_MX Get all the free variables of the function.
    %
    %  {MX} = FREE_MX(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(811, self, varargin{:});
    end
    function varargout = has_free(self,varargin)
    %HAS_FREE Does the function have free variables.
    %
    %  bool = HAS_FREE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(812, self, varargin{:});
    end
    function varargout = generate_lifted(self,varargin)
    %GENERATE_LIFTED Extract the functions needed for the Lifted Newton method.
    %
    %  [Function OUTPUT, Function OUTPUT] = GENERATE_LIFTED(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(813, self, varargin{:});
    end
    function varargout = getAlgorithmSize(self,varargin)
    %GETALGORITHMSIZE Get the number of atomic operations.
    %
    %  int = GETALGORITHMSIZE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(814, self, varargin{:});
    end
    function varargout = getWorkSize(self,varargin)
    %GETWORKSIZE Get the length of the work vector.
    %
    %  int = GETWORKSIZE(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(815, self, varargin{:});
    end
    function varargout = getAtomicOperation(self,varargin)
    %GETATOMICOPERATION Get an atomic operation operator index.
    %
    %  int = GETATOMICOPERATION(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(816, self, varargin{:});
    end
    function varargout = getAtomicInput(self,varargin)
    %GETATOMICINPUT Get the (integer) input arguments of an atomic operation.
    %
    %  [int,int] = GETATOMICINPUT(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(817, self, varargin{:});
    end
    function varargout = getAtomicInputReal(self,varargin)
    %GETATOMICINPUTREAL Get the floating point output argument of an atomic operation.
    %
    %  double = GETATOMICINPUTREAL(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(818, self, varargin{:});
    end
    function varargout = getAtomicOutput(self,varargin)
    %GETATOMICOUTPUT Get the (integer) output argument of an atomic operation.
    %
    %  int = GETATOMICOUTPUT(self, int k)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(819, self, varargin{:});
    end
    function varargout = n_nodes(self,varargin)
    %N_NODES Number of nodes in the algorithm.
    %
    %  int = N_NODES(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(820, self, varargin{:});
    end
    function varargout = spCanEvaluate(self,varargin)
    %SPCANEVALUATE [INTERNAL]  Is the class able to propagate seeds through the algorithm?
    %
    %  bool = SPCANEVALUATE(self, bool fwd)
    %
    %
    %(for usage, see the example propagating_sparsity.cpp)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(821, self, varargin{:});
    end
    function varargout = sz_arg(self,varargin)
    %SZ_ARG [INTERNAL]  Get required length of arg field.
    %
    %  size_t = SZ_ARG(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(822, self, varargin{:});
    end
    function varargout = sz_res(self,varargin)
    %SZ_RES [INTERNAL]  Get required length of res field.
    %
    %  size_t = SZ_RES(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(823, self, varargin{:});
    end
    function varargout = sz_iw(self,varargin)
    %SZ_IW [INTERNAL]  Get required length of iw field.
    %
    %  size_t = SZ_IW(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(824, self, varargin{:});
    end
    function varargout = sz_w(self,varargin)
    %SZ_W [INTERNAL]  Get required length of w field.
    %
    %  size_t = SZ_W(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(825, self, varargin{:});
    end
    function varargout = checkInputs(self,varargin)
    %CHECKINPUTS [INTERNAL]  Check if the numerical values of the supplied bounds make sense.
    %
    %  CHECKINPUTS(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(826, self, varargin{:});
    end
    function varargout = name(self,varargin)
    %NAME Name of the function.
    %
    %  char = NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(827, self, varargin{:});
    end
    function varargout = type_name(self,varargin)
    %TYPE_NAME Get type name.
    %
    %  char = TYPE_NAME(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(828, self, varargin{:});
    end
    function varargout = is_a(self,varargin)
    %IS_A Check if the function is of a particular type Optionally check if name
    %
    %  bool = IS_A(self, char type, bool recursive)
    %
    %matches one of the base classes (default true)
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(829, self, varargin{:});
    end
    function varargout = assert_size_in(self,varargin)
    %ASSERT_SIZE_IN Assert that an input dimension is equal so some given value.
    %
    %  ASSERT_SIZE_IN(self, int i, int nrow, int ncol)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(832, self, varargin{:});
    end
    function varargout = assert_size_out(self,varargin)
    %ASSERT_SIZE_OUT Assert that an output dimension is equal so some given value.
    %
    %  ASSERT_SIZE_OUT(self, int i, int nrow, int ncol)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(833, self, varargin{:});
    end
    function varargout = checkout(self,varargin)
    %CHECKOUT Checkout a memory object.
    %
    %  int = CHECKOUT(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(834, self, varargin{:});
    end
    function varargout = release(self,varargin)
    %RELEASE Release a memory object.
    %
    %  RELEASE(self, int mem)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(835, self, varargin{:});
    end
    function varargout = get_function(self,varargin)
    %GET_FUNCTION 
    %
    %  [char] = GET_FUNCTION(self)
    %  Function = GET_FUNCTION(self, char name)
    %
    %
      [varargout{1:nargout}] = casadiMEX(836, self, varargin{:});
    end
    function varargout = has_function(self,varargin)
    %HAS_FUNCTION 
    %
    %  bool = HAS_FUNCTION(self, char fname)
    %
    %
      [varargout{1:nargout}] = casadiMEX(837, self, varargin{:});
    end
    function varargout = rootfinder_fun(self,varargin)
    %ROOTFINDER_FUN Access rhs function for a rootfinder.
    %
    %  Function = ROOTFINDER_FUN(self)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(838, self, varargin{:});
    end
    function varargout = integrator_dae(self,varargin)
    %INTEGRATOR_DAE [DEPRECATED] Get the DAE for an integrator To generate a function with the
    %
    %  Function = INTEGRATOR_DAE(self)
    %
    %legacy syntax: oracle().factory("f", {"x", "z", "p", "t"},
    %{"ode", "alg", "quad"})
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(839, self, varargin{:});
    end
    function varargout = conic_debug(self,varargin)
    %CONIC_DEBUG Generate native code in the interfaced language for debugging
    %
    %  CONIC_DEBUG(self, std::ostream & file)
    %  CONIC_DEBUG(self, char filename)
    %
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(840, self, varargin{:});
    end

    function varargout = subsref(self,s)
      if numel(s)==1 && strcmp(s.type,'()')
        [varargout{1:nargout}]= paren(self, s.subs{:});
      else
        [varargout{1:nargout}] = builtin('subsref',self,s);
      end
   end
   function varargout = paren(self, varargin)
      if nargin==1 || (nargin>=2 && ischar(varargin{1}))
        % Named inputs: return struct
        assert(nargout<2, 'Syntax error');
        assert(mod(nargin,2)==1, 'Syntax error');
        arg = struct;
        for i=1:2:nargin-1
          assert(ischar(varargin{i}), 'Syntax error');
          arg.(varargin{i}) = varargin{i+1};
        end
        res = self.call(arg);
        varargout{1} = res;
      else
        % Ordered inputs: return variable number of outputs
        res = self.call(varargin);
        assert(nargout<=numel(res), 'Too many outputs');
        for i=1:max(min(1,numel(res)),nargout)
          varargout{i} = res{i};
        end
      end
    end
      function self = Function(varargin)
    %FUNCTION 
    %
    %  new_obj = FUNCTION()
    %    Default constructor, null pointer.
    %  new_obj = FUNCTION(char fname)
    %    Construct from a file.
    %  new_obj = FUNCTION(char name, {SX} arg, {SX} res, struct opts)
    %    Construct an SX function.
    %  new_obj = FUNCTION(char name, {MX} arg, {MX} res, struct opts)
    %    Construct an MX function.
    %  new_obj = FUNCTION(char name, struct:SX dict, [char] argn, [char] resn, struct opts)
    %    Construct an SX function.
    %  new_obj = FUNCTION(char name, struct:MX dict, [char] argn, [char] resn, struct opts)
    %    Construct an MX function.
    %  new_obj = FUNCTION(char name, {SX} arg, {SX} res, [char] argn, [char] resn, struct opts)
    %    Construct an SX function.
    %  new_obj = FUNCTION(char name, {MX} arg, {MX} res, [char] argn, [char] resn, struct opts)
    %    Construct an MX function.
    %
    %> FUNCTION()
    %------------------------------------------------------------------------
    %
    %
    %Default constructor, null pointer.
    %
    %
    %> FUNCTION(char fname)
    %------------------------------------------------------------------------
    %
    %
    %Construct from a file.
    %
    %
    %> FUNCTION(char name, {SX} arg, {SX} res, struct opts)
    %> FUNCTION(char name, struct:SX dict, [char] argn, [char] resn, struct opts)
    %> FUNCTION(char name, {SX} arg, {SX} res, [char] argn, [char] resn, struct opts)
    %------------------------------------------------------------------------
    %
    %
    %Construct an SX function.
    %
    %
    %> FUNCTION(char name, {MX} arg, {MX} res, struct opts)
    %> FUNCTION(char name, struct:MX dict, [char] argn, [char] resn, struct opts)
    %> FUNCTION(char name, {MX} arg, {MX} res, [char] argn, [char] resn, struct opts)
    %------------------------------------------------------------------------
    %
    %
    %Construct an MX function.
    %
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(841, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
  end
  methods(Static)
    function varargout = conditional(varargin)
    %CONDITIONAL 
    %
    %  Function = CONDITIONAL(char name, {Function} f, Function f_def, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(793, varargin{:});
    end
    function varargout = if_else(varargin)
    %IF_ELSE 
    %
    %  Function = IF_ELSE(char name, Function f_true, Function f_false, struct opts)
    %
    %
     [varargout{1:nargout}] = casadiMEX(794, varargin{:});
    end
    function varargout = check_name(varargin)
    %CHECK_NAME 
    %
    %  bool = CHECK_NAME(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(830, varargin{:});
    end
    function varargout = fix_name(varargin)
    %FIX_NAME 
    %
    %  char = FIX_NAME(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(831, varargin{:});
    end
  end
end
