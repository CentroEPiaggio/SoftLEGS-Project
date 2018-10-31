function varargout = linsol_new(varargin)
    %LINSOL_NEW [DEPRECATED] Create a linear solver (legacy syntax, use Linsol constructor
    %
    %  Function = LINSOL_NEW(char name, char solver, Sparsity sp, int nrhs, struct opts)
    %
    %instead)
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(886, varargin{:});
end
