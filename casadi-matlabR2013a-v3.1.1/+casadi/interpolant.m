function varargout = interpolant(varargin)
    %INTERPOLANT An interpolant function for lookup table data
    %
    %  Function = INTERPOLANT(char name, char solver, [[double]] grid, [double] values, struct opts)
    %
    %
    %General information
    %===================
    %
    %
    %
    %List of plugins
    %===============
    %
    %
    %
    %- linear
    %
    %Note: some of the plugins in this list might not be available on your
    %system. Also, there might be extra plugins available to you that are not
    %listed here. You can obtain their documentation with
    %Interpolant.doc("myextraplugin")
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %linear
    %------
    %
    %
    %
    %
    %
    %--------------------------------------------------------------------------------
    %
    %
    %
    %Joel Andersson
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(898, varargin{:});
end
