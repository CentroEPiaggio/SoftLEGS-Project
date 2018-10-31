classdef CasadiMeta < SwigRef
    %CASADIMETA Collects global CasADi meta information.
    %
    %
    %
    %Joris Gillis
    %
    %C++ includes: casadi_meta.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function self = CasadiMeta(varargin)
    %CASADIMETA 
    %
    %  new_obj = CASADIMETA()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(968, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        casadiMEX(969, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = getVersion(varargin)
    %GETVERSION 
    %
    %  char = GETVERSION()
    %
    %
     [varargout{1:nargout}] = casadiMEX(957, varargin{:});
    end
    function varargout = getGitRevision(varargin)
    %GETGITREVISION 
    %
    %  char = GETGITREVISION()
    %
    %
     [varargout{1:nargout}] = casadiMEX(958, varargin{:});
    end
    function varargout = getGitDescribe(varargin)
    %GETGITDESCRIBE 
    %
    %  char = GETGITDESCRIBE()
    %
    %
     [varargout{1:nargout}] = casadiMEX(959, varargin{:});
    end
    function varargout = getFeatureList(varargin)
    %GETFEATURELIST 
    %
    %  char = GETFEATURELIST()
    %
    %
     [varargout{1:nargout}] = casadiMEX(960, varargin{:});
    end
    function varargout = getBuildType(varargin)
    %GETBUILDTYPE 
    %
    %  char = GETBUILDTYPE()
    %
    %
     [varargout{1:nargout}] = casadiMEX(961, varargin{:});
    end
    function varargout = getCompilerId(varargin)
    %GETCOMPILERID 
    %
    %  char = GETCOMPILERID()
    %
    %
     [varargout{1:nargout}] = casadiMEX(962, varargin{:});
    end
    function varargout = getCompiler(varargin)
    %GETCOMPILER 
    %
    %  char = GETCOMPILER()
    %
    %
     [varargout{1:nargout}] = casadiMEX(963, varargin{:});
    end
    function varargout = getCompilerFlags(varargin)
    %GETCOMPILERFLAGS 
    %
    %  char = GETCOMPILERFLAGS()
    %
    %
     [varargout{1:nargout}] = casadiMEX(964, varargin{:});
    end
    function varargout = getModules(varargin)
    %GETMODULES 
    %
    %  char = GETMODULES()
    %
    %
     [varargout{1:nargout}] = casadiMEX(965, varargin{:});
    end
    function varargout = getPlugins(varargin)
    %GETPLUGINS 
    %
    %  char = GETPLUGINS()
    %
    %
     [varargout{1:nargout}] = casadiMEX(966, varargin{:});
    end
    function varargout = getInstallPrefix(varargin)
    %GETINSTALLPREFIX 
    %
    %  char = GETINSTALLPREFIX()
    %
    %
     [varargout{1:nargout}] = casadiMEX(967, varargin{:});
    end
  end
end
