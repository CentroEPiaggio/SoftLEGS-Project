classdef Variable < casadi.PrintVariable
    %VARIABLE 
    %
    %   = VARIABLE()
    %
    %
  methods
    function varargout = name(self,varargin)
    %NAME 
    %
    %  char = NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(990, self, varargin{:});
    end
    function v = v(self)
      v = casadiMEX(991, self);
    end
    function v = d(self)
      v = casadiMEX(992, self);
    end
    function v = nominal(self)
      v = casadiMEX(993, self);
    end
    function v = start(self)
      v = casadiMEX(994, self);
    end
    function v = min(self)
      v = casadiMEX(995, self);
    end
    function v = max(self)
      v = casadiMEX(996, self);
    end
    function v = guess(self)
      v = casadiMEX(997, self);
    end
    function v = derivative_start(self)
      v = casadiMEX(998, self);
    end
    function v = variability(self)
      v = casadiMEX(999, self);
    end
    function v = causality(self)
      v = casadiMEX(1000, self);
    end
    function v = category(self)
      v = casadiMEX(1001, self);
    end
    function v = alias(self)
      v = casadiMEX(1002, self);
    end
    function v = description(self)
      v = casadiMEX(1003, self);
    end
    function v = valueReference(self)
      v = casadiMEX(1004, self);
    end
    function v = unit(self)
      v = casadiMEX(1005, self);
    end
    function v = display_unit(self)
      v = casadiMEX(1006, self);
    end
    function v = free(self)
      v = casadiMEX(1007, self);
    end
    function varargout = print(self,varargin)
    %PRINT 
    %
    %  PRINT(self, bool trailing_newline)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1008, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP 
    %
    %  DISP(self, bool trailing_newline)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1009, self, varargin{:});
    end
    function self = Variable(varargin)
    %VARIABLE 
    %
    %  new_obj = VARIABLE()
    %  new_obj = VARIABLE(char name, Sparsity sp)
    %
    %
      self@casadi.PrintVariable(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1010, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        casadiMEX(1011, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
