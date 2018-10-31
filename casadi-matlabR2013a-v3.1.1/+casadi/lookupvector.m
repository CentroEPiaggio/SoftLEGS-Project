function varargout = lookupvector(varargin)
    %LOOKUPVECTOR Returns a vector for quickly looking up entries of supplied list.
    %
    %  [int] = LOOKUPVECTOR([int] v, int size)
    %
    %
    %lookupvector[i]!=-1 <=> v contains i v[lookupvector[i]] == i <=> v contains
    %i
    %
    %Duplicates are treated by looking up last occurrence
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(57, varargin{:});
end
