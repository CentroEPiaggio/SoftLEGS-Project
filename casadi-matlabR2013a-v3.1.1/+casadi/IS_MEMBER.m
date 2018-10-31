function v = IS_MEMBER()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 94);
  end
  v = vInitialized;
end
