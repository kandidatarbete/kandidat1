%To shift our indexes the right way we use X*circShiftMatrix(X.length)

function ret = circShiftMatrix (n)
    
    ret= circshift(eye(n),1);
    ret(1,n)=0;


end