%% funzione che returna o 1 o -1
function x=randbool()
    x=single(2*int32(randn(1)>0)-1);
end