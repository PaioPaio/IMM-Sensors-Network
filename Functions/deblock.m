function H = deblock(Hbig,blocksize)
%gets a diagonal matrix(or block diagonal) and outputs a matrix of stacked
%blocks of size blocksize
    len=size(Hbig,1);
    len1=len/blocksize;
    H=Hbig(1:blocksize,1:blocksize);
    for i=1:(len1-1)
        indices=blocksize*i+1:blocksize*(i+1);
        H(indices,1:blocksize)=Hbig(indices,indices);
    end
end

