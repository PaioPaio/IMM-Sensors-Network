function A = checkvertex(cellarr,arr)
a=size(cellarr,1);
b=size(cellarr,2);
A=zeros(a,b);
for i=1:size(cellarr,1)
    for j=1:size(cellarr,2)     
        if cellarr{i,j}==arr
            A(i,j)=1;
        end
    end
end
end

