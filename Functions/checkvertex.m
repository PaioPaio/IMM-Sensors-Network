function A = checkvertex(cellarr,arr)
%check if in the cell array cellarr there's a match for the array arr, if
%arr is actually a cell then it checks for all of the arrays inside.
%optionally outputs out also the indices in which there was a match
a=length(cellarr);
if iscell(arr)
    b=length(arr);
    A=zeros(a,b);
    for i=1:b
        arr1=arr{i};
        for k=1:a
            if cellarr{k}==arr1
                A(k,i)=1;
            end
        end
    end
else
    A=zeros(1,a);
    for i=1:size(cellarr,1)
        for j=1:size(cellarr,2)
            if cellarr{i,j}==arr
                A(i,j)=1;
            end
        end
    end
end
end

