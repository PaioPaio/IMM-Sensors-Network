function varargout = checkvertex(cellarr,arr)
%check if in the cell array cellarr there's a match for the array arr, if
%arr is actually a cell then it checks for all of the arrays inside.
%optionally outputs out also the indices in which there was a match
nOutputs = nargout;
varargout = cell(1,nOutputs);
a=size(cellarr,1);
b=size(cellarr,2);
if iscell(arr)
    indic={};
    c=length(arr);
    A=zeros(a,b,c);
    for i=1:c
        arr1=arr{i};
        for k=1:size(cellarr,1)
            for j=1:size(cellarr,2)     
                if cellarr{k,j}==arr1
                    A(i,k,j)=1;
                    indic{end+1}=[k,j];
                end
            end
        end
    end
else
    indic={};
    A=zeros(a,b);
    for i=1:size(cellarr,1)
        for j=1:size(cellarr,2)     
            if cellarr{i,j}==arr
                A(i,j)=1;
                indic{end+1}=[i,j];
            end
        end
    end
end
varargout{1}=A;
if nOutputs==2
    varargout{2}=indic;
end
end
