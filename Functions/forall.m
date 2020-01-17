function Arrayofproperty = forall(cellofclass,property)
%outputs an array of the property specified (doesnt work if properties are
%arrays or cells, no need to implement)
a=size(cellofclass,1);
b=size(cellofclass,2);
for i=1:a
    for j=1:b
        Arrayofproperty(i,j)=get(cellofclass{i,j},property);
    end
end
end

