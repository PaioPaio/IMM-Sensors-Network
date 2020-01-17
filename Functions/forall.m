function Arrayofproperty = forall(cellofclass)
%outputs an array of inrange for a cell of sensors
a=size(cellofclass,1);
b=size(cellofclass,2);
for i=1:a
    for j=1:b
        Arrayofproperty(i,j)=cellofclass{i,j}.inrange;
    end
end
end

