function x = tellyourfriends(sensor,grid,message)
%x->fake output cause matlab is stupid
%sensor->Sensor object, grid->cell array of sensors, message->string
%this function sends a message to every sensor in the neighboor of the one
%that has changed it's state. 
%The change can be either from ON->IDLE ("Cannot") or from IDLE->ON ("Can")
%the changes that involve OFF aren't useful to our case
if message=="Can"
    amici=sensor.neighboors;
    n=length(amici);
    for k=1:n
        gigi=amici{k};
        grid{gigi(1),gigi(2)}.change=1;
        grid{gigi(1),gigi(2)}.checkchange;
    end
elseif message=="Cannot"
    amici=sensor.neighboors;
    n=length(amici);
    for k=1:n
        gigi=amici{k};
        grid{gigi(1),gigi(2)}.change=-1;
        grid{gigi(1),gigi(2)}.checkchange;
    end
end

end

