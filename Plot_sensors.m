%% plot the Sensor
for i=1:n-2 
for u=1:size(Pos_sens_on,2)
        if size(Pos_sens_on{i+1,u}) == [0,0];
            Pos_sens_on{i+1,u} = Pos_sens_on{i,u};
            Pos_sens_grid{i+1,u} =Pos_sens_grid{i+1,u};
        else
        end
 end
end