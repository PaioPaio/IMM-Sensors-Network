%% output: (x-> position + speed)->c; s->next markov state;
%% input:  y-> position + speed + ; a->acceleration magnitude; s-> state of markov chain, vel->magnitude of constant inizial speed;
%%         T-> transition Matrix 

function [c, s]=markovmove(y,a,vel,s,T,delta)
    A=[1 0 delta 0 delta^2/2 0;
           0 1 0 delta 0 delta^2/2;
           0 0 1 0 delta 0;
           0 0 0 1 0 delta;
           0 0 0 0 0 0;
           0 0 0 0 0 0];           %state matrix
        switch s
            case 1
                %constant speed, north direction
                y=[y(1:2);0;vel;0;0];      %if we move in constantspeed we conserve the position but we reset the speed
                B=[0 0 0 0 0 0]';          %input matrix for no acceleration
                
            case 2
                %constant speed, n-e direction
                y=[y(1:2);vel/sqrt(2);vel/sqrt(2);0;0];
                B=[0 0 0 0 0 0]';
                
            case 3
                %constant speed, east direction
                y=[y(1:2);vel;0;0;0];
                B=[0 0 0 0 0 0]';
                
            case 4
                %constant speed, s-e direction
                y=[y(1:2);vel/sqrt(2);-vel/sqrt(2);0;0];
                B=[0 0 0 0 0 0]';   
                
            case 5
                %constant speed, south direction
                y=[y(1:2);0;-vel;0;0];
                B=[0 0 0 0 0 0]';  
                
            case 6
                %constant speed, s-w direction
                y=[y(1:2);-vel/sqrt(2);-vel/sqrt(2);0;0];
                B=[0 0 0 0 0 0]';  
                
            case 7
                %constant speed, west direction
                y=[y(1:2);-vel;0;0;0];
                B=[0 0 0 0 0 0]';                
                
            case 8
                %constant speed, n-w direction
                y=[y(1:2);-vel/sqrt(2);vel/sqrt(2);0;0];
                B=[0 0 0 0 0 0]';                
                
            case 9
                %constant acceleration, north direction
                %here we have no need of resetting the speed, we just need
                %to impose the direction of the acceleration
                B=[0 0 0 0 0 1]';
                
            case 10
                %constant acceleration, n-e direction
                B=[0 0 0 0 1 1]'./sqrt(2);
                
            case 11
                %constant acceleration, east direction
                B=[0 0 0 0 1 0]';
                
            case 12
                %constant acceleration, s-e direction
                B=[0 0 0 0 1 -1]'./sqrt(2);
                
            case 13
                %constant acceleration, south direction
                B=[0 0 0 0 0 -1]';
                
            case 14
                %constant acceleration, s-w direction
                B=[0 0 0 0 -1 -1]'./sqrt(2);
                
            case 15
                %constant acceleration, west direction
                B=[0 0 0 0 -1 0]';
                
            case 16
                %constant acceleration, n-w direction
                B=[0 0 0 0 -1 1]'./sqrt(2);
                
        end
        x=A*y+B*a;
        s=markchange(s,T);
        c=x;
end