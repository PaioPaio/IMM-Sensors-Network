%% output: (x-> position + speed; a->acceleration)->c; s1->new state
%% input:  y-> position + speed; y1->acceleration magnitude; s-> state of markov chain;
%%         T-> transition Matrix 

function [c, s1]=markovmove(y,y1,s,T,delta)
    A=[1 0 delta 0;
           0 1 0 delta;
           0 0 1 0
           0 0 0 1];           %state matrix
    B=[delta^2/2 0;
            0 delta^2/2
            delta 0
            0 delta];          %input matrix (for acceleration
    s1=markchainez(s,T);
        switch s1
            case 1
                %constant speed, no change of direction
                x=A*y(1:4);
                a=[0;0];
            case 2
                %constant acceleration, no change of direction
                a=(y1/sqrt(y(1)^2+y(2)^2))*[y(1);y(2)];    %defining acceleration with same direction
                x=A*y(1:4)+B*a;
            case 3
                %change of direction through acceleration
                ct=2*rand()-1;   %cosine of theta, used to determine in which way the guy turns 
                st=randbool()*sqrt(1-ct^2);
                a=y1*[ct; st];
                x=A*y(1:4)+B*a;
            case 4
                %change of direction just to make things chaotic
                ct=2*rand()-1;   %cosine of theta, used to determine in which way the guy turns 
                st=randbool()*sqrt(1-ct^2);
                a=[0;0];
                x=y(1:4);
                x(3:4)=sqrt(x(3)^2+x(4)^2)*[ct;st];
        end
        c=[x;a];
end