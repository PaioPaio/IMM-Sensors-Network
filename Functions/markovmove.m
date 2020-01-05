%% output: (x-> position + speed)->c; s->next markov state;
%% input:  y-> position + speed + ; a->acceleration magnitude; s-> state of markov chain, Mat->cell array of all B matrices;
%%         T-> transition Matrix 

function [c, s, x1]=markovmove(Mat,y,a,s,T)    %y1->no noise
        x=Mat{s,1}*y+Mat{s,2}*a+[0;0;0;0;randn(2,1).*0.02];  %the real one (noise on acceleration)
        x1=Mat{s,1}*y+Mat{s,2}*a;                           %the predicted one
        s=markchange(s,T);
        c=x;
end