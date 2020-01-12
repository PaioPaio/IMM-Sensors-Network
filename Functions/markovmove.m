%% output: x1-> position + speed; s->next markov state;
%% input:  x-> position + speed + ; a->acceleration magnitude; s-> state of markov chain, Mat->cell array of all B matrices;
%%         T-> transition Matrix 

function [x1, s]=markovmove(Mat,x,a,s,T,Q)    %y1->no noise
        x1=Mat{s,1}*x+Mat{s,2}*a+Mat{s,3}*randn(6,1).*diag(Q);  %the real one (noise on acceleration)
        s=markchange(s,T);
end