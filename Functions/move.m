%% output: x1-> position + speed; s->next markov state;
%% input:  x-> position + speed + ; a->acceleration magnitude; s-> state of markov chain, Mat->cell array of all B matrices;
%%         T-> transition Matrix 

function x1=move(Mat,x,a,s,Q)    %y1->no noise
        x1=Mat{s,1}*x+Mat{s,2}*a+Mat{s,3}*randn(size(Mat{s,1},1),1).*arrayfun(@(a)sqrt(a),diag(Q)); 
end