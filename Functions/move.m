%% output: x1-> position + speed; s->next markov state;
%% input:  x-> position + speed + ; a->acceleration magnitude; s-> state of markov chain, Mat->cell array of all B matrices;
%%         T-> transition Matrix 

function x1=move(x,a,s,Q,caso,delta,Mat)
    %Mat is ABG in linear case (1), radius in non linear
switch caso
    case 1
        x1=Mat{s,1}*x+Mat{s,2}*a+Mat{s,3}*(randn(size(Mat{s,3},2),1).*arrayfun(@(a)sqrt(a),diag(Q))); 
    case 2
        A=[1 0 delta*cos(x(4)) 0;
            0 1 delta*sin(x(4)) 0;
            0 0 1 0
            0 0 0 1];
        B={[0 0 0 0;0 0 0 0]',[delta^2/2*cos(x(4))*Mat delta^2/2*sin(x(4))*Mat delta*Mat 0;0 0 0 0]',[-delta^2/2*cos(x(4))*Mat -delta^2/2*sin(x(4))*Mat delta*Mat 0;0 0 0 0]',[0 0 0 0;0 0 0 delta]',[0 0 0 0;0 0 0 -delta]'};
        G=[delta^2/2*cos(x(4))*Mat delta^2/2*sin(x(4))*Mat delta*Mat 0; 0 0 0 delta]';
        x1=A*x+B{s}*a+G*(randn(size(G,2),1).*arrayfun(@(a)sqrt(a),diag(Q)));
end
end