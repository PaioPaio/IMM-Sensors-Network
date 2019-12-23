%% funzione per implementare la markov chain
%% s è lo stato e t è la matrice di transizione

%prende T, fa la cumulata della riga dello stato a cui sono, guarda se un
%numero preso a caso fra 0 e 1 è minore della cumulata e se lo è va a
%prendere la colonna risultante -> quello sarà il nostro nuovo stato
function s1=markchange(s,T)
    statedistr=T(s,:);
    cumulative=cumsum(statedistr);
    statetrans=cumulative>=rand();
    s1=find(statetrans,1);
end