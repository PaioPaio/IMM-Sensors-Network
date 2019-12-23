clc; clear all; close all;
%% Dati

% states -> all the cardinal directions (including the hybrid ones ex.
% South-west) both with constant speed and constant acceleration

%Transition Matrix of markov chain
p=18/107;
q=15/116;
v1=[0.4 p/3 p/10 0 0 0 p/10 p/3 p p/3 p/6 p/10 p/2 p/10 p/6 p/3];
v2=[0.25 q q/2 0 0.15 0 q/2 q 0.1 q/3 q/10 0 0 0 q/10 q/3];


%the matrix is "circulant" (trova termine migliore) so we can build it
%programmatically
Transmat=zeros(16);
for i=1:16
    if i<=8
        Transmat(i,:)=[v1(end+2-i:end),v1(1:end+1-i)];
    else
        Transmat(i,:)=[v2(end+2-i:end),v2(1:end+1-i)];
    end
end

%%


%delta time
deltaT=0.3;
%initial state of markov chain
s=1;
%initial 2D position, velocity and acceleration
x0=[0 0 0 1 0 0]';
%magnitude of acceleration
x1=1;
%magnitude of uniform speed
vel1=1;

%switch for the type of movement
%1-> markov chain uniform speed or costant acceleration
%2-> unicycle
caso=1;

n=500;
persona=zeros(6,n);
stati=zeros(1,n);
x=x0;
switch caso
    case 1
        for i=1:n
            [x, s]=markovmove(x,x1,vel1,s,Transmat,deltaT);
            persona(:,i)=x;
            stati(i)=s;
        end
    case 2
        
        
end