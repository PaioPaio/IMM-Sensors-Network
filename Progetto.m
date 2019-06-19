clc; clear all; close all;
%% Dati

% state 1 -> walk constant speed, constant direction (high probability of
%            staying like that)
%
% state 2 -> walk accelerating, constant direction
%
% state 3 -> walk accelerating, change direction

%Transition Matrix of markov chain
% TransMatrix=[0.5 0.3 0.2;
%              0.2 0.3 0.5;
%              0.5 0.35 0.15];
TransMatrix=[0.45 0.3 0.2 0.05;
             0.3 0.35 0.3 0.05;
             0.5 0.35 0.125 0.025;
             0.5 0.25 0.225 0.025];
%delta time
deltaT=0.3;
%initial state of markov chain
s=1;
%initial 2D position, velocity and acceleration
x=[0 0 -1 3 0 0]';
%magnitude of acceleration
x1=1;
%switch for the type of movement
%1-> markov chain constant acceleration
%2-> markov chain random acceleration
%3-> unicycle
caso=1;

n=500;
persona=zeros(6,n);
stati=zeros(1,n);

switch caso
    case 1
        for i=1:n
            [x, s]=markovmove(x,x1,s,TransMatrix,deltaT);
            persona(:,i)=x;
            stati(1,i)=s;
        end
    case 2
        
        
    case 3
        
        
end