clc; clear all; close all;
%% Dati

% states -> all the cardinal directions (including the hybrid ones ex.
% South-west) both with constant speed and constant acceleration, going
% clockwise starting from north

%Cell matrix of various B for the input

B={[0;0;0;0;0;0],[0;0;0;0;0;1],[0;0;0;0;1;0],[0;0;0;0;0;-1],[0;0;0;0;-1;0]};

%Transition Matrix of markov chain

%number of states
ns=length(B);

%case in which you are costant velocity
p1=0.8;     %probability of constant speed while being in constant speed
q1=0.7;     %probability of still accelerating forward while accelerating forward
p=(1-p1)/4;
q=(1-q1)/6;
v1=[p1 p p p p];
v2=[q1 q 2*q q];


%caso con 16
% p=(1-p1)*30/107;
% q=15/116;
% v1=[p1 p/3 p/10 0 0 0 p/10 p/3 p p/3 p/6 p/10 p/2 p/10 p/6 p/3];
% v2=[0.25 q q/2 0 0.15 0 q/2 q 0.1 q/3 q/10 0 0 0 q/10 q/3];


%the matrix is "circulant" (trova termine migliore) so we can build it
%algorithmically
Transmat=zeros(ns);
Transmat(1,:)=v1;
Transmat(2:end,1)=ones(4,1).*0.1;
for i=2:ns
    Transmat(i,2:end)=[v2(end+3-i:end),v2(1:end+2-i)];
end





%delta time
deltaT=0.1;
%initial state of markov chain
s=1;
%initial 2D position, velocity and acceleration
x0=[0 0 0 1 0 0]';
%magnitude of acceleration
acc=0.4;
%magnitude of uniform speed
% vel1=1;

%switch for the type of movement
%1-> markov chain uniform speed or costant acceleration
%2-> unicycle
caso=1;

n=500;
stato=zeros(6,n);
statononoise=zeros(6,n);
mode=zeros(1,n);
x=x0;
x1=x0;
switch caso
    case 1
        for i=1:n
            [x, s, x1]=markovmove(B,x,acc,s,Transmat,deltaT,x1);
            stato(:,i)=x;
            statononoise(:,i)=x1;
            mode(i)=s;
        end
    case 2
        
        
end