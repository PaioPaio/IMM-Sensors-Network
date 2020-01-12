clc; clear all; close all;
%% Dati


%switch for the type of movement
%1-> markov chain constant speed or costant acceleration
%2-> unicycle

caso=1;
%delta time
delta=0.1;

%Cell matrix of various B for the input
switch caso
    case 1
A=[1 0 delta 0 delta^2/2 0;
   0 1 0 delta 0 delta^2/2;
   0 0 1 0 delta 0;
   0 0 0 1 0 delta;
   0 0 0 0 0 0;
   0 0 0 0 0 0];
G=eye(6);       %better change this to something that makes sense

ABG={A,[0;0;0;0;0;0],G;A,[0;0;0;0;0;1],G;A,[0;0;0;0;1;0],G;A,[0;0;0;0;0;-1],G;A,[0;0;0;0;-1;0],G};
    case 2
end

%Transition Matrix of markov chain

%number of states
ns=length(ABG);

%case in which you are costant velocity
p1=0.8;     %probability of constant speed while being in constant speed
q1=0.7;     %probability of still accelerating forward while accelerating forward
p=(1-p1)/4;
q=(1-q1)/6;
v1=[p1 p p p p];
v2=[q1 q 2*q q];

%the matrix is "circulant" (trova termine migliore) so we can build it
%algorithmically
Transmat=zeros(ns);
Transmat(1,:)=v1;
Transmat(2:end,1)=ones(4,1).*0.1;
for i=2:ns
    Transmat(i,2:end)=[v2(end+3-i:end),v2(1:end+2-i)];
end


%Senosors grid
range=50;
R=diag([0.1/3,2*pi/360]);               %sensor covariance
sensoreprova=Sensor([0,0],range,R);



%Power spectra density of process noise
Q=diag([0.01,0.01,0.02,0.02,0.05,0.05]);
%initial state of markov chain
s=1;
%initial 2D position, velocity and acceleration
x0=[0 0 0 1 0 0]';
%magnitude of acceleration
acc=0.4;
%magnitude of uniform speed
% vel1=1;

n=1;
stato=zeros(6,10);
statopredicted=zeros(6,10);
positionsensed=zeros(2,10);
state
mode=zeros(1,10);
x=x0;
x1=x0;
H=[1,1;1,1];

%initialize sensor Grid

while sensoreprova.inrange
    [x, s, x1]=markovmove(ABG,x,acc,s,Transmat,Q);
    stato(:,n)=x;
    sensoreprova.sense(stato(1:2,n));
    mode(n)=s;
    if sensoreprova.sensed(1)==99999 && sensoreprova.sensed(2)==99999
        break
    else
        positionsensed(:,n)=[cos(sensoreprova.sensed(2));sin(sensoreprova.sensed(2))].*sensoreprova.sensed(1);
    end
    
    
    
    n=n+1;
end


figure(1)
plot(stato(1,1:n),stato(2,1:n))
hold on
plot(positionsensed(1,1:n-1),positionsensed(2,1:n-1),'*')
trackingIMM