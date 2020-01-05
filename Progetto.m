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

AB={A,[0;0;0;0;0;0];A,[0;0;0;0;0;1];A,[0;0;0;0;1;0];A,[0;0;0;0;0;-1];A,[0;0;0;0;-1;0]};
    case 2
end

%Transition Matrix of markov chain

%number of states
ns=length(AB);

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
sigma=diag([0.1/3,2*pi/360]);
sensoreprova=Sensor([0,0],range,sigma);




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
mode=zeros(1,10);
x=x0;
x1=x0;
H=[1,1;1,1];



while sensoreprova.inrange
    [x, s, x1]=markovmove(AB,x,acc,s,Transmat);
    stato(:,n)=x;
    statopredicted(:,n)=x1;
    sensoreprova.sense(stato(1:2,n));
    mode(n)=s;
    if sensoreprova.sensed(1)==99999 && sensoreprova.sensed(2)==99999
        break
    else
        positionsensed(:,n)=[cos(sensoreprova.sensed(2));sin(sensoreprova.sensed(2))].*sensoreprova.sensed(1);
    end
    %H has to be linearized and computed each step since z=f(x) and not z=H*x
    H=[positionsensed(:,n)'/sensoreprova.sensed(1);...
       -positionsensed(2,n)/sensoreprova.sensed(1)^2,positionsensed(1,n)/sensoreprova.sensed(1)^2 ]
    
    
    
    n=n+1;
end


figure(1)
plot(stato(1,1:n),stato(2,1:n))
hold on
plot(positionsensed(1,1:n-1),positionsensed(2,1:n-1),'*')