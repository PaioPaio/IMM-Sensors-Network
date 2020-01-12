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
q1=0.6;     %probability of still accelerating forward while accelerating forward
backcost=0.2;
p=(1-p1)/4;
q=(1-q1-backcost)/4;
v1=[p1 p p p p];
v2=[q1 2*q q 2*q];

%the matrix is "circulant" (trova termine migliore) so we can build it
%algorithmically
Transmat=zeros(ns);
Transmat(1,:)=v1;
Transmat(2:end,1)=ones(4,1).*backcost;
for i=2:ns
    Transmat(i,2:end)=[v2(end+3-i:end),v2(1:end+2-i)];
end


%Senosors grid
range=50;
R=diag([0.01/3,(2*pi/360)^2]);               %sensor covariance
sensoreprova=Sensor([0,0],range,R);



%Power spectra density of process noise
Q=diag([0.0001,0.0001,0.002,0.002,0.05,0.05]);
%state of markov chain
s=1;
%initial 2D position, velocity and acceleration
x0=[0 0 0 1 0 0]';
%magnitude of acceleration
acc=5;
%magnitude of uniform speed
% vel1=1;

%real state and mode
n=1;
stato=zeros(6,10);
mode=zeros(1,10);
stato(:,1)=x0;
mode(1)=s;


%predicted and measured ones
%one cell for each model
statopred=cell(1,ns);
Ppred=cell(1,ns);
for i=1:ns
    statopred{i}(:,:)=zeros(6,10);
    Ppred{i}(:,:,:)=zeros(6,6,10);
    statopred{i}(:,1)=x0;
    Ppred{i}(:,:,1)=Q;
end

for i=1:ns
    statopred1(:,i)=x0;
    Ppred1(:,:,i)=Q;
end

%model probabilities
mu=zeros(5,10);
mu(:,1)=[0.95 0.125 0.125 0.125 0.125]'; %first state is constant velocity for design and we suppose we know

%mixed one (output of IMM
statomix=zeros(6,10);
Pmix=zeros(6,6,10);
statomix(:,1)=x0;
Pmix(:,:,1)=Q;

positionsensed=zeros(2,10);
polarsensed=zeros(2,10);

%initialize sensor Grid
% for i=1:10
%     for j=1:10
%         
%     end
% end

while sensoreprova.inrange
    x=move(ABG,stato(:,n),acc,mode(n),Q);   
    mode(n+1)=markchange(s,Transmat);
    stato(:,n+1)=x;
    sensoreprova.sense(stato(1:2,n+1));
    if sensoreprova.sensed(1)==99999 && sensoreprova.sensed(2)==99999
        break
    else
        %these arrays will be one element shorter
        polarsensed(:,n)=sensoreprova.sensed;
        positionsensed(:,n)=[cos(sensoreprova.sensed(2));sin(sensoreprova.sensed(2))].*sensoreprova.sensed(1);
    end
    [statomix(:,n+1),Pmix(:,:,n+1),statopred1,Ppred1,mu(:,n+1)]=IMM(statopred1,Ppred1,sensoreprova.sensed,ABG,Transmat,Q,R,mu(:,n)',acc);
    for i=1:ns
        statopred{i}(:,n+1)=statopred1(:,i);
        Ppred{i}(:,:,n+1)=Ppred1(:,:,i);
    end
    
    n=n+1;
end


figure(1)
plot(stato(1,1:n),stato(2,1:n))
hold on
plot(positionsensed(1,1:n-1),positionsensed(2,1:n-1),'*')