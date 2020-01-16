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
    A=[1 0 delta 0;
       0 1 0 delta;
       0 0 1 0;
       0 0 0 1];    
    G=eye(4);       %noise matrix

    ABG={A,[0 0 0 0; 0 0 0 0]',G;A,[0 0 0 0;0 delta^2/2 0 delta]',G;A,[delta^2/2 0 delta 0; 0 0 0 0]',G;A,[0 0 0 0;0 -delta^2/2 0 -delta]',G;A,[-delta^2/2 0 -delta 0; 0 0 0 0]',G};
    
    %number of markov states
    ns=length(ABG);
    %length of state vector
    nx=size(A,1);
    
    %Transition Matrix of markov chain
    p1=0.8;     %probability of constant speed while being in constant speed
    q1=0.6;     %probability of still accelerating forward while accelerating forward
    backcost=0.2;
    p=(1-p1)/4;
    q=(1-q1-backcost)/5;
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
    
    %initial 2D position, velocity and acceleration
    x0=zeros(nx,1);
    x0([1,2,4])=1;
    
    %Power spectra density of process noise
    Q=diag([0.0001,0.0001,0.002,0.002]);  
    %state of markov chain
    s=1;
    %magnitude of acceleration
    acc=[1;1].*3;


    case 2
end

%Senosors 
range=20;
R=diag([0.01/3,(2*pi/360)^2]);               %sensor covariance
sensoreprova=Sensor([0,0],range,R);

%real state and mode
n=1;
stato=zeros(nx,10);
mode=zeros(1,10);
stato(:,1)=x0;
mode(1)=s;


%predicted and measured ones
%one cell for each model
statopred=cell(1,ns);
Ppred=cell(1,ns);
for i=1:ns
    statopred{i}(:,:)=zeros(nx,10);
    Ppred{i}(:,:,:)=zeros(nx,nx,10);
    statopred{i}(:,1)=x0;
    Ppred{i}(:,:,1)=Q;
end
for i=1:ns
    statopred1(:,i)=x0;
    Ppred1(:,:,i)=Q;
end

%model probabilities
mu=zeros(5,10);
mu(:,1)=[1 0 0 0 0]'; %first state is constant velocity for design and we suppose we know

%mixed one (output of IMM), initialize the first one
statomix=zeros(nx,10);
Pmix=zeros(nx,nx,10);
statomix(:,1)=x0;
Pmix(:,:,1)=Q;
%just for plotting reasons
positionsensed=zeros(2,10);
positionsensed(:,1)=x0(1:2);
polarsensed=zeros(2,10);

%initialize sensor Grid
nq=10;      %grid's size (even number plez)
sensors=num2cell(zeros(nq));
for i=1:nq
    for j=1:nq
        sensors{i,j}=Sensor([(-nq/2+i)*range,(-nq/2+j)*range],range,R,[i,j]);
    end
end
for i=1:nq
    for j=1:nq
        for i1=-1:1
            for j1=-1:1
                if (i+i1<1||i+i1>nq||j+j1<1||j+j1>nq)
                     continue;
                else
                    sensors{i,j}.addlistener(sensors{i+i1,j+j1});
                    sensors{i,j}.neighboors(end+1)=[i+i1,j+j1];
                end
            end
        end
    end
end


while sensoreprova.inrange
    x=move(ABG,stato(:,n),acc,mode(n),Q);   
    mode(n+1)=markchange(s,Transmat);
    stato(:,n+1)=x;
    sensoreprova.sense(stato(1:2,n+1));
    if sensoreprova.inrange==false
        break
    else
        %these arrays will be one element shorter
        polarsensed(:,n)=sensoreprova.sensed;
        positionsensed(:,n+1)=[cos(sensoreprova.sensed(2));sin(sensoreprova.sensed(2))].*sensoreprova.sensed(1);
    end
    [statomix(:,n+1),Pmix(:,:,n+1),statopred1,Ppred1,mu(:,n+1)]=IMM(statopred1,Ppred1,sensoreprova.sensed,ABG,Transmat,Q,R,mu(:,n)',acc,caso);
    for i=1:ns
        statopred{i}(:,n+1)=statopred1(:,i);
        Ppred{i}(:,:,n+1)=Ppred1(:,:,i);
    end
    
    n=n+1;
end
 
%%

onindices={};     %active vertexes indices
mini=999;
minj=999;
%initialize onindices with x0
for i=1:nq
    for j=1:nq
        sensors{i,j}.inRange(stato(1:2,1));
        if sensors{i,j}.inrange
            onindices(end+1)=[i,j];
            if i<mini
                mini=i;
            end
            if j<minj
                minj=j;
            end
            maxi=i;
            maxj=j;
        end
    end
end
idlerange=[mini,maxi;minj,maxj];


while (any([sensors{:,:}.inrange])&&n<1000)
    x=move(ABG,stato(:,n),acc,mode(n),Q);   
    mode(n+1)=markchange(s,Transmat);
    stato(:,n+1)=x;
    %check of all idle sensors if any of them is now in range
    for i=idlerange(1,1):idlerange(1,2)
        for j=idlerange(2,1):idlerange(2,2)
            sensors{i,j}.inRange(stato(1:2,n+1));
            check=checkvertex(onindices,[i,j]);
            if sensors{i,j}.inrange
                %add to onindices if it's in range
                if ~any(check)
                    onindices(end+1)=[i,j];
                    sensors{i,j}.initializefromidle(sensors)
                    %the sensors doesn't have anything to compute the IMM,
                    %we initialize it getting everything from it's
                    %neighboors that are on
                end
            else
                %kick out of onindices if it's not in range
                if any(check)
                    onindices(logical(checkvertex(onindices,[i,j])))=[];
                end
            end
        end
    end
    
    for i=1:length(onindices)
        %get indices from onindices cell array
        kk=onindices{i}(1);
        ll=onindices{i}(2);
        sensors{kk,ll}.sense(stato(1:2,n+1));
        %IMM for each sensor on
        
        [sensors{kk,ll}.xmix,sensors{kk,ll}.Pmix...
        ,sensors{kk,ll}.xpred,sensors{kk,ll}.Ppred,sensors{kk,ll}.mu]...
        =IMM(sensors{kk,ll}.xpred,sensors{kk,ll}.Ppred,...
        sensors{kk,ll}.sensed,ABG,Transmat,Q,sensors{kk,ll}.R,sensors{kk,ll}.mu',acc,caso);
    
    end
end


figure(1)
plot(stato(1,1:n),stato(2,1:n))
hold on
plot(positionsensed(1,1:n-1),positionsensed(2,1:n-1),'*')
plot(statomix(1,1:n-1),statomix(2,1:n-1))
hold off

diffmeas=positionsensed-stato(1:2,1:n);
for i=1:n
    rmsmeas(i)=norm(diffmeas(:,i));
end
diff=statomix-stato(:,1:n);
for i=1:n
    rmspos(i)=norm(diff(1:2,i));
    rmsvel(i)=norm(diff(3:4,i));
end

figure(2)
plot(rmspos)
hold on
plot(rmsmeas)
hold off

