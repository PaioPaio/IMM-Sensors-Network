clc; clear all; close all;
%% Dati


%switch for the type of movement
%1-> markov chain constant speed or costant acceleration
%2-> unicycle

caso=1;
%delta time
delta=0.05;

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

    %the matrix is "circulant" from so we can build it
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
    Q=diag([0.001,0.001,0.005,0.005]);  
    %state of markov chain
    s=1;
    %magnitude of acceleration
    acc=[1;1].*3;


    case 2
end

%Senosors 
range=10;
R=diag([0.01/3,(2*pi/360)^2]);               %sensor covariance
% sensoreprova=Sensor([0,0],range,R);

%real state and mode
n=1;
stato=zeros(nx,10);
mode=zeros(1,10);
stato(:,1)=x0;
mode(1)=s;
mu1=zeros(1,ns);
mu1(s)=1;
% sensoreprova.inRange(stato(1:2,1));


% %predicted and measured ones
% %one cell for each model
% statopred=cell(1,ns);
% Ppred=cell(1,ns);
% for i=1:ns
%     statopred{i}(:,:)=zeros(nx,10);
%     Ppred{i}(:,:,:)=zeros(nx,nx,10);
%     statopred{i}(:,1)=x0;
%     Ppred{i}(:,:,1)=Q;
% end
% for i=1:ns
%     statopred1(:,i)=x0;
%     Ppred1(:,:,i)=Q;
% end

%model probabilities
mu=zeros(5,10);
mu(:,1)=mu1'; %first state is constant velocity for design and we suppose we know

%mixed one (output of IMM), initialize the first one
statomix=zeros(nx,10);
Pmix=zeros(nx,nx,10);
statomix(:,1)=x0;
Pmix(:,:,1)=Q;
% %just for plotting reasons
% positionsensed=zeros(2,10);
% positionsensed(:,1)=x0(1:2);
% polarsensed=zeros(2,10);

%initialize sensor Grid
nq=10;      %grid's size (even number plez)
sensors=num2cell(zeros(nq));
%basic sensor proprieties
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
                    if ~(i1==0&&j1==0)
                        sensors{i,j}.neighboors{end+1}=[i+i1,j+j1];
                    end
                end
            end
        end
    end
end

% 
% while sensoreprova.inrange
%     x=move(ABG,stato(:,n),acc,mode(n),Q);   
%     mode(n+1)=markchange(s,Transmat);
%     stato(:,n+1)=x;
%     sensoreprova.sense(stato(1:2,n+1));
%     if sensoreprova.inrange==false
%         break
%     else
%         %these arrays will be one element shorter
%         polarsensed(:,n)=sensoreprova.sensed;
%         positionsensed(:,n+1)=[cos(sensoreprova.sensed(2));sin(sensoreprova.sensed(2))].*sensoreprova.sensed(1);
%     end
%     [statomix(:,n+1),Pmix(:,:,n+1),statopred1,Ppred1,mu(:,n+1)]=IMM(statopred1,Ppred1,sensoreprova.sensed,ABG,Transmat,Q,R,mu(:,n)',acc,caso);
%     for i=1:ns
%         statopred{i}(:,n+1)=statopred1(:,i);
%         Ppred{i}(:,:,n+1)=Ppred1(:,:,i);
%     end
%     
%     n=n+1;
% end
 
%%

statecons=zeros(nx,10);
Pcons=zeros(nx,nx,10);
statocons(:,1)=x0;
Pcons(:,:,1)=Q;
n=1;

xpredstart=zeros(nx,ns);
Ppredstart=zeros(nx,nx,ns);
for i=1:ns
    xpredstart(:,i)=x0;
    Ppredstart(:,:,i)=Q;
end

onindices={};     %active vertexes indices
maxi=0;
maxj=0;
mini=999;
minj=999;
%initialize onindices with x0
for i=1:nq
    for j=1:nq
        sensors{i,j}.inRange(stato(1:2,1));
        if sensors{i,j}.inrange
            onindices{end+1}=[i,j];
            sensors{i,j}.xpred=xpredstart;
            sensors{i,j}.Ppred=Ppredstart;
            sensors{i,j}.mu=mu1;
            tellyourfriends(sensors{i,j},sensors,"Can");
            if i<mini
                mini=i;
            end
            if j<minj
                minj=j;
            end
            if i>maxi
                maxi=i;
            end
            if j>maxj
                maxj=j;
            end
        end
    end
end
idlerange=[mini,maxi;minj,maxj];

%we will check only here if some sensor is now in range
outskirt=idlerange+[-1 1;-1 1];
for i=outskirt(1,1):outskirt(1,2)
    for j=outskirt(2,1):outskirt(2,2)
        sensors{i,j}.activevertex=onindices;
    end
end
%we will give these sensors the indices of the active ones, so that they
%can check if those are in their neighborhood and turn on

zsens=[];
Psens=[];
Hsens=[];
musens=[];

while (any(forall(sensors),'all')&&n<1000)
    x=move(ABG,stato(:,n),acc,mode(n),Q);   
    mode(n+1)=markchange(s,Transmat);
    stato(:,n+1)=x;
    %check of all idle sensors if any of them is now in range
    maxi=max([1,idlerange(1,2)-3]);
    maxj=max([1,idlerange(2,2)-3]);
    mini=min([nq,idlerange(1,1)+3]);
    minj=min([nq,idlerange(2,1)+3]);
    for i=idlerange(1,1):idlerange(1,2)
        for j=idlerange(2,1):idlerange(2,2)
            sensors{i,j}.inRange(stato(1:2,n+1));
            check=checkvertex(onindices,[i,j]);
            if sensors{i,j}.inrange
                %add to onindices if it's in range and not already in the
                %list
                if ~any(check,'all')
                    onindices{end+1}=[i,j];
                    %send your neighboors a message that you are now on
                    tellyourfriends(sensors{i,j},sensors,"Can");
                    sensors{i,j}.initializefromidle(sensors)
                    %the sensors doesn't have anything to compute the IMM,
                    %we initialize them getting everything from their
                    %neighboors that are on
                end
                if ~(i-1<1||i+1>nq||j-1<1||j+1>nq)
                    %check that new idlerange is out not of bounds
                    %and update idlerange
                    if i-1<mini
                        mini=i-1;
                    end
                    if j-1<minj
                        minj=j-1;
                    end
                    if i+1>maxi
                        maxi=i+1;
                    end
                    if j+1>maxj
                        maxj=j+1;
                    end
                end
            else
                %kick out of onindices if it's not in range
                if any(check)
                    %send your neighboors a message that you are now idle
                    tellyourfriends(sensors{i,j},sensors,"Cannot");
                    onindices(logical(checkvertex(onindices,[i,j])))=[];
                end
            end
        end
    end
    idlerange=[mini,maxi;minj,maxj];
    
    %update outskirt
    if mini-1<1
        outskirt(1,1)=mini;
    else
        outskirt(1,1)=mini-1;
    end
    if maxi+1>nq
        outskirt(1,2)=maxi;
    else
        outskirt(1,2)=maxi+1;
    end
    if minj-1<1
        outskirt(2,1)=minj;
    else
        outskirt(2,1)=minj-1;
    end
    if maxj+1>nq
        outskirt(2,2)=maxj;
    else
        outskirt(2,2)=maxj+1;
    end
    %give updated onsensors list
    for i=outskirt(1,1):outskirt(1,2)
        for j=outskirt(2,1):outskirt(2,2)
            sensors{i,j}.activevertex=onindices;
        end
    end
    
    
    %compute IMM for all the sensors that are on
    for i=1:length(onindices)
        %get indices from onindices cell array
        kk=onindices{i}(1);
        ll=onindices{i}(2);
        %take a measurement
        sensors{kk,ll}.sense(stato(1:2,n+1));
        %IMM for each sensor on
        
        [sensors{kk,ll}.xmix,sensors{kk,ll}.Pmix...
        ,sensors{kk,ll}.xpred,sensors{kk,ll}.Ppred,sensors{kk,ll}.mu]...
        =IMM(sensors{kk,ll}.xpred,sensors{kk,ll}.Ppred,sensors{kk,ll}.position,...
        sensors{kk,ll}.sensed,ABG,Transmat,Q,sensors{kk,ll}.R,sensors{kk,ll}.mu,acc,caso);
        
        %quantities for consensus
        zsens=[zsens;sensors{kk,ll}.xmix];
        Psens=blkdiag(Psens,sensors{kk,ll}.Pmix);
        Hsens=[Hsens;eye(nx)];
        musens=[musens,sensors{kk,ll}.mu'];
    end
    
    %WLS can be seen as 2 linear consensus algorithms running in parallel
    %in a fully connected graph.
    %Here we have a dynamic (vertexes can go in and out) fully connected 
    %graph
    [statocons(:,n+1),Pcons(:,:,n+1)]=WLS(zsens,Psens,Hsens);
    
    zsens=[];
    Psens=[];
    Hsens=[];
    musens=[];
    n=n+1;
end

xgrid=-range*nq/2:range:range*nq/2;
ygrid=xgrid;

figure(1)
plot(stato(1,1:n),stato(2,1:n))
hold on
for i=1:nq+1
    plot(xgrid,ones(1,nq+1).*ygrid(i),'*r')
end
%plot(positionsensed(1,1:n-1),positionsensed(2,1:n-1),'*')
plot(statocons(1,1:n),statocons(2,1:n))
hold off

diff=statocons-stato(:,1:n);
for i=1:n
    rmspos(i)=norm(diff(1:2,i));
    rmsvel(i)=norm(diff(3:4,i));
end

figure(2)
plot(rmspos)

