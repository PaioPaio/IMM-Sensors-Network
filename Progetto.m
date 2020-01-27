clc; clear all; close all;
%% Dati
rmsposall=zeros(100,1500);
for ss=1:100
%switch for the type of movement
%1-> markov chain constant speed or costant acceleration
%2-> unicycle

caso=2;
%delta time
delta=0.5;
%at which rate do we do the consensus ?
rate=1;

nstop=1500;

%Cell matrix of various B for the input
switch caso
    
    case 1
        %
        %
        %markov chain random walk with 5 states
        %Linear Model
        %
        %
        
        
        A=[1 0 delta 0;
            0 1 0 delta;
            0 0 1 0;
            0 0 0 1];
        G=[delta^2/2 0 delta 0;0 delta^2/2 0 delta]';       %noise matrix
        
        ABG={A,[0 0 0 0; 0 0 0 0]',G;A,[0 0 0 0;0 delta^2/2 0 delta]',G;A,[delta^2/2 0 delta 0; 0 0 0 0]',G;A,[0 0 0 0;0 -delta^2/2 0 -delta]',G;A,[-delta^2/2 0 -delta 0; 0 0 0 0]',G};
        
        %length of state vector
        nx=size(A,1);
        
        %Transition Matrix of markov chain
        p1=0.6;     %probability of constant speed while being in constant speed
        q1=0.5;     %probability of still accelerating forward while accelerating forward
        backcost=0.2;
        p=(1-p1)/4;
        q=(1-q1-backcost)/5;
        v1=[p1 p p p p];
        v2=[q1 2*q q 2*q];
        
        %number of markov states
        ns=length(v1);
        
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
        x0([1,2,4])=0.5;
        
        %Power spectra density of process noise
        Q=diag([0.2,0.2]);
        %magnitude of acceleration
        acc=[1;1].*1;
        P0=diag([0.01,0.01,0.005,0.005]);
        %
        %
        %
        %
    case 2
        %
        %Unycicle
        %this is a nonlinear model
        %
        
        %length of state vector
        nx=4;
        
        %Transition Matrix of markov chain
        p1=0.7;     %probability of constant speed while being in constant speed
        q1=0.4;     %probability of still accelerating forward while accelerating forward
        backcost=0.2;
        p=(1-p1)/4;
        q=(1-q1-backcost)/5;
        v1=[p1 p p p p];
        v2=[q1 2*q q 2*q];
        
        ns=length(v1);
        
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
        x0([1,2])=0.5;
        x0(4)=pi/2;
        %radius, called like this for reasons
        ABG=0.3;
        %Power spectra density of process noise
        Q=diag([0.2,0.2]);
        %magnitude of acceleration
        acc=[2;1].*0.1;
        P0=diag([0.1,0.1,0.5,0.5]);
end
%state of markov chain
% s=randi([1,ns]);
howconfident=0.95;
s=1;

%Senosors
range=10;
R=diag([0.01,(2*pi/360)^2]);               %sensor covariance
% sensoreprova=Sensor([0,0],range,R);

%real state and mode
% n=1;
stato=zeros(nx,10); %vector of the real state of the moving object
mode=zeros(1,10);   %vector of the real mode of the moving object
stato(:,1)=x0;
mode(1)=s;


%mu1 is the starting vector of probabilities for the imm
mu1=zeros(ns,1);
for i=1:ns
    if i==s
        mu1(i)=howconfident;
    else
        mu1(i)=(1-howconfident)/(ns-1);
    end
end
% mu1=[0.9 0.25 0.25 0.25 0.25]';
% sensoreprova.inRange(stato(1:2,1));

%model probabilities
mu=zeros(ns,10);
mu(:,1)=mu1'; %first state is constant velocity for design and we suppose we know



%%

%initialize sensor Grid
nq=20;      %grid's size (even number plez)
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


statecons=zeros(nx,10);
Pcons=zeros(nx,nx,10);
mucons=zeros(ns,10);
muijcons=zeros(ns,ns,10);
statocons(:,1)=x0;
Pcons(:,:,1)=P0;
mucons(:,1)=mu1;
muijcons(:,:,1)=Transmat;

n=1;
ncons=1;

xpredstart=zeros(nx,ns);
Ppredstart=zeros(nx,nx,ns);
for i=1:ns
    xpredstart(:,i)=x0;
    Ppredstart(:,:,i)=P0;
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
            sensors{i,j}.xconskalm=xpredstart;
            sensors{i,j}.Pconskalm=Ppredstart;
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

lon=length(onindices);

xsens=[];
Psens=[];
Hsens=[];

musens=[];
Hmu=[];
muijsens=[];

xksens=[];
Pksens=cell(1,lon);

xgrid=-range*nq/2:range:range*nq/2;
ygrid=xgrid;

% hold on
% figure(1)
% for i=1:nq+1
%     plot(xgrid,ones(1,nq+1).*ygrid(i),'*r')
% end


while (~(isempty(onindices))&&n<nstop)
    x=move(stato(:,n),acc,mode(n),Q,caso,delta,ABG);
    mode(n+1)=markchange(s,Transmat);
    stato(:,n+1)=x;
    %check of all idle sensors if any of them is now in range
    
    for i=idlerange(1,1):idlerange(1,2)
        for j=idlerange(2,1):idlerange(2,2)
            sensors{i,j}.inRange(stato(1:2,n+1));
            check=checkvertex(onindices,[i,j]);
            if sensors{i,j}.inrange
                %add to onindices if it's in range and not already in the
                %list
                if ~any(check,'all')
                    
                    sensors{i,j}.activevertex=onindices;
                    sensors{i,j}.initializefromidle(sensors);
                    %the sensors doesn't have anything to compute the IMM,
                    %we initialize them getting everything from their
                    %neighboors that are on
                    
                    onindices{end+1}=[i,j];
                    sensors{i,j}.activevertex=onindices;
                    %send your neighboors a message that you are now on
                    tellyourfriends(sensors{i,j},sensors,"Can");
                end
            elseif ~sensors{i,j}.inrange
                %kick out of onindices if it's not in range
                if any(check,'all')
                    %send your neighboors a message that you are now idle
                    onindices(logical(checkvertex(onindices,[i,j])))=[];
                    sensors{i,j}.activevertex=onindices;
                    tellyourfriends(sensors{i,j},sensors,"Cannot");
                end
            end
        end
    end
    
    lon=length(onindices);
    if lon==0
        break
    end
    for i=1:lon
        mini=max([1,min([onindices{i}(1)-1,mini])]);
        minj=max([1,min([onindices{i}(2)-1,minj])]);
        maxi=min([nq,max([onindices{i}(1)+1,maxi])]);
        maxj=min([nq,max([onindices{i}(2)+1,maxi])]);
    end
    
    
    idlerange=[mini,maxi;minj,maxj];
    %give updated onsensors list
    for i=idlerange(1,1):idlerange(1,2)
        for j=idlerange(2,1):idlerange(2,2)
            sensors{i,j}.activevertex=onindices;
        end
    end
    
    
    %compute IMM for all the sensors that are on
    for i=1:lon
        %get indices from onindices cell array
        kk=onindices{i}(1);
        ll=onindices{i}(2);
        %take a measurement
        sensors{kk,ll}.sense(stato(1:2,n+1));
        %IMM for each sensor on
        
        [sensors{kk,ll}.xmix,sensors{kk,ll}.Pmix...
            ,sensors{kk,ll}.xpred,sensors{kk,ll}.Ppred,sensors{kk,ll}.mu,sensors{kk,ll}.muij]...
            =IMM(sensors{kk,ll}.xconskalm,sensors{kk,ll}.Pconskalm,sensors{kk,ll}.position,...
            sensors{kk,ll}.sensed,ABG,Transmat,Q,sensors{kk,ll}.R,sensors{kk,ll}.mu',acc,caso,delta);
        
        %quantities for consensus
        xsens=[xsens;sensors{kk,ll}.xmix];
        Psens=blkdiag(Psens,sensors{kk,ll}.Pmix);
        Hsens=[Hsens;eye(nx)];
        Hmu=[Hmu;eye(ns)];
        musens=[musens;sensors{kk,ll}.mu'];
        
        xksens=[xksens;sensors{kk,ll}.xpred];
        Pksens{i}(:,:,:)=sensors{kk,ll}.Ppred;
        muijsens=blkdiag(muijsens,sensors{kk,ll}.muij);
        
    end
    
    %WLS can be seen as 2 linear consensus algorithms running in parallel
    %in a fully connected graph.
    %Here we have a dynamic (vertexes can go in and out) fully connected
    %graph
    if rem(n+1,rate)==0
        [statocons(:,ncons+1),Pcons(:,:,ncons+1)]=WLS(xsens,Psens,Hsens);
        [mucons(:,ncons+1),muijcons(:,:,ncons+1)]=WLS(musens,muijsens,Hmu);
        %check when do we have to do consensus
        for i=1:lon
            kk=onindices{i}(1);
            ll=onindices{i}(2);
            sensors{kk,ll}.xcons=statocons(:,ncons+1);
            sensors{kk,ll}.Pcons=Pcons(:,:,ncons+1);
            sensors{kk,ll}.mu=mucons(:,ncons+1);
        end
        
        for i=1:ns
            Pkksens=[];
            for j=1:lon
                Pkksens=blkdiag(Pkksens,Pksens{j}(:,:,i));
            end
            
            for j=1:lon
                kk=onindices{j}(1);
                ll=onindices{j}(2);
                [sensors{kk,ll}.xconskalm(:,i),sensors{kk,ll}.Pconskalm(:,:,i)]=WLS(xksens(:,i),Pkksens,Hsens);
            end
        end
        ncons=ncons+1;
    else
        for i=1:lon
            kk=onindices{i}(1);
            ll=onindices{i}(2);
            sensors{kk,ll}.xcons=sensors{kk,ll}.xmix;
            sensors{kk,ll}.Pcons=sensors{kk,ll}.Pmix;
            sensors{kk,ll}.xconskalm=sensors{kk,ll}.xpred;
            sensors{kk,ll}.Pconskalm=sensors{kk,ll}.Ppred;
            sensors{kk,ll}.mu=sensors{kk,ll}.mu';

            Pos_sens_on{n,i}=sensors{kk,ll}.position;
            Pos_sens_grid{n,i}=sensors{kk,ll}.ingrid;
        end
    end
    
    if rem(n+1,rate)==0
        [ell_x,ell_y] =plotellipse(Pcons(1:2,1:2,ncons),statocons(1:2,ncons),0);
        ellx(:,ncons)=ell_x(:);
        elly(:,ncons)=ell_y(:);
    end
    
    for q=1:size(onindices,2)
        saveon{n,q}=onindices{q};
    end
    
    xsens=[];
    Psens=[];
    Hsens=[];
    musens=[];
    Hmu=[];
    
    xksens=[];
    Pksens=cell(1,lon);
    muijsens=[];
    %plot(stato(1,n),stato(2,n),'ro')
    n=n+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Plot Zone %%%%%%%%%%%%%%%%%%


% hold off
% xgrid=-range*nq/2:range:range*nq/2;
% ygrid=xgrid;
% 
% 
% curve1 = animatedline('Color','r');
% curve2= animatedline;
% 
% curve3= animatedline('Color','b');
% curve4= animatedline('Marker','*','Color','g');
% 
% % for u=1:size(Pos_sens_on,2)
% %     if Pos_sens_on{i+1,u} ~= 0
% %      addpoints(curve4,Pos_sens_on{i+1,u}(1),Pos_sens_on{i+1,u}(2),'*g');
% %      drawnow
% %     end  
% %     end
% 
% 
% dd=size(ellx,2);
% for i=1:(n-dd)+10
%     ellx(:,dd+i)=zeros(size(ellx,1),1);
%     elly(:,dd+i)=zeros(size(elly,1),1);
% end
% figure(1)
% ni=1;
% hold on
% for i=1:nq+1
%     plot(xgrid,ones(1,nq+1).*ygrid(i),'*r')
% end
% for i=0:n-2
%     addpoints(curve1,stato(1,i+1),stato(2,i+1))
%     drawnow
%     
% % %     for q=1:nq+1
% % %     plot(xgrid,ones(1,nq+1).*ygrid(q),'*r')
% % %     end
% %     
%     for u=1:size(Pos_sens_on,2)
%         if size(Pos_sens_on{i+1,u}) == [0,0];
%         Pos_sens_on{i+1,u} = [0,0];
%         Pos_sens_grid{i+1,u} =[1,1];
%         
% 
%     else
%         if i>15
% %         temp1=Pos_sens_on{i,u}(1);
% %         temp2=Pos_sens_on{i,u}(2);
%             if sensors{Pos_sens_grid{i-14,u}(1),Pos_sens_grid{i-14,u}(2)}.inrange==false
%                 plot(Pos_sens_on{i-14,u}(1),Pos_sens_on{i-14,u}(2),'*b'); %,u}(1),Pos_sens_on{i+1,u}(2)
%             end
%         end   
%     plot(Pos_sens_on{i+1,u}(1),Pos_sens_on{i+1,u}(2),'*g'); %,u}(1),Pos_sens_on{i+1,u}(2)
%        
%         end 
%     end
%     if rem(i,rate)==0
%         addpoints(curve2,statocons(1,ni),statocons(2,ni))
%         drawnow
%         addpoints(curve3,ellx(:,ni),elly(:,ni))
%         drawnow
%         %plot([ellx(i,ni),ellx(i+1,ni)],[elly(i,ni),elly(i+1,ni)])
%         ni=ni+1;
%     end
% end
% 
% for i=0:n-2
% for u=1:size(Pos_sens_on,2)
%     if size(Pos_sens_on{i+1,u}) == [0,0];
%     else
%     plot(Pos_sens_on{i+1,u}(1),Pos_sens_on{i+1,u}(2),'*g'); %,u}(1),Pos_sens_on{i+1,u}(2)
%     end
% end
% end
% hold off
% % plot utilizzando maggiore degli outskirt e il minore
% figure(3)
% plot(stato(1,1:n),stato(2,1:n))
% hold on
% for i=1:nq+1
%     plot(xgrid,ones(1,nq+1).*ygrid(i),'*r')
% end
% %plot(positionsensed(1,1:n-1),positionsensed(2,1:n-1),'*')
% plot(statocons(1,1:ncons),statocons(2,1:ncons))
% hold off
diff=statocons(:,(1:rate:n)./rate)-stato(:,1:rate:n);
for i=1:ncons
    rmspos(i)=norm(diff(1:2,i));
    rmsvel(i)=norm(diff(3:4,i));
end
rmsposall(ss,1:size(rmspos,2))=rmspos;
end

save rms.mat rmsposall 
% diff=statocons(:,(1:rate:n)./rate)-stato(:,1:rate:n);
% for i=1:ncons
%     rmspos(i)=norm(diff(1:2,i));
%     rmsvel(i)=norm(diff(3:4,i));
% end

% figure(4)
% plot(rmspos)
% 
% figure(5)
% plot(stato(3,1:n))

