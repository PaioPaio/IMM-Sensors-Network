clc; clear all; close all;
%% Dati

%switch for the type of movement
%1-> markov chain constant speed or costant acceleration
%2-> unicycle

caso=2;
%delta time
delta=0.05;
%at which rate do we do the consensus ?
rate=10;
%Maximum number of iteration
nstop=1000;
%iterations montecarlo
nmonte=100;
%Plot
videon = 0;
ploton =0;
plotresults=1;
plotallsenson=0;
plotallsensed=0;
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
        
        
        
        %Power spectra density of process noise
       %%% Q=diag([1,1]);
        %magnitude of acceleration
        acc=[1;1].*5;
        P0=diag([1,1,0.5,0.5]);
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
        ns=5;
        
        %Transition Matrix of markov chain
        p1=0.8;     %probability of constant speed while being in constant speed
        q1=1/2;     %probability of still accelerating forward while accelerating forward
        p=(1-p1)/4;
        q=(1-q1)/2;
        
        % Hardcoded
        Transmat=zeros(ns);
        Transmat(1,:)=[p1 p p p p];
        Transmat(2,:)=[q q1 q 0 0];
        Transmat(3,:)=[q q q1 0 0];
        Transmat(4,:)=[q 0 0 q1 q];
        Transmat(5,:)=[q 0 0 q q1];
        %initial 2D position, velocity and acceleration
        %radius, called like this for reasons
        ABG=0.5;
        %Power spectra density of process noise
        Q=diag([1,1]);
        %magnitude of acceleration
        acc=[1;2].*3;
        P0=diag([1,1,1,1]);
end
%state of markov chain
howconfident=0.95;
s=1;

%Sensors
range=10;
R=diag([0.1,(2*pi/180)^2]);               %sensor covariance

rmserr=[];
maxerr=[];

%mu1 is the starting vector of probabilities for the imm
mu1=zeros(ns,1);
for i=1:ns
    if i==s
        mu1(i)=howconfident;
    else
        mu1(i)=(1-howconfident)/(ns-1);
    end
end


%create sensor Grid
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

for y=1:nmonte
    %initial state
    switch caso
        case 1
            x0=randn(nx,1).*[3;3;2;2];
        case 2
            x0=randn(nx,1).*[3;3;2;pi/2];
    end
    
    n=1;
    ncons=1;
    %real state and mode
    stato=zeros(nx,10); %vector of the real state of the moving object
    mode=zeros(1,10);   %vector of the real mode of the moving object
    stato(:,1)=x0;
    mode(1)=s;
    
    %model probabilities
    mu=zeros(ns,10);
    mu(:,1)=mu1'; %first state is constant velocity for design and we suppose we know
    
    
    
    %%
    
    %initialize sensors grid
    
    statocons=zeros(nx,10);
    Pcons=zeros(nx,nx,10);
    mucons=zeros(ns,10);
    statocons(:,1)=x0;
    Pcons(:,:,1)=P0;
    mucons(:,1)=mu1;
    
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
    
    curve1 = animatedline('Color','r');
    curve2= animatedline('Color','b');
    curve3= animatedline('Color','y');
    
    curvesens1= animatedline('Color','m');
    curvesens2= animatedline('Color','c');
    curvesens3= animatedline('Color','y');
    curvesens4= animatedline('Color','k');
    
    
    curvesens5= animatedline('Color','m','Marker','*');
    curvesens6= animatedline('Color','c');
    curvesens7= animatedline('Color','y');
    curvesens8= animatedline('Color','k','Marker','*');
    
    
    if ploton ==1
        plotS=cell(nq,nq);
        hold on
        hhh=figure(1)
        filename = 'testAnimated.gif';
        for i=1:nq % initializate the plot grid
            for j=1:nq
                plotS{i,j}=sensors{i,j}.plotsensor;
            end
        end
        title('Actual vs Tracked Trajectory')
        xlabel('x [m]')
        ylabel('y [m]')
    end
    xconsallsensor=cell(nstop,1);
    sensed=cell(nstop,1);
    errsensed=cell(nstop,1);
    meanerrsensed=[];
    maxerrsens=0;
    meanimmnocons=[];
    maximmnocons=0;
    
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
        %and maybe plot
        
        for i=idlerange(1,1)-1:idlerange(1,2)+1
            if i>=nq
                i=nq;
            elseif i<=1
                i=1;
            end
            for j=idlerange(2,1)-1:idlerange(2,2)+1
                if j>=nq
                    j=nq;
                elseif j<=1
                    j=1;
                end
                sensors{i,j}.activevertex=onindices;
                if ploton==1
                    plotS{i,j}=sensors{i,j}.plotsensor;
                end
            end
        end
        %compute IMM for all the sensors that are on
        rmsimm=[];
        for i=1:lon
            %get indices from onindices cell array
            kk=onindices{i}(1);
            ll=onindices{i}(2);
            %take a measurement
            sensors{kk,ll}.sense(stato(1:2,n+1));
            sensed{n,i}=[sensors{kk,ll}.position(1)+sensors{kk,ll}.sensed(1)*cos(sensors{kk,ll}.sensed(2));...
                sensors{kk,ll}.position(2)+sensors{kk,ll}.sensed(1)*sin(sensors{kk,ll}.sensed(2))];
            errsensed{n,i}=norm(stato(1:2,n+1)-sensed{n,i});
            
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
            musens=[musens,sensors{kk,ll}.mu'];
            rmsimm(end+1)=norm(sensors{kk,ll}.xmix(1:2)-stato(1:2,n+1));
            maximmnocons=max(maximmnocons,rmsimm(i));
            xksens=[xksens;sensors{kk,ll}.xpred];
            Pksens{i}(:,:,:)=sensors{kk,ll}.Ppred;
            muijsens=blkdiag(muijsens,sensors{kk,ll}.muij);
        end
        
        %WLS can be seen as 2 linear consensus algorithms running in parallel
        %in a fully connected graph.
        %Here we have a dynamic (vertexes can go in and out) fully connected
        %graph
        
        meanerrsensed(end+1)=rms([errsensed{n,:}]);
        maxerrsens=max(maxerrsens,max([errsensed{n,:}]));
        
        
        if rem(n,rate)==0
            % (n+1) is the current iteration of the movement, so we sense that
            % and we cant to do consensus on that. We already have initiated
            % the first consensus at x0 and P0 with at n=1 out of the loop, so
            % the next consensi should happen at n+1=1+rate*k or rem(n,rate)==0
            [statocons(:,ncons+1),Pcons(:,:,ncons+1)]=WLS(xsens,Psens,Hsens);
            if rate>3
                mucons(:,ncons+1)=mean(musens,2);
            end
            %check when do we have to do consensus
            if ploton==1
                addpoints(curve2,statocons(1,ncons+1),statocons(2,ncons+1)) ; % addpoints(curve2,statocons(1,(n+1)/rate+1),statocons(2,(n+1)/rate+1))
                drawnow
            end
            for i=1:lon
                kk=onindices{i}(1);
                ll=onindices{i}(2);
                sensors{kk,ll}.xcons=statocons(:,ncons+1);
                sensors{kk,ll}.Pcons=Pcons(:,:,ncons+1);
                if rate > 3
                sensors{kk,ll}.mu=mucons(:,ncons+1);
                else
                sensors{kk,ll}.mu=sensors{kk,ll}.mu';
                end
                %           
                %pick xcons of all on sensors
                %
                xconsallsensor{n,i}=sensors{kk,ll}.xcons;
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
            meanimmnocons(end+1)=norm(stato(1:2,n+1)-statocons(1:2,ncons+1));
            maximmnocons=max(maximmnocons,norm(stato(1:2,n+1)-statocons(1:2,ncons+1)));
            ncons=ncons+1;
        else
            %addpoints(curve2,statocons(1,n-1),statocons(2,n-1));
            for i=1:lon
                kk=onindices{i}(1);
                ll=onindices{i}(2);
                sensors{kk,ll}.xcons=sensors{kk,ll}.xmix;
                sensors{kk,ll}.Pcons=sensors{kk,ll}.Pmix;
                sensors{kk,ll}.xconskalm=sensors{kk,ll}.xpred;
                sensors{kk,ll}.Pconskalm=sensors{kk,ll}.Ppred;
                sensors{kk,ll}.mu=sensors{kk,ll}.mu';
                
                % %pick xcons of all on sensors
                %
                xconsallsensor{n,i}=sensors{kk,ll}.xcons;
                
            end
            meanimmnocons(end+1)=rms(rmsimm);
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
        
        if ploton==1
            if rem(n-1,rate)==0
                [ell_x,ell_y] =plotellipse(Pcons(1:2,1:2,ncons),statocons(1:2,ncons),ploton);
                ellx(:,ncons)=ell_x(:);
                elly(:,ncons)=ell_y(:);
                
            end
            
            addpoints(curve1,stato(1,n),stato(2,n))
            drawnow
            if plotallsenson==1
                if lon>=1
                    addpoints(curvesens1,xconsallsensor{n,1}(1),xconsallsensor{n,1}(2));
                    drawnow
                end
                if lon>=2
                    addpoints(curvesens2,xconsallsensor{n,2}(1),xconsallsensor{n,2}(2));
                    drawnow
                end
                if lon>=3
                    addpoints(curvesens3,xconsallsensor{n,3}(1),xconsallsensor{n,3}(2));
                    drawnow
                end
                if lon>=4
                    addpoints(curvesens4,xconsallsensor{n,4}(1),xconsallsensor{n,4}(2));
                    drawnow
                end
            end
            if plotallsensed==1
                if lon>=1
                    %                 addpoints(curvesens5,sensed{n,1}(1),sensed{n,1}(2));
                    %                 drawnow
                    plot(sensed{n,1}(1),sensed{n,1}(2),'*','Color','b')
                end
                if lon>=2
                    %                 addpoints(curvesens6,sensed{n,2}(1),sensed{n,2}(2));
                    %                 drawnow
                    plot(sensed{n,2}(1),sensed{n,2}(2),'*','Color','c')
                end
                if lon>=3
                    %                 addpoints(curvesens7,sensed{n,3}(1),sensed{n,3}(2));
                    %                 drawnow
                    plot(sensed{n,3}(1),sensed{n,3}(2),'*','Color','m')
                end
                if lon>=4
                    %                 addpoints(curvesens8,sensed{n,4}(1),sensed{n,4}(2));
                    %                 drawnow
                    plot(sensed{n,4}(1),sensed{n,4}(2),'*','Color','k')
                end
            end
            %         for i=1:lon
            %         addpoints(curvesens(i),xconsallsensor{n,i}(1),xconsallsensor{n,i}(2));
            %         drawnow
            %         end
        end
        %% to enable to save gifs
        %    if n == 1
        % frame = getframe(hhh);
        %       im = frame2im(frame);
        %       [imind,cm] = rgb2ind(im,256)
        %           imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        %    else
        %    if rem(n,3)==0
        %       frame = getframe(hhh);
        %       im = frame2im(frame);
        %       [imind,cm] = rgb2ind(im,256);
        %       % Write to the GIF File
        %
        %           imwrite(imind,cm,filename,'gif','WriteMode','append');
        %       end
        %     end
        %% Video
        if videon == 1
            frame(n) = getframe(hhh);
        end
        n=n+1;
    end
    
    diff=statocons-stato(:,1:rate:rate*ncons);
    for i=1:ncons
        rmspos(i)=norm(diff(1:2,i));
        switch caso
            case 1
                rmsvel(i)=norm(diff(3:4,i));
            case 2
                rmsvel(i)=abs(diff(3,i));
                rmsang(i)=abs(rem(diff(4,i),2*pi));
        end
    end
    rmserr(y)=rms(rmspos);
    maxerr(y)=max(rmspos);
    rmssenserr(y)=rms(meanerrsensed);
    maxnsenserr(y)=maxerrsens;
    maxerrnocons(y)=maximmnocons;
    rmsnocons(y)=rms(meanimmnocons);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% Plot Zone %%%%%%%%%%%%%%%%%%
if videon==1
    video = VideoWriter('Gif_Movie\Movie_all_sensors.avi', 'Motion JPEG AVI');
    video.FrameRate = 15;
    open(video);
    writeVideo(video, frame);
    close(video);
end

hold off

if plotresults==1
   fig2= figure(2)
fig2.WindowState = 'maximized';
    plot(meanerrsensed)
    title('RMS of ON Sensors Error Norm on Position')
    ylabel('rms(|dx|) [m]')
    xlabel('Timestep number [k]')
    
   fig3= figure(3)
fig3.WindowState = 'maximized';
    plot(meanimmnocons)
    title('RMS Individual IMM Errors Norm on Position')
    ylabel('|dx| [m]')
    xlabel('Timestep number [k]')
    
    fig4=figure(4)
fig4.WindowState = 'maximized';
    plot(rmspos)
    title('Consensus IMM Error Norm on Position')
    ylabel('|dx| [m]')
    xlabel('Consensus number [k]')
    
    figure(5)
    switch caso
        case 1
           fig5=  figure(5)
           fig5.WindowState = 'maximized';
            plot(rmsvel);
            title('Consensus IMM Error Norm on Speed')
            ylabel('|dv| [m/s]')
            xlabel('Consensus number [k]')
        case 2
         fig5=   figure(5)
fig5.WindowState = 'maximized';
            plot(rmsvel);
            title('Consensus IMM Error Norm on Tangential Speed')
            ylabel('|dv| [m/s]')
            xlabel('Consensus number [k]')
           fig6= figure(6)
fig6.WindowState = 'maximized';
            plot(rmsang);
            title('Consensus IMM Error Norm on Yaw Angle')
            ylabel('|d\alpha| [rad]')
            xlabel('Consensus number [k]')
    end
 fig7=   figure(7)
fig7.WindowState = 'maximized';
    plot(rmserr);
    title('RMS of Consensus IMM Position Errors at Iteration [k]')
    ylabel('rms(dx) [m]')
    xlabel('Iteration [k]')
   fig8= figure(8)
fig8.WindowState = 'maximized';
    plot(maxerr);
    title('Max of Consensus IMM Position Errors at Iteration [k]')
    ylabel('max(dx) [m]')
    xlabel('Iteration [k]')
   fig9= figure(9)
fig9.WindowState = 'maximized';
    plot(rmssenserr)
    title('RMS of RMS of ON Sensors Error Norm on Position at Iteration [k]')
    ylabel('rms(|dx|) [m]')
    xlabel('Iteration number [k]')
   fig10= figure(10)
fig10.WindowState = 'maximized';
    plot(maxnsenserr)
    title('Max of ON Sensors Error Norm on Position at Iteration [k]')
    ylabel('max(|dx|) [m]')
    xlabel('Iteration number [k]')
   fig11= figure(11)
fig11.WindowState = 'maximized';
    plot(maxerrnocons)
    title('Max of Individual IMM Errors Norm on Position at Iteration [k]')
    ylabel('max(|dx|) [m]')
    xlabel('Iteration number [k]')
  fig12=  figure(12)
fig12.WindowState = 'maximized';
    plot(rmsnocons)
    title('RMS of RMS Individual IMM Errors Norm on Position at Iteration [k]')
    ylabel('max(|dx|) [m]')
    xlabel('Iteration number [k]')
end

mediamedie=mean(rmserr)
mediamax=mean(maxerr)
maxmax=max(maxerr)

mediamedienocons=rms(rmsnocons)
maxmedianocons=max(rmsnocons)
maxmaxnocons=max(maxerrnocons)

 mediasens=mean(rmssenserr)
 mediamaxsens=mean(maxnsenserr)
 maxsens=max(maxnsenserr)

saveas(fig4,'errpos.png')
saveas(fig5,'errorspeed.png')
saveas(fig6,'erroryaw.png')
saveas(fig8,'maxIMM.png')
saveas(fig3,'errnocons.png')
saveas(fig11,'maxnocons.png')
saveas(fig7,'RMSIMM.png')
saveas(fig12,'RMSnocons.png')