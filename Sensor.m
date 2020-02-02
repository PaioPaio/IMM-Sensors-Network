classdef Sensor<handle
    properties
       range {}                 %how far can the sensor work
       R {}                     %diagonal 
       
       sensed {}                %[distance,angle]
       
       position {}              %position of the sensor in the room
       ingrid {}                %indices in the grid
       
       inrange=false            %check on range
       state {}                 %on/idle/off
       neighboors {}            %cell array with indices of listeners/sources
       activevertex{}           %active graph indices
       
       change {} 
       color {}                 % for plot
       %token that should be -1 if a neighboor has gone from ON to IDLE,
       % 1 if it has gone from IDLE to ON, 0 in any other case
       
       xmix {}                  %state result of IMM filter estimation
       Pmix {}                  %covariance result o IMM filter estimation
       xpred {}                 %all predicted states from every kalmann filter
       Ppred {}                 %all predicted covariances from every kalman filter
       mu {}                    %all model probabilities from every kalman filter
       muij {}                  %
       
       xcons {}                 %state after consesus
       Pcons {}                 %covariance after consensus
       xconskalm {}             %state for each kalman after consensus
       Pconskalm {}             %covariance for each kalman after consensus
       
    end
    
    methods
        
        %generating function
        function obj=Sensor(position,range,sigma,grid)
            obj.position=position;
            obj.range=range;
            obj.R=sigma;
            obj.state="off";
            if nargin<4
                obj.ingrid=[0,0];
            else
                obj.ingrid=grid;
            end
            obj.sensed=[0,0]';               %starts as [0,0]
            obj.neighboors={};
        end
        
        %to check if in range of the object moving and to change status
        function obj=inRange(obj,x)
            distance=sqrt((obj.position-x')*(obj.position-x')');
            if distance>obj.range
                obj.inrange=false;
                if obj.state=="on"
                    obj.state="idle";
                end
            else
                obj.inrange=true;
                obj.state="on";
            end
        end
    
        %it measures distance and angle from the sensor
        function obj=sense(obj,x)
            %obj->sensore, x->column vector that stores the position of the
            %moving object
            inRange(obj,x);
            if obj.inrange
                vector=x'-obj.position;
                actualangle=atan2(vector(2),vector(1));
                obj.sensed=[sqrt(vector*vector');actualangle]+randn(2,1).*arrayfun(@(a)sqrt(a),diag(obj.R));
            end
        end
        function obj=plotsensor(obj)
            if obj.state=="on"
                plot(obj.position(1),obj.position(2),'*g');
            elseif obj.state=="idle"
                plot(obj.position(1),obj.position(2),'*y');
            elseif obj.state=="off"
                plot(obj.position(1),obj.position(2),'*r');
            end
        end
        function obj=checkchange(obj)
            %checks events nearby
            if obj.change==1
                %if some sensor nearby has turned on then switch to idle
                if obj.state=="off"
                    obj.state="idle";
                end
                obj.change=0;
            elseif obj.change==-1
                %if some sensor nearby has turned off from idle, and you see that no
                %other sensor nearby is on, then turn off
                a=checkvertex(obj.activevertex,obj.neighboors);
                if obj.state=="idle" && ~any(a,'all')
                    obj.state="off";
                end
                obj.change=0;
            end
        end
        
        %when the state goes from idle to on, the sensor gets all the data
        %from it's neighboors that are on, this is dont through a WLS (that
        %can be seen as a consensus in it's own way). The probabilities of
        %the various kalman filters are only a mean of all the other
        %sensors.
        function obj=initializefromidle(obj,sensorgrid)
            %obj->sensor,   sensorgrid->the whole sensor grid
            %check which neighboors are on
            %again matlab sucks with all these temp stuff
            indic=obj.activevertex{1};
            
            lungstato=length(sensorgrid{indic(1),indic(2)}.xmix);
            nummarkov=size(sensorgrid{indic(1),indic(2)}.Ppred,3);
            numsensors=length(obj.activevertex);
            
            xcons1=[];
            covmix=[];
            
            xpred1=[];
            wantPpred={};
            mu1=[];
            muijg=[];
            Hcons=[];
            Hmu=[];
            
            for i=1:length(obj.activevertex)
                kk=obj.activevertex{i}(1);
                ll=obj.activevertex{i}(2);
                %final state and covariance estimate
                xcons1=[xcons1;sensorgrid{kk,ll}.xcons];
                covmix=blkdiag(covmix,sensorgrid{kk,ll}.Pcons);
                %all the kalman estimates
                xpred1=[xpred1;sensorgrid{kk,ll}.xpred];
                wantPpred{i}(:,:,:)=sensorgrid{kk,ll}.Ppred;
                mu1=[mu1,sensorgrid{kk,ll}.mu];
                Hcons=[Hcons;eye(lungstato)];
                Hmu=[Hmu;eye(nummarkov)];
            end
            %do WLS from all the data of the on neighboors
            
            %the final result of the IMM
            %H is gonna be a stacked matrix of identities since our measure
            %is exactly zmix
            [obj.xcons,obj.Pcons]=WLS(xcons1,covmix,Hcons);
            obj.mu=mean(mu1,2);
            %kalman
            for j=1:nummarkov
                Cpred=[];
                for i=1:numsensors
                    Cpred=blkdiag(Cpred,wantPpred{i}(:,:,j));
                end
                [obj.xconskalm(:,j),obj.Pconskalm(:,:,j)]=WLS(xpred1(:,j),Cpred,Hcons);
                [obj.xpred(:,j),obj.Ppred(:,:,j)]=WLS(xpred1(:,j),Cpred,Hcons);
            end
        end
    end
    
end
