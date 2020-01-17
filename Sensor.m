classdef Sensor<handle
    properties (SetObservable = true)
       range {}                 %how far can the sensor work
       R {}                     %diagonal cuz we cool
       
       sensed {}                %[distance,angle]
       
       position {}              %position of the sensor in the room
       ingrid {}                %indices in the grid
       
       inrange=false             %pretty selfexplanatory
       state {}                 %on/idle/off
       neighboors {}            %cell array with indices of listeners/sources
       activevertex{}           %active graph indices
       
       change {}                
       %token that should be -1 if a neighboor has gone from ON to IDLE,
       % 1 if it has gone from IDLE to ON, 0 in any other case
       
       xmix {}                  %state result of IMM filter estimation
       Pmix {}                  %covariance result o IMM filter estimation
       xpred {}                 %all predicted states from every kalmann filter
       Ppred {}                 %all predicted covariances from every kalman filter
       mu {}                    %all model probabilities from every kalman filter
       
       xcons {}                 %state after consesus
       Pcons {}                 %covariance after consensus
       
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
            obj.sensed=[0,0];               %starts as [0,0]
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
            else
                obj.sensed=[99999,99999];
            end
        end
        
        function obj=checkchange(obj)
            if obj.change==1
                %if some sensor nearby has turned on then switch to idle
                if obj.state=="off"
                    obj.state="idle";
                end
                obj.change=0;
            elseif obj.change==-1
                %if some sensor nerby has turned off, and you see that no
                %other sensor nearby is on, then turn off
                a=checkvertex(obj.activevertex,obj.neighboors);
                if obj.state=="idle" && ~any(a,'all')
                    obj.state="off";
                end
                
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
            n=1;
            %again matlab sucks with all these temp stuff
            indic=obj.activevertex{1};
            lungstato=length(sensorgrid{indic(1),indic(2)}.xmix);
            wantxmix=zeros(lungstato,1);
            wantcovmix=zeros(lungstato,lungstato,1);
            
            wantxpred={};
            wantPpred={};
            wantmu={};
            for i=1:length(obj.neighboors)
                %temp sensor with neighboor's indices cause matlab sucks
                gigi=sensorgrid{obj.neighboors{i}(1),obj.neighboors{i}(2)};
                if gigi.state=="on"
                    %final state and covariance estimate
                    wantxmix(:,n)=gigi.xmix;
                    wantcovmix(:,:,n)=gigi.Pmix;
                    %all the kalman estimates
                    wantxpred{n}(:,:)=gigi.xpred;
                    wantPpred{n}(:,:,:)=gigi.Ppred;
                    wantmu{n}(:)=gigi.mu;
                    n=n+1;
                end
            end
            nummarkov=size(wantPpred{1},3);
            numsensors=length(wantPpred);
            %do WLS from all the data of the on neighboors
            
            %the final result of the IMM
            Cmix=[];
            for i=1:numsensors
                Cmix=blkdiag(Cmix,wantcovmix(:,:,i));
            end
            zmix=reshape(wantxmix,[lungstato*numsensors,1]);
            %H is gonna be a stacked matrix of identities since our measure
            %is exactly zmix
            Hmix=deblock(eye(lungstato*numsensors),lungstato); %gonna use this one also for pred
            [obj.xmix,obj.Pmix]=WLS(zmix,Cmix,Hmix);
            
            %kalman
            zpred=reshape(cell2mat(wantxpred),[numsensors*lungstato,nummarkov]);
            Cpred={};
            for j=1:nummarkov
                Cpred{j}=[];
                for i=1:numsensors
                    Cpred{j}=blkdiag(Cpred{j},wantPpred{i}(:,:,i));
                end
            end
            obj.mu=mean(reshape(cell2mat(wantmu),[numsensors,nummarkov]),1);
            for j=1:nummarkov
                [obj.xpred(:,j),obj.Ppred(:,:,j)]=WLS(zpred(:,j),Cpred{j},Hmix);
            end
        end
    end
    
end
