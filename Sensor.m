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
       activevertex{}         %active graph indices
       
       Slistener {}              %element of another class to handle the events
       
       xmix {}                  %state result of IMM filter estimation
       Pmix {}                  %covariance result o IMM filter estimation
       xpred {}                 %all predicted states from every kalmann filter
       Ppred {}                 %all predicted covariances from every kalman filter
       mu {}                    %all model probabilities from every kalman filter
       
       xcons {}                 %state after consesus
       Pcons {}                 %covariance after consensus
       
    end
    
    events
        %signal sent to all the listeners of this sensor (all it's
        %neighboors) when there's a change of state from on to idle or
        %viceversa
        Cansense
        Cannotsense
    end
    
    methods
        
        %generating function
        function obj=Sensor(position,range,sigma,grid)
            obj.position=position;
            obj.range=range;
            obj.R=sigma;
            obj.state="idle";
            if nargin<4
                obj.ingrid=[0,0];
            else
                obj.ingrid=grid;
            end
            obj.sensed=[0,0];               %starts as [0,0]
        end
        
        %to check if in range of the object moving and to change status
        function obj=inRange(obj,x)
            if (obj.state=="on"||obj.state=="idle")
                distance=sqrt((obj.position-x')*(obj.position-x')');
                if distance>obj.range
                    obj.inrange=false;
                    notify(obj,'Cannotsense');          %event to notify
                    obj.state="idle";
                else
                    obj.inrange=true;
                    notify(obj,'Cansense');
                    obj.state="on";
                end
            else
                debugMsgObj("Something went wrong, off sensors shouldn't call inRange()",obj);
            end
        end
    
        %it measures distance and angle from the sensor
        function obj=sense(obj,x)
            %obj->sensore, x->column vector that stores the position of the
            %moving object
            inRange(obj,x);
            if obj.inrange
                obj.state=x'-obj.position;
                actualangle=atan2(obj.state(2),obj.state(1));
                obj.sensed=[sqrt(obj.state*obj.state');actualangle]+randn(2,1).*arrayfun(@(a)sqrt(a),diag(obj.R));
            else
                obj.sensed=[99999,99999];
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
            for i=1:length(obj.neighboors)
                %temp sensor with neighboor's indices
                gigi=sensorgrid{obj.neighboors{i}(1),obj.neighboors{i}(2)};
                if gigi.state=="on"
                    %final state and covariance estimate
                    wantxmix(:,end+1)=gigi.xmix;
                    wantcovmix(:,:,end+1)=gigi.Pmix;
                    %all the kalman estimates
                    wantxpred{end+1}(:,:)=gigi.xpred;
                    wantPpred{end+1}(:,:,:)=gigi.Ppred;
                    wantmu{end+1}(1,:)=gigi.mu;
                end
            end
            nummarkov=size(wantPpred{1},3);
            lungstato=size(wantPpred{1},2);
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
                for i=1:numsensors
                    Cpred{j}=blkdiag(Cpred{j},wantPpred{i}(:,:,i));
                end
            end
            obj.mu=mean(reshape(cell2mat(wantmu),[numsensors,nummarkov]),1);
            for j=1:nummarkov
                [obj.xpred(:,j),obj.Pred(:,:,j)]=WLS(zpred(:,j),Cpred{j},Hmix);
            end
        end
        
        
        %add the listener for both events
        function obj=addlistener(obj,source)
            %add the ability to sense both events can/cant sense sent from 
            addlistener1(source);
            addlistener2(source);
            obj.neighboors{end+1}=source.ingrid;
        end
        function obj=addlistener1(obj,source)
            listener(source,'Cansense',@Sensor.handleEvent1);
        end
        function obj=addlistener2(obj,source)
            listener(source,'Cannotsense',@Sensor.handleEvent2);
        end
    end
    
    %callback functions for events
    methods (Static)
        function obj=handleEvent1(obj,source)
            %event Cansense
            if obj.state=="off"
                obj.state="idle"; 
            end
        end
        function obj=handleEvent2(obj,source)
            %event Cannotsense
            a=checkvertex(obj.activevertex,obj.neighboors);
            if obj.state=="idle"&& ~any(a)
                obj.state="off";
            end
            
        end
    end
    
end
