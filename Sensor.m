classdef Sensor<handle
    properties (SetObservable = true)
       H {}                     %Measurement matrix (in this case linearized)
       range {}                 %how far can the sensor work
       R {}                     %diagonal cuz we cool
       
       sensed {}                %[distance,angle]
       
       position {}              %position of the sensor in the room
       ingrid {}                %indices in the grid
       
       inrange=true             %pretty selfexplanatory
       state {}                 %on/idle/off
       neighboors {}            %cell array with indices of listeners/sources
       
       listener {}              %element of another class to handle the events
       
       xmix {}                  %state result of IMM filter estimation
       Pmix {}                  %covariance result o IMM filter estimation
       xpred {}                 %all predicted states from every kalmann filter
       Ppred {}                 %all predicted covariances from every kalman filter
       mu {}                    %all model probabilities from every kalman filter
       
       xcons {}                 %state after consesus
       Pcons {}                 %covariance after consensus
       
    end
        events
        Cansense
        Cannotsense
    end
    methods
        function obj=Sensor(position,range,sigma,grid)   %generating function
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
        function obj=inRange(obj,x)
            distance=sqrt((obj.position-x')*(obj.position-x')');
            if distance>obj.range
                obj.inrange=false;
                notify(obj,'Cannotsense');          %event to notify

                %TODO add listeners, tell other sensors in the
                %neighboorhood that you are out of range, to maybe switch
                %between 3 states on, idle, off
                %so here check if all the other neighbors are not in range
                %then stay OFF, else switch to idle
            else
                obj.inrange=true;
                notify(obj,'Cansense');
                obj.state="on";
            end
        end
        function obj=sense(obj,x)
            %obj->sensore, x->column vector that stores the position of the
            %moving object
            %it measures distance and angle from the sensor, maybe do not
            %use sigma for angle but quanta ?
            inRange(obj,x);
            if obj.inrange
                obj.state=x'-obj.position;
                actualangle=atan2(obj.state(2),obj.state(1));
                obj.sensed=[sqrt(obj.state*obj.state');actualangle]+randn(2,1).*arrayfun(@(a)sqrt(a),diag(obj.R));
            else
                obj.sensed=[99999,99999];
            end
        end
        
       
        function obj=initializefromidle(obj,sensorgrid)
            %check which neighboors are on
            for i=1:length(obj.neighboors)
                gigi=sensorgrid{obj.neighboors{i}(1),obj.neighboors{i}(2)};
                if gigi.state=="on"
                    wantxmix(:,end+1)=gigi.xmix;
                    wantcovmix(:,:,end+1)=gigi.Pmix;
                    
                    wantxpred{end+1}(:,:)=gigi.xpred;
                    wantxpred{end+1}(:,:,:)=gigi.Ppred;
                    wantmu{end+1}(:,:)=gigi.mu;
                end
            end
            Cmix=blkdiag(wantcovmix(:,:,:));
            zmix=reshape(wantmix,[size(wantmix,1)*size(wantmix,2),1]);
            Hmix=reshape(eye(size(wantmix,1)*size(wantmix,2)),[size(wantmix,2),size(wantmix,1)]);
        end
        
        
        
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
    
    methods (Static)
        function obj=handleEvent1(obj,src)          %the source sensor states that it is in range
            if any(obj.neighboors{:}.state=="on")&&obj.state=="off"
                obj.state="idle";
            elseif obj.state=="idle"
                inRange(obj);   
            end
        end
        function handleEvent2(src)
            
        end
    end
    
end
