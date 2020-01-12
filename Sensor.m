classdef Sensor<handle
    properties
       H {}                     %Measurement matrix (in this case linearized)
       range {}                 %how far can the sensor work
       R {}                 %one for distance, the other for angular position
       sensed {}                %[distance,angle]
       position {}              %position of the sensor in the room
       inrange=true             %pretty selfexplanatory
       state {}                 %on/idle/off
    end
    methods
        function obj=Sensor(position,range,sigma)   %generating function
            obj.position=position;
            obj.range=range;
            obj.R=sigma;
            obj.sensed=[99999,99999];               %starts as offrange aka turned off
%           obj.quanta=[range/nticks(1),2*pi/nticks(2)];
        end
        function obj=inRange(obj,x)
            distance=sqrt((obj.position-x')*(obj.position-x')');
            if distance>obj.range
                obj.inrange=false;
%                 notify(obj,Cannotsense);          %event to notify

                %TODO add listeners, tell other sensors in the
                %neighboorhood that you are out of range, to maybe switch
                %between 3 states on, idle, off
                %so here check if all the other neighbors are not in range
                %then stay OFF, else switch to idle
            else
                obj.inrange=true;
%                 notify(obj,Cansense);

                %TODO same here
%                 if obj.state=="idle"
%                     obj.state="on";
%                 end
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
    end
%     events
%         Cansense
%         Cannotsense
%     end
    
end
