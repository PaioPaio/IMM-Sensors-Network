classdef Sensor<handle
    properties
       H {}                     %Measurement matrix (in this case linearized)
       range {}                 %how far can the sensor work
       sigma {}                 %one for distance, the other for angular position
       sensed {}                %[distance,angle]
       position {}              %position of the sensor in the room
       inrange=true             %pretty selfexplanatory
       state {}                 %[x,y]
    end
    methods
        function obj=Sensor(position,range,sigma)
            obj.position=position;
            obj.range=range;
            obj.sigma=sigma;
            obj.sensed=[99999,99999];
%           obj.quanta=[range/nticks(1),2*pi/nticks(2)];
        end
        function obj=inRange(obj,x)
            distance=sqrt((obj.position-x')*(obj.position-x')');
            if distance>obj.range
                obj.inrange=false;
%                 notify(obj,Cannotsense);
            else
                obj.inrange=true;
%                 notify(obj,Cansense);
            end
        end
        function obj=sense(obj,tiziochesimuove)
            %it measures distance and angle from the sensor, maybe do not
            %use sigma for angle but quanta ?
            inRange(obj,tiziochesimuove);
            if obj.inrange
                obj.state=tiziochesimuove'-obj.position;
                actualangle=atan2(obj.state(2),obj.state(1));
                obj.sensed=[sqrt(obj.state*obj.state');actualangle]+randn(2,1).*diag(obj.sigma);
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
