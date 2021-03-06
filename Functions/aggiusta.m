function dz = aggiusta(dzwrong)
%if the difference between z and zpred is 2pi that's equal to 0, the
%likelihood function doesn't know that and spits out bad results, so if
%we're near 2pi we just subtract it to make it near 0
    if dzwrong(2)>pi
        dz=[dzwrong(1);dzwrong(2)-2*pi];
    elseif dzwrong(2)<-pi
        dz=[dzwrong(1);dzwrong(2)+2*pi];
    else
        dz=dzwrong;
    end
end

