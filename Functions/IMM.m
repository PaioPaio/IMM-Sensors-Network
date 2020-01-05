function [] = IMM(xprev,Pprev,z,Mat,Trans,Q,R,muk,a)
%xprev->previous iteration estimates, so if we have 5models it will be 5
%vectors
%Pprev->same thing with covariance
%z->measurement
%Mat->all the models matices (A,B)
%Trans->Transition matrix for markovchain
%Q,R->Respectively model noise covariance/spectral density and measurement noise
%covariance
%muk->model probabilities at previous timestep

alto=size(1,Trans);

%% Reinitialization

mukk=muk*Trans;             %predicted model probabilities at k|k-1
mudata=zeros(alto);         %mixing probabilities mu(i|j) at k-1
for i=1:alto
    for j=1:alto
        mudata(i,j)=Trans(i,j)*muk(i)/mukk(j);
    end
end
xmix=zeros(alto);                               %mixing estimate
for i=1:alto
    for j=1:alto
        xmix(:,i)=xmin(:,i)+mudata(i,j)*xprev(:,j);
    end   
end

Pmix=zeros(alto,alto,alto);                    %mixing covariance
for i=1:alto
    for j=1:alto
        Pmix(:,:,i)=Pmix(:,:,i)+(Pprev(:,:,j)+(xmix(j)-xprev(i))*(xmix(j)-xprev(i))')*mudata(i,j);
    end
end

%% Kalman stage
xpred=zeros(alto);                      %
Ppred=zeros(alto,alto,alto);             %

for i=1:alto
    [xpred(:,i),Ppred(:,i)]=kalman(Mat(i),xmix(:,i),a,z,Pmix(:,:,i),Q,R);
end

%% Model probability update

end

