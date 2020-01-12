function [xkk,Pkk] = IMM(xprev,Pprev,z,Mat,Trans,Q,R,muk,a)
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
lungo=length(z);

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
        xmix(:,i)=xmix(:,i)+mudata(i,j)*xprev(:,j);
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
S=zeros(lungo,lungo,alto);
L=cell(zeros(1,alto));

for i=1:alto
    [xpred(:,i),Ppred(:,:,i),dz,S(:,:,i)]=kalman(Mat(i),xmix(:,i),a,z,Pmix(:,:,i),Q,R);
    L(i)=normpdf(dz,0,S).NLogL;
end

%% Model probability update
for i=1:alto
    muk(i)=mukk(i)*L(i);
end
muk=muk./(mukk*L');

%%Estimate Fusion

xkk=xpred*muk';
Pkk=zeros(alto);
for i=1:alto
    Pkk=Pkk+(Ppred(:,:,i)+(xkk-xpred(:,i))*(xkk-xpred(:,i))')*muk(i);
end

