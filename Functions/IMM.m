function [xkk,Pkk,xpred,Ppred,muk1] = IMM(xprev,Pprev,z,Mat,Trans,Q,R,muk,u)
%xprev->previous iteration estimates, so if we have 5models it will be 5
%column vectors
%Pprev->same thing with covariance
%z->measurement
%Mat->all the models matices (A,B)
%Trans->Transition matrix for markovchain
%Q,R->Respectively model noise covariance/spectral density and measurement noise
%covariance
%muk->model probabilities at previous timestep, row vector
%u->acceleration intensity

alto=size(Trans,1);
lungo=length(z);
largo=size(xprev,1);

%% Reinitialization

mukk=muk*Trans;             %predicted model probabilities at k|k-1
muij=zeros(alto);         %mixing probabilities mu(i|j) at k-1|k-1    (11.6.6-7)
for j=1:alto
    for i=1:alto
        muij(i,j)=Trans(i,j)*muk(i)/mukk(j);
    end
end
xmix=zeros(largo,alto);                               %mixing estimate
for j=1:alto
    for i=1:alto
        xmix(:,j)=xmix(:,j)+muij(i,j)*xprev(:,i);
    end   
end

Pmix=zeros(largo,largo,alto);                    %mixing covariance
for j=1:alto
    for i=1:alto
        Pmix(:,:,j)=Pmix(:,:,j)+(Pprev(:,:,i)+(xprev(:,i)-xmix(:,j))*(xprev(:,i)-xmix(:,j))')*muij(i,j);
    end
end

%% Kalman stage
xpred=zeros(largo,alto);                      %
Ppred=zeros(largo,largo,alto);             %
S=zeros(lungo,lungo,alto);
L=zeros(1,alto);
dz=zeros(2,alto);

for i=1:alto
    [xpred(:,i),Ppred(:,:,i),dz(:,i),S(:,:,i)]=kalman(Mat(i,:),xmix(:,i),u,z,Pmix(:,:,i),Q,R);
    L(i)=mvnpdf(dz(:,i),0,S(:,:,i));
end

%% Model probability update
muk1=zeros(1,alto);
for i=1:alto
    muk1(i)=mukk(i)*L(i);
end
muk1=muk1./(mukk*L');

%%Estimate Fusion

xkk=xpred*muk1';
Pkk=zeros(largo);
for i=1:alto
    Pkk=Pkk+(Ppred(:,:,i)+(xkk-xpred(:,i))*(xkk-xpred(:,i))')*muk1(i);
end
