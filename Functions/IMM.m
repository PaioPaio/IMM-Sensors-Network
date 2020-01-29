function [xkk,Pkk,xpred,Ppred,muk1,muij] = IMM(xprev,Pprev,offset,z,Mat,Trans,Q,R,muk,u,caso,delta)
%xprev->previous iteration estimates, so if we have 5models it will be 5
%column vectors
%Pprev->same thing with covariance
%offset->position of sensor from where you measured
%z->measurement
%Mat-> case 1: all the models matices (A,B,)/ case 2: radius of the wheel
%Trans->Transition matrix for markovchain
%Q,R->Respectively model noise covariance/spectral density and measurement noise
%covariance
%muk->model probabilities at previous timestep, row vector
%u->acceleration intensity

alto=size(Trans,1); %markov states number
lungo=length(z);    %length of measurement
largo=size(xprev,1);    %length of state

%% Reinitialization

mukk=muk*Trans;             %predicted model probabilities at k|k-1 given k-1|k-1
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
dz=zeros(lungo,alto);

for i=1:alto
    if caso==1
        [xpred(:,i),Ppred(:,:,i),dz(:,i),S(:,:,i)]=kalman(Mat(i,:),xmix(:,i),u,z,Pmix(:,:,i),Q,R,caso,offset,delta,i);
    else
        [xpred(:,i),Ppred(:,:,i),dz(:,i),S(:,:,i)]=kalman(Mat,xmix(:,i),u,z,Pmix(:,:,i),Q,R,caso,offset,delta,i);
    end
    %% to avoid error => threshold 
    if  rcond(S(:,:,i))<=0.00000001
        L(i)=0.001;
    else
        L(i)=mvnpdf(dz(:,i),0,S(:,:,i));
    end
    
end

%% Model probability update
muk1=mukk.*L;
muk1=muk1./(mukk*L');

%% Estimate Fusion

xkk=xpred*muk1';
Pkk=zeros(largo);
for i=1:alto
    Pkk=Pkk+(Ppred(:,:,i)+(xkk-xpred(:,i))*(xkk-xpred(:,i))')*muk1(i);
end
if rcond(Pkk)<0.0001
    "OMEGALUL"
end
end
