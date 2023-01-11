function [R, t]=ICPMatching(data1, data2)
%% found on GitHub
preError=0;%一つ前のイタレーションのerror値
dError=1000;%エラー値の差分
EPS=0.0001;%収束判定値
maxIter=100;%最大イタレーション数
count=0;%ループカウンタ

R=eye(2);%回転行列
t=zeros(2,1);%並進ベクトル

while ~(dError < EPS)
	count=count+1;
    
    [ii, error]=FindNearestPoint(data1, data2);%最近傍点探索
    [R1, t1]=SVDMotionEstimation(data1, data2, ii);%特異値分解による移動量推定
    %計算したRとtで点群とRとtの値を更新
    data2=R1*data2;
    data2=[data2(1,:)+t1(1) ; data2(2,:)+t1(2)];
    R = R1*R;
    t = R1*t + t1; 
    
    dError=abs(preError-error);%エラーの改善量
    preError=error;%一つ前のエラーの総和値を保存
    
    if count > maxIter %収束しなかった
        disp('Max Iteration');return;
    end
end
disp(['Convergence:',num2str(count)]);

function [index, error]=FindNearestPoint(data1, data2)
%data2に対するdata1の最近傍点のインデックスを計算する関数
m1=size(data1,2);
m2=size(data2,2);
index=[];
error=0;

for i=1:m1
    dx=(data2-repmat(data1(:,i),1,m2));
    dist=sqrt(dx(1,:).^2+dx(2,:).^2);
    [dist, ii]=min(dist);
    index=[index; ii];
    error=error+dist;
end

function [R, t]=SVDMotionEstimation(data1, data2, index)
%特異値分解法による並進ベクトルと、回転行列の計算

%各点群の重心の計算
M = data1; 
mm = mean(M,2);
S = data2(:,index);
ms = mean(S,2); 

%各点群を重心中心の座標系に変換
Sshifted = [S(1,:)-ms(1); S(2,:)-ms(2);];
Mshifted = [M(1,:)-mm(1); M(2,:)-mm(2);];

W = Sshifted*Mshifted';
[U,A,V] = svd(W);%特異値分解

R = (U*V')';%回転行列の計算
t = mm - R*ms;%並進ベクトルの計算

