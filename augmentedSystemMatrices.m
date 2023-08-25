function [AA,BB,CC, QQ, RR,SS,PP,II] = augmentedSystemMatrices(A,B,C, Q, R,S, N)

% get dimensions
[n,m]=size(B);
[r,~]=size(C);


% allocate
AA=zeros((N+1)*n,n);
BB=zeros((N+1)*n,N*m);
CC=zeros((N+1)*r,(N+1)*n);
QQ=zeros((N+1)*r,(N+1)*r);
RR=zeros(N*m,N*m);
SS=zeros(N*m,N*m);
PP=[eye(m);zeros((N-1)*m,m)];
II=eye(m*N);

% compute matrices
for k=0:N
    AA(k*n+1:(k+1)*n,:)=A^k;
    for j=0:k-1
        BB(k*n+1:(k+1)*n,j*m+1:(j+1)*m)=A^(k-j-1)*B;
    end
    CC(k*r+1:(k+1)*r,k*n+1:(k+1)*n)=C;

    if k<N
        QQ(k*r+1:(k+1)*r,k*r+1:(k+1)*r)=Q;
        RR(k*m+1:(k+1)*m,k*m+1:(k+1)*m)=R;
        if k>0
              SS(k*m+1:(k+1)*m,(k-1)*m+1:k*m)=eye(m);
        end
    else
        QQ(k*r+1:(k+1)*r,k*r+1:(k+1)*r)=S;
    end
end
end