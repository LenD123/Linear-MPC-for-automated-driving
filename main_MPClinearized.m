%%  Tracking MPC for automated driving (linearized kinematic single track car model)
clc; clear; close all;

%% Setup
% states: 
% x     [m]         x-Pos, 
% y     [m]         y-Pos, 
% psi   [rad]       yaw-angle, 
% v     [m /s]      velocity, 
% delta  [rad]      steering angle

% inputs: 
% a     [m /s^2]    accerleration, 
% s     [rad /s]    delta_dot (steering rate)

n = 5; m = 2;
h = 0.05; % discretization constant

% Prediction horizon
N = 15;

% Constraints to the vehicle model
xLb = [-inf; -inf; -inf; -1; -2]; 
xUb = [inf; inf; inf; 3; 2];
uLb = [-1; -2.5];
uUb = [2;2.5];

% Weighting factors for OCP
Q = 100*eye(2);
R = eye(2);
S = 10*Q;

%% Simulation
% Load and Plot race track
[ref, kEnd] = plotTrack(N, 0, 1);

%%

% Initialization of states, inputs and outputs
% States:
x0 = [ref(1,1); ref(2,1); 0; 0;0];
x0(3) = atan2( (ref(2,2)-ref(2,1)) , (ref(1,2)-ref(1,1))); %align with track center
X = [x0,zeros(n,kEnd)];
dX = X;
dX(:,1) = X(:,1) - [0;0;X(3,1);0;X(5,1)];
XPred = {};

% Outputs:
YRef = ref; refpl=[];
YRefN = zeros(2*(N+1),1);
YPred = {};

% Inputs
U = zeros(m,kEnd);
uPre = [0; 0];

for k = 1:kEnd 
    % Build up reference prediction path to be followed
    YRefN(1:2:2*N+1) = YRef(1,k:k+N)';
    YRefN(2:2:2*N+2) = YRef(2,k:k+N)';
   
    % Linearize model and build augmented Matrices every time step
    [A,B,C] = linearizedModel(X(:,k), h); 
    [AA,BB,CC,QQ, RR,SS,PP,II] = augmentedSystemMatrices(A,B,C,Q,R,S,N);
 
      % QP matrices
      H = 2*(BB'*CC'*QQ*CC*BB +(II-SS)'*RR*(II-SS));
      FT = 2*[AA'*CC'*QQ*CC*BB; -QQ*CC*BB; -PP'*RR*(II-SS)];
      G = [eye(N*m);-eye(N*m);BB;-BB];
      E = [zeros(N*m,n);zeros(N*m,n);-AA;AA];
      EE = [E, zeros(2*(N*m+(N+1)*n),(N+1)*2), zeros(2*(N*m+(N+1)*n),m)];

      % Constraints to the vehicle model, updated each step for delta states
      dxLb = xLb - [0;0;X(3,k);0;X(5,k)];
      dxUb = xUb - [0;0;X(3,k);0;X(5,k)];
      [ULb,UUb,XLb,XUb] = augmentedConstraintVectors(uLb,uUb,dxLb,dxUb,N);
      de = [UUb;-ULb;XUb;-XLb];
      xi = [dX(:,k);YRefN; uPre]; % augmented state for tracking
    
    % Solve quadratic program and extract the first m inputs
    zOpt = quadprog(H,xi'*FT,G,EE*xi+de);
    U(:,k) = zOpt(1:m);
    uPre = U(:,k);
    
    % Apply inputs to nonlinear system
    X(:,k+1) = fdyn( X(:,k), U(:,k), h );
    dX(:,k+1) = X(:,k+1) - [0;0;X(3,k+1);0;X(5,k+1)];
    
    % predicted states of horizon N+1, uses delta coordinates for x3 and x5
    dXPred{k} = AA*dX(:,k)+BB*zOpt; 
    
    % Evaluate output
    Y(:,k) = C*dX(:,k);
    
    % The predicted vehicle positions will be shown as a red line in the plot
    YPred{k} = CC*dXPred{k};

    
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualising vehicle on the road
    [pred(k+1), t(:,k+1)] = visuals(X(:,k), YPred{k}, N);
    delete(pred(k)); delete(t(:,k)); delete(refpl);
    refpl = plot(YRef(1, k+N), YRef(2,k+N), 'ko' );
    pause(0.01)
end
delete(pred(k+1))
