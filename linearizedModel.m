function [A,B,C] = linearizedModel(x, h)
  % Dynamics of the vehicle
  % states: x, y, psi, v, delta
  % inputs: a, s

  psi = x(3); % Orientation
  v = x(4); % Velocity
  delta = x(5); % Steering angle
  l = car.length_car; % vehicle length
  
  % Linearized Dynamics:
  A =  [1,  0, -h*sin(psi+delta) * v,   h*cos(psi+delta),   -h*sin(psi+delta) * v;
        0,  1, h*cos(psi+delta) * v,    h*sin(psi+delta),   h*cos(psi+delta) * v;
        0,  0, 1,                       h/l * sin(delta),   h*cos(delta) * v/l;
        0,  0, 0,                       1,                  0;
        0,  0, 0,                       0,                  1];
  
  B = h*[0, 0;
       0, 0;
       0, 0;
       1, 0;
       0, 1];
   
   C = [1, 0, 0, 0, 0;
       0, 1, 0, 0, 0];
end


