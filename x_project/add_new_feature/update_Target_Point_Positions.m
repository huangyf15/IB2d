
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: updates the target point positions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function targets = update_Target_Point_Positions(dt,current_time,targets, ...
                       xLag_prev_dti2,yLag_prev_dti2,...
                       xLag_present,yLag_present,...
                       xLag_next_dti2,yLag_next_dti2,...
                       F_fluid)

  % % rigid motion equations
  %  let d(xC,yC,theta) = (xC,yC,theta) - (xC0,yC0,theta0), then
  %     /  mr * DDdtheta + bt * Ddtheta + kt * dtheta = M_sum
  %    |   mt * DDdxC + bx * DdxC + kx * dyC = Fx_sum
  %     \  mt * DDdyC + by * DdyC + ky * dyC = Fy_sum
  %  where F_sum = F_fluid + F_impel
  %
  % % local index with Target Pts.
  %  IDs = targets(:,1);                % Stores Lag-Pt IDs in col vector
  %  xPts= targets(:,2);                % Original x-Values of x-Target Pts.
  %  yPts= targets(:,3);                % Original y-Values of y-Target Pts.
  %  kStiffs = targets(:,4);            % Stores Target Stiffnesses 
  %
  % % global index with all Lag Pts.
  %  Fx_fluid = F_Lag(:,1)              % x-Force on the Lag Pts.
  %  Fy_fluid = F_Lag(:,2)              % y-Force on the Lag Pts.
  %
  
  %
  % get Reference_ID, RigidBody_info and Lag Pts' initial position at origin
  %
  
  [updated_xLag,updated_yLag,ComptMode_info, ~, RigidBody_info,...
                Reference_ID, ~, Update_Target_ID,~,~,~,~,~,~]...
              = give_Me_Propty_Topo_Geometry(0,0);
  Rotation_Flag = ComptMode_info(1);
  RigidCoupling_Flag = ComptMode_info(2);
  mt = RigidBody_info(1);
  bt = RigidBody_info(2);
  kt = RigidBody_info(3);
  mr = RigidBody_info(4);
  bx = RigidBody_info(5);
  kx = RigidBody_info(6);
  by = RigidBody_info(7);
  ky = RigidBody_info(8);
  A_impel = RigidBody_info(9);
  f_impel = RigidBody_info(10);
  C_ID = Reference_ID(1);
  
  %
  % deal with the rotation first
  %
 
  if Rotation_Flag
    theta0 = 0.0;
    dtheta = 0.0;
    M_sum = 0.0;
    dtheta_new = please_Compute_Rigid_Motion(dtheta,Ddtheta,M_sum,mr,bt,kt,dt);
    theta_new = theta0 + dtheta_new;
    [updated_xLag,updated_yLag] = rotate_rigid_body(...
          updated_xLag,updated_yLag,theta_new,Update_Target_ID);
  end
  
  %
  % then do the translation
  %
  
  xC0 = targets(C_ID,2);
  yC0 = targets(C_ID,3);
  dxC = xLag_present(C_ID) - xC0;
  dyC = yLag_present(C_ID) - yC0;
  vxC = (xLag_next_dti2-xLag_prev_dti2)/dt;
  vyC = (yLag_next_dti2-yLag_prev_dti2)/dt;
  [Fx_impel, Fy_impel] = give_Me_Impelling_Force(current_time,A_impel,f_impel);
  if RigidCoupling_Flag
    Fx_fluid = sum(F_fluid(:,1));
    Fy_fluid = sum(F_fluid(:,2));
  else
    Fx_fluid = 0.0;
    Fy_fluid = 0.0;
  end
  Fx_sum = Fx_fluid + Fx_impel;
  Fy_sum = Fy_fluid + Fy_impel;
  dxC_new = 0;
%   dxC_new = please_Compute_Rigid_Motion(dxC, vxC, Fx_sum, mt, bx, kx, dt);
  dyC_new = please_Compute_Rigid_Motion(dyC, vyC, Fy_sum, mt, by, ky, dt);
  updated_xLag = updated_xLag + xC0 + dxC_new;
  updated_yLag = updated_yLag + yC0 + dyC_new;
    
  %
  % update the positions of target points
  %
  
  for i = 1:size(targets,1)
    if ismember(targets(i,1),Update_Target_ID)
      targets(i,2) = updated_xLag(targets(i,1));
      targets(i,3) = updated_yLag(targets(i,1));
    end
  end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: update the location of the rigid body
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function xdt = please_Compute_Rigid_Motion(x0, Dx0, F, m, k, b, dt)
  
  % Used 4-order Runge-Kutta method to get x(dt),
  %   where x satisfies the following problem
  %    m * DDx + b * Dx + k * x = F, x(0) = x0, Dx0 = Dx0
  % - let y = Dx, then we deduce that
  %    / Dx =     0 * x -   1 * y +   0 =: ax * x + bx * y + cx
  %    \ Dy = - k/m * x - b/m * y - F/m =: ay * x + by * y + cy
  
  N = 100; dt0 = dt/N;
  ax = 0;    bx = 1;    cx = 0;
  ay = -k/m; by = -b/m; cy = -F/m;
  x = x0; y = Dx0;
  for i = 1:N
    Kx1 = ax*(x)             + bx*(y)             + cx;
    Kx2 = ax*(x + dt0*Kx1/2) + bx*(y + dt0*Kx1/2) + cx;
    Kx3 = ax*(x + dt0*Kx2/2) + bx*(y + dt0*Kx2/2) + cx;
    Kx4 = ax*(x + dt0*Kx3)   + bx*(y + dt0*Kx3)   + cx;
    dx0 = (Kx1 + 2*Kx2 + 2*Kx3 + Kx4)*dt0/6;
    Ky1 = ay*(x)             + by*(y)             + cy;
    Ky2 = ay*(x + dt0*Ky1/2) + by*(y + dt0*Ky1/2) + cy;
    Ky3 = ay*(x + dt0*Ky2/2) + by*(y + dt0*Ky2/2) + cy;
    Ky4 = ay*(x + dt0*Ky3)   + by*(y + dt0*Ky3)   + cy;
    dy0 = (Ky1 + 2*Ky2 + 2*Ky3 + Ky4)*dt0/6;
    x = x + dx0;
    y = y + dy0;
  end
  xdt = x;

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: creates the impelling force at certain amplitude and frequency
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 function [Fx_forced, Fy_forced] = give_Me_Impelling_Force(current_time,A,f)
   Fx_forced = 0;
   Fy_forced = A*cos(2*pi*f*current_time);

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: rotates the target point positions according to the reference vector
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xLag,yLag] = rotate_rigid_body(xLag,yLag,theta,Update_Target_ID)

  N_update = length(Update_Target_ID);
  cost = cos(theta);
  sint = sin(theta);
  rotA = [cost -sint; sint cost];
  for i = 1:N_update
    newPosition = rotA*[xLag(Update_Target_ID(i));yLag(Update_Target_ID(i))];
    xLag(Update_Target_ID(i)) = newPosition(1);
    yLag(Update_Target_ID(i)) = newPosition(2);
  end




