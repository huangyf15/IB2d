
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: creates the Lagrangian structure geometry for cylinder
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [xLag,yLag, ComptMode_info, Stiffness_info, RigidBody_info,...
                     Reference_ID, Fixed_Target_ID, Update_Target_ID,...
                     Mass_ID, Skeleton_Spring_ID, Damped_Spring_ID,...
                     Skeleton_Beam_ID, Locking_Beam_ID, Moderate_Beam_ID]...
              = give_Me_Propty_Topo_Geometry(x0,y0)
  %
  % Grid Parameters (MAKE SURE MATCHES IN input2d !!!)
  %
  Nx =  256;       % # of Eulerian Grid Pts. in x-Direction (MUST BE EVEN!!!)
  Ny =  256;       % # of Eulerian Grid Pts. in y-Direction (MUST BE EVEN!!!)
  Lx = 1.0;        % Length of Eulerian Grid in x-Direction
  Ly = 1.0;        % Length of Eulerian Grid in y-Direction
  
  % Immersed Structure Geometric / Dynamic Parameters %
  ds= min(Lx/(2*Nx),Ly/(2*Ny));  % Lagrangian spacing
  r = 0.05;                      % radius of the cylinder (Re_D = 100 U)

  %
  % Define system parameters
  %
  
  % Determine the computation mode
  Rotation_Flag = 0;             % determine whether the object could rotate
  RigidCoupling_Flag = 0;        % determine whether fluid impels the object
  Plot_Flag = 0;                 % determine whether to plot the schematic
  
  % Define the characteristic parameters of the rigid body (requires m > 0)
  % - See update_Target_Point_Positions.m
  mt = 1.0; bt = 0.0; kt = 1.0;
  mr = 1.0;
  bx = 0.0; kx = 1.0e3;
  by = 0.0; ky = 1.0e3;
  A_impel = 1.0; f_impel = 1000.0;
  
  % Define the characteristic parameters of the muscle structure
  % - See generate_Cylinder.m 
  k_fixed_target = 0;
  k_update_target = 4e6;
  k_locking_beam = 0;
  k_damp = 0; b_damp = 0;
  k_skeleton_beam = 0;
  k_skeleton_spring = 0;
  k_moderate_beam = 0;
  k_Mass = 0; Mass = 0;
  
  %
  % Generate the property arrays, geometry and topology
  %
  
  % Get the geometry
  [xLag,yLag] = give_Me_Geometry(ds,r,x0,y0);
  
  % Get the topology
  [Reference_ID, Fixed_Target_ID, Update_Target_ID, ...
          Mass_ID, Skeleton_Spring_ID, Damped_Spring_ID, ...
          Skeleton_Beam_ID, Locking_Beam_ID, Moderate_Beam_ID] ...
       = give_Me_Topology(length(xLag));
  
  % Integrate the discrete info
  ComptMode_info = [Rotation_Flag RigidCoupling_Flag];
  Stiffness_info = [k_fixed_target k_update_target ...
            k_Mass Mass k_skeleton_spring k_damp b_damp ...
            k_skeleton_beam k_locking_beam k_moderate_beam];
  RigidBody_info = [mt bt kt mr bx kx by ky A_impel f_impel];  
  
  %
  % Plot Geometry to test
  %
  if (Plot_Flag)
    plot_Geometry_to_test(xLag,yLag,...
             Reference_ID, Fixed_Target_ID, Update_Target_ID,...
             Skeleton_Spring_ID, Damped_Spring_ID,...
             Skeleton_Beam_ID, Locking_Beam_ID, Moderate_Beam_ID);
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: create the topology of the immersed geometry
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
function [Reference_ID, Fixed_Target_ID, Update_Target_ID, ...
          Mass_ID, Skeleton_Spring_ID, Damped_Spring_ID, ...
          Skeleton_Beam_ID, Locking_Beam_ID, Moderate_Beam_ID] ...
        = give_Me_Topology(N_LagPts)
  %
  % Define the Lag Pts' types in IDs
  % 1. AT LEAST ONE Reference_ID for recording the object's state
  % 2. ANY Lag Pts should be included in AT LEAST ONE type of ID
  %
  
  Reference_ID = 1;
  Fixed_Target_ID = [];
  Update_Target_ID = 1:N_LagPts;
  Mass_ID = [];
  Skeleton_Spring_ID = [];
  Damped_Spring_ID = [];
  Skeleton_Beam_ID = [];
  Locking_Beam_ID = [];
  Moderate_Beam_ID = [];
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: create the geometry of immersed geometry
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
function [xLag, yLag] = give_Me_Geometry(ds,r,x0,y0)

  % add the points on the circle
  dtheta = ds/(2*r);
  theta = 0; i = 1;
  while theta < 2*pi
     xLag(i) = x0 - r*cos(theta);
     yLag(i) = y0 - r*sin(theta);
     theta = theta + dtheta;
     i = i+1;
  end
  % add the reference points
  xLag = [x0 xLag];
  yLag = [y0 yLag];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: plot the immersed geometry to test
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

function plot_Geometry_to_test(xLag,yLag,...
             Reference_ID, Fixed_Target_ID, Update_Target_ID,...
             Skeleton_Spring_ID, Damped_Spring_ID,...
             Skeleton_Beam_ID, Locking_Beam_ID, Moderate_Beam_ID)
  plot(xLag,yLag,'k.'); hold on;
  title(['Schematic of the system']);
  ylim([0.0, 0.5]);
  xlim([0.0, 2.0]);
  axis equal;
  xlabel('x (m)'); ylabel('y (m)');
%   for i = 1:length(xLag)
%     text(xLag(i)+0.0005,yLag(i),num2str(i),'FontSize',5);
%   end
  % - Moderate_Beam_ID
  if mod(length(Moderate_Beam_ID),3) ~= 0
    error('ERROR: Moderate_Beam_ID/3 is not integer!')
  end
  for i = 1:length(Moderate_Beam_ID)/3
    plot([xLag(Moderate_Beam_ID(3*i-2)),xLag(Moderate_Beam_ID(3*i-1)),...
                                       xLag(Moderate_Beam_ID(3*i)) ],...
         [yLag(Moderate_Beam_ID(3*i-2)),yLag(Moderate_Beam_ID(3*i-1)),...
                                       yLag(Moderate_Beam_ID(3*i)) ],...
        'g-','LineWidth',0.75);
  end
  % - Skeleton_Spring_ID
  if mod(length(Skeleton_Spring_ID),2) ~= 0
    error('ERROR: The length of Skeleton_Spring_ID is not EVEN!')
  end
  for i = 1:length(Skeleton_Spring_ID)/2
    plot([xLag(Skeleton_Spring_ID(2*i-1)),xLag(Skeleton_Spring_ID(2*i))],...
         [yLag(Skeleton_Spring_ID(2*i-1)),yLag(Skeleton_Spring_ID(2*i))],...
        'y-');
  end
  % - Skeleton_Beam_ID
  if mod(length(Skeleton_Beam_ID),3) ~= 0
    error('ERROR: Skeleton_Beam_ID/3 is not integer!')
  end
  for i = 1:length(Skeleton_Beam_ID)/3
    plot([xLag(Skeleton_Beam_ID(3*i-2)),xLag(Skeleton_Beam_ID(3*i-1)),...
                                       xLag(Skeleton_Beam_ID(3*i)) ],...
         [yLag(Skeleton_Beam_ID(3*i-2)),yLag(Skeleton_Beam_ID(3*i-1)),...
                                       yLag(Skeleton_Beam_ID(3*i)) ],...
        'k--');
  end
  % - Locking_Beam_ID
  if mod(length(Locking_Beam_ID),3) ~= 0
    error('ERROR: Locking_Beam_ID/3 is not integer!')
  end
  for i = 1:length(Locking_Beam_ID)/3
    plot([xLag(Locking_Beam_ID(3*i-2)),xLag(Locking_Beam_ID(3*i-1)),...
                                       xLag(Locking_Beam_ID(3*i)) ],...
         [yLag(Locking_Beam_ID(3*i-2)),yLag(Locking_Beam_ID(3*i-1)),...
                                       yLag(Locking_Beam_ID(3*i)) ],...
        'b-','LineWidth',0.75);
  end
  % - Damped_Spring_ID
  if mod(length(Damped_Spring_ID),2) ~= 0
    error('ERROR: The length of Damped_Spring_ID is not EVEN!')
  end
  for i = 1:length(Damped_Spring_ID)/2
    plot([xLag(Damped_Spring_ID(2*i-1)),xLag(Damped_Spring_ID(2*i))],...
         [yLag(Damped_Spring_ID(2*i-1)),yLag(Damped_Spring_ID(2*i))],...
        'r--','LineWidth',0.75);
  end
  % - Reference_ID, Fixed_Target_ID and Update_Target_ID
  plot(xLag(Update_Target_ID),yLag(Update_Target_ID),'g*');
  plot(xLag(Fixed_Target_ID),yLag(Fixed_Target_ID),'b*');
  plot(xLag(Reference_ID),yLag(Reference_ID),'r*');
  hold off




   
 