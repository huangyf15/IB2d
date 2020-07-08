
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: creates the CHANNEL_CHANNEL-EXAMPLE geometry and prints associated input files
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function generate_Cylinder()
  
  %
  % Define the structure's location and name
  %
  x0 = 0.6;                      % x-Center for cylinder
  y0 = 0.25;                     % y-Center for cylinder
  struct_name = 'cylinder';      % Name for .vertex, .spring, etc files.
  
  %
  % Call function to construct geometry
  %
  [xLag, yLag, ~, Stiffness_info, ~, ~, Fixed_Target_ID, Update_Target_ID,...
               Mass_ID, Skeleton_Spring_ID, Damped_Spring_ID,...
               Skeleton_Beam_ID, Locking_Beam_ID, Moderate_Beam_ID]...
           = give_Me_Propty_Topo_Geometry(x0,y0);
  k_fixed_target    = Stiffness_info(1);
  k_update_target   = Stiffness_info(2);
  k_Mass            = Stiffness_info(3);
  Mass              = Stiffness_info(4);
  k_skeleton_spring = Stiffness_info(5);
  k_damp            = Stiffness_info(6);
  b_damp            = Stiffness_info(7);
  k_skeleton_beam   = Stiffness_info(8);
  k_locking_beam    = Stiffness_info(9);
  k_moderate_beam   = Stiffness_info(10);
  
  %
  % Prints the structure points with different properties
  %
  
  % Prints .vertex file!
  print_Lagrangian_Vertices(xLag,yLag,struct_name);
  
  % Prints .target file!
  print_Lagrangian_Target_Pts(struct_name,...
                              k_fixed_target,k_update_target,...
                              Fixed_Target_ID,Update_Target_ID);
  % Prints .spring file!
  print_Lagrangian_Springs(xLag,yLag,struct_name,...
                           k_skeleton_spring,Skeleton_Spring_ID);
  % Prints .d_spring file!
  print_Lagrangian_Damped_Springs(xLag,yLag,struct_name,...
                                  k_damp,b_damp,Damped_Spring_ID);
  % Prints .beam file!
  print_Lagrangian_Beams(struct_name,...
                         k_skeleton_beam,k_locking_beam,k_moderate_beam,0.0,...
                         Skeleton_Beam_ID,Locking_Beam_ID,Moderate_Beam_ID);

  % Prints .mass file!
  print_Lagrangian_Mass_Pts(struct_name,k_Mass,Mass,Mass_ID);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: prints VERTEX points to a file called rubberband.vertex
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function print_Lagrangian_Vertices(xLag,yLag,struct_name)

    N = length(xLag);
    vertex_fid = fopen([struct_name '.vertex'], 'w');
    fprintf(vertex_fid, '%d\n', N );

    if N == 0
        error('ERROR: The num of Largrangian Vertices is zero!')
    end

    %Loops over all Lagrangian Pts.
    for s = 1:N
        X_v = xLag(s);
        Y_v = yLag(s);
        fprintf(vertex_fid, '%1.16e %1.16e\n', X_v, Y_v);
    end

    fclose(vertex_fid); 
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: prints Vertex points to a file called rubberband.vertex
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function print_Lagrangian_Target_Pts(struct_name,...
                              k_fixed_target,k_update_target,...
                              Fixed_Target_ID,Update_Target_ID)
    
    % k_fixed_target, k_update_target: penalty length

    N_rest = length(Fixed_Target_ID);
    N_update = length(Update_Target_ID);
    
    if N_rest+N_update > 0
      target_fid = fopen([struct_name '.target'], 'w');
      fprintf(target_fid, '%d\n', N_rest+N_update);

      %Loops over all Lagrangian Pts.
      for s = 1:N_rest
          fprintf(target_fid, '%d %1.16e\n', Fixed_Target_ID(s), k_fixed_target);
      end
      for s = 1:N_update
          fprintf(target_fid, '%d %1.16e\n', Update_Target_ID(s), k_update_target);
      end

      fclose(target_fid); 
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: prints SPRING points to a file called rubberband.spring
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function print_Lagrangian_Springs(xLag,yLag,struct_name,...
                          k_skeleton_spring,Skeleton_Spring_ID)

    % k_skeleton_spring: spring stiffness
    
    if mod(length(Skeleton_Spring_ID),2) ~= 0
      error('ERROR: Skeleton_Spring_ID/2 is not integer!')
    end
    N_skeleton = length(Skeleton_Spring_ID)/2;
    if N_skeleton > 0
      spring_fid = fopen([struct_name '.spring'], 'w');
      fprintf(spring_fid, '%d\n', N_skeleton);
      % springs between vertices
      for s = 1:N_skeleton
        fprintf(spring_fid, '%d %d %1.16e %1.16e\n',...
          Skeleton_Spring_ID(2*s-1), Skeleton_Spring_ID(2*s),...
          k_skeleton_spring,...
          sqrt((xLag(Skeleton_Spring_ID(2*s-1))-xLag(Skeleton_Spring_ID(2*s)))^2+...
               (yLag(Skeleton_Spring_ID(2*s-1))-yLag(Skeleton_Spring_ID(2*s)))^2));  
      end
      fclose(spring_fid);
    end

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: prints DAMPED SPRING points to a file called rubberband.d_spring
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function print_Lagrangian_Damped_Springs(xLag,yLag,struct_name,...
                                         k_damp,b_damp,Damped_Spring_ID)

    % k_damp: damping spring's stiffness
    % b_damp: damping coefficient
    
    if mod(length(Damped_Spring_ID),2) ~= 0
      error('ERROR: Damped_Spring_ID/2 is not integer!')
    end
    N_damped = length(Damped_Spring_ID)/2;
    if N_damped > 0
      d_spring_fid = fopen([struct_name '.d_spring'], 'w');
      fprintf(d_spring_fid, '%d\n', N_damped);
      % damped spring between vertices
      for s = 1:N_damped
        fprintf(d_spring_fid, '%d %d %1.16e %1.16e %1.16e\n',...
          Damped_Spring_ID(2*s-1), Damped_Spring_ID(2*s),...
          k_damp,...
          sqrt(...
            (xLag(Damped_Spring_ID(2*s-1))-xLag(Damped_Spring_ID(2*s)))^2+ ...
            (yLag(Damped_Spring_ID(2*s-1))-yLag(Damped_Spring_ID(2*s)))^2  ...
          ),...
          b_damp);  
      end
      fclose(d_spring_fid);
    end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: prints BEAM (Torsional Spring) points to a file called rubberband.beam
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function print_Lagrangian_Beams(struct_name,...
                       k_skeleton_beam,k_locking_beam,k_moderate_beam,C,...
                       Skeleton_Beam_ID,Locking_Beam_ID,Moderate_Beam_ID)
    
    % k_skeleton_beam, k_moderate_beam: beam stiffness
    % C: beam curvature
    
    if mod(length(Skeleton_Beam_ID),3) ~= 0
      error('ERROR: Skeleton_Beam_ID/3 is not integer!')
    end
    N_skeleton = length(Skeleton_Beam_ID)/3;
    if mod(length(Locking_Beam_ID),3) ~= 0
      error('ERROR: Locking_Beam_ID/3 is not integer!')
    end
    N_locking = length(Locking_Beam_ID)/3;
    if mod(length(Moderate_Beam_ID),3) ~= 0
      error('ERROR: Moderate_Beam_ID/3 is not integer!')
    end
    N_moderate = length(Moderate_Beam_ID)/3;
    
    if N_skeleton + N_locking + N_moderate > 0
      beam_fid = fopen([struct_name '.beam'], 'w');
      fprintf(beam_fid, '%d\n', N_skeleton + N_locking + N_moderate);
      for s = 1:N_skeleton
        fprintf(beam_fid, '%d %d %d %1.16e %1.16e\n',...
          Skeleton_Beam_ID(3*s-2),Skeleton_Beam_ID(3*s-1),Skeleton_Beam_ID(3*s),...
          k_skeleton_beam, C);  
      end
      for s = 1:N_locking
        fprintf(beam_fid, '%d %d %d %1.16e %1.16e\n',...
          Locking_Beam_ID(3*s-2),Locking_Beam_ID(3*s-1),Locking_Beam_ID(3*s),...
          k_locking_beam, C);  
      end
      for s = 1:N_moderate
        fprintf(beam_fid, '%d %d %d %1.16e %1.16e\n',...
          Moderate_Beam_ID(3*s-2),Moderate_Beam_ID(3*s-1),Moderate_Beam_ID(3*s),...
          k_moderate_beam, C);  
      end
      fclose(beam_fid); 
    end

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTION: prints TARGET points to a file called struct_name.mass
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
function print_Lagrangian_Mass_Pts(struct_name,k_Mass,Mass,Mass_ID)

    N_Mass = length(Mass_ID);

    if N_Mass > 0
      mass_fid = fopen([struct_name '.mass'], 'w');
      fprintf(mass_fid, '%d\n', N );
      for s = 1:N_Mass
          fprintf(mass_fid, '%d %1.16e %1.16e\n', Mass_ID(s), k_Mass, Mass);
      end
      fclose(mass_fid); 
    end
    
