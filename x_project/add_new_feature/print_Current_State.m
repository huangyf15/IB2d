function print_Current_State(current_time,xLag_present,yLag_present,F_fluid)
% only used to output the current state data
 [~,~,ComptMode_info,~,~,Reference_ID,~,~,~,~,~,~,~,~]...
              = give_Me_Propty_Topo_Geometry(0,0);
  C_ID = Reference_ID(1);
  Rotation_Flag = ComptMode_info(2);
  xC = xLag_present(C_ID);
  yC = yLag_present(C_ID);
  Fx_sum = sum(F_fluid(:,1));
  Fy_sum = sum(F_fluid(:,2));
  trajectory_fid = fopen('Output_Current_State.dat', 'a');
  if Rotation_Flag
    xArrow = 0.0;
    yArrow = 0.0;
    fprintf(trajectory_fid, ...
        '%1.16e %1.16e %1.16e %1.16e %1.16e %1.16e %1.16e\n', ...
        current_time, xC, yC, xArrow, yArrow, Fx_sum, Fy_sum);
  else
    fprintf(trajectory_fid, ...
        '%1.16e %1.16e %1.16e %1.16e %1.16e\n', ...
        current_time, xC, yC, Fx_sum, Fy_sum);
  end
  fclose(trajectory_fid);
end

