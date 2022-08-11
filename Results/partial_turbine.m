turbine_x=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_x.txt');
turbine_y=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_y.txt');
turbine_z=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_z.txt');


turbine_nx=dlmread('Data/normals/nx_inter.txt');
turbine_ny=dlmread('Data/normals/nx_inter.txt');
turbine_nz=dlmread('Data/normals/nx_inter.txt');



turbine_y=turbine_y+18;
turbine_x=turbine_x+80;
turbine_z=turbine_z-74;

length(turbine_z)
point=28600;

plot3(turbine_x(point:end),turbine_y(point:end),turbine_z(point:end))




scaling_xy=4;
scaling_z=10;

blade_x=turbine_y(point:end)/scaling_xy;
blade_y=turbine_x(point:end)/scaling_xy;
blade_z=turbine_z(point:end)/scaling_z;


blade_nx=turbine_ny(point:end);
blade_ny=turbine_nx(point:end);
blade_nz=turbine_nz(point:end);



writematrix(blade_x, "blade_x.txt");
writematrix(blade_y, "blade_y.txt");
writematrix(blade_z, "blade_z.txt");


writematrix(blade_nx, "blade_nx.txt");
writematrix(blade_ny, "blade_ny.txt");
writematrix(blade_nz, "blade_nz.txt");



max_x=max(abs(blade_x));
max_y=max(abs(blade_y));
max_z=max(abs(blade_z));


