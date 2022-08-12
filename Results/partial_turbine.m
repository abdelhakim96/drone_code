turbine_x=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_x.txt');
turbine_y=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_y.txt');
turbine_z=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_z.txt');


turbine_nx=dlmread('Data/normals/nx_inter.txt');
turbine_ny=dlmread('Data/normals/ny_inter.txt');
turbine_nz=dlmread('Data/normals/nz_inter.txt');







turbine_y=turbine_y+22;
turbine_x=turbine_x+80;
turbine_z=turbine_z-65;

length(turbine_z)
point=28800;

plot3(turbine_x(point:end),turbine_y(point:end),turbine_z(point:end))




scaling_xy=5;
scaling_z=15;








blade_x=turbine_y(point:end)/scaling_xy;
blade_y=turbine_x(point:end)/scaling_xy;
blade_y=blade_y+2;

blade_z=turbine_z(point:end)/scaling_z;


blade_nx=turbine_ny(point:end);
blade_ny=turbine_nx(point:end);
blade_nz=turbine_nz(point:end);






add_x=[0:0.01:2];
add_x(:)=blade_x(1);
add_y=[0:0.01:2];
add_x(:)=blade_y(1);


add_z=[0:0.01:2];


add_nx=[0:0.01:2];
add_ny=[0:0.01:2];
add_nz=[0:0.01:2];

writematrix(blade_x, "blade_x.txt");
writematrix(blade_y, "blade_y.txt");
writematrix(blade_z, "blade_z.txt");


writematrix(blade_nx, "blade_nx.txt");
writematrix(blade_ny, "blade_ny.txt");
writematrix(blade_nz, "blade_nz.txt");



max_x=max(abs(blade_x));
max_y=max(abs(blade_y));
max_z=max(abs(blade_z));
min_z=min(abs(blade_z));

