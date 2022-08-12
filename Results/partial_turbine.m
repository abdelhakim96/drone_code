
close all
clear all
clc
wp=dlmread('/home/hakim/drone_code/Results/Data/WP_trajectory/wp_inter.txt');




turbine_x=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_x.txt');
turbine_y=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_y.txt');
turbine_z=dlmread('Data/Point_to_View_Trajectory/mp_d1cm_interp_z.txt');


turbine_nx=dlmread('Data/normals/nx_inter.txt');
turbine_ny=dlmread('Data/normals/ny_inter.txt');
turbine_nz=dlmread('Data/normals/nz_inter.txt');





a=22;
b=80;
c=-65;
d=2;


turbine_y=turbine_y+a;
turbine_x=turbine_x+b;
turbine_z=turbine_z+c;






length(turbine_z)
point=28800;

%plot3(turbine_x(point:end),turbine_y(point:end),turbine_z(point:end))



path_x=wp(point:end,1)+a;
path_y=wp(point:end,2)+b;
path_z=wp(point:end,3)+c;
path_psi=wp(point:end,4);




scaling_xy=5;
scaling_z=15;







blade_x=turbine_y(point:end)/scaling_xy;
blade_y=turbine_x(point:end)/scaling_xy;
blade_y=blade_y+d;
blade_z=turbine_z(point:end)/scaling_z;

path=zeros(length(path_z),4);
path(:,1)=path_x/scaling_xy;
path(:,2)=path_y/scaling_xy;
path(:,3)=path_z+d;
path(:,3)=path(:,3)/scaling_z;
path(:,4)=path_psi;



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


path(:,1)=blade_x+0.5*blade_nx;
path(:,2)=blade_y+0.5*blade_ny;
path(:,3)=blade_z;
path(:,4)=path_psi;


writematrix(path, "path.txt");






max_x=max(abs(blade_x));
max_y=max(abs(blade_y));
max_z=max(abs(blade_z));
min_z=min(abs(blade_z));
hold on



wp_x=blade_x+0.5*blade_nx;
wp_y=blade_y+0.5*blade_ny;
wp_z=blade_z;
figure 
plot3(blade_x,blade_y,blade_z)
hold on
plot3(path(:,1),path(:,2),path(:,3))


