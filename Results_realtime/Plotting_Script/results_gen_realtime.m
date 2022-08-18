close all
clc
clear all


%% plot parameters

save_dest='/home/hakim/catkin_ws/src/WTI_catkin/Results/Figures';
%turbine translation
x_shift= 80;
y_shift= 22;
z_shift= -68;

%x_shift=0;
%y_shift=0;
%z_shift=0;



%skip values
skip=4;


%Weights
W_h=80;
W_d=200;
W_r=60;












%colors defintion
%color_VTNMPC = [0 0.4470 0.7410];
%color_PAMPC = [0 0.5 0.1];
%color_NMPC = [0.85 0.3250 0.0980];
%color_point_traj = [0.4980 0.3250 0.85];
color_point_traj = [0 0 0];
color_VTNMPC = [0 0.1 0.9];

%color_NMPC = [0.9100 0.4100 0.1700];
color_NMPC =[0 0.9 0.1];
color_NMPC =[0.9 0.1 0];

color_VTNMPC_nw = [0.9100 0.4100 0.1700];
color_VTNMPC_nw = [0 0 1];

color_PAMPC = [0.1 0.9 0.1];
%color_NMPC_nw = [0.2 0.7 0.2];
color_NMPC_nw= [0.1 0.1 0.1];

color_turbine = [0.85 0.3250 0.0980];
color_covered_turbine = [ 1 0 0];
color_Ch= [ 1 0 0];
color_Cd= [ 0 1 0];
color_Cr= [ 0 0 1];
color_Ct= [ 0 0 0];
%% Read text files



R_pampc=dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/pampc.txt');

R_NMPC_nowind=dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/nmpc_f.txt');
%R_VT_nowind = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/vtnmpc_low_wind.txt');    %VTMPC data
R_VT_nowind = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/vtnmpc_f.txt');  
M_VT_nowind = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/vtnmpc_f.txt');    %VTMPC data

%M_VT_wind = dlmread('/home/hakim/catkin_ws/src/WTI_catkin/Results/Data/VTNMPC/test1.txt');
M_VT_wind = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/vtnmpc_f.txt');


M_NMPC_nowind=dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/nmpc_f.txt');
M_NMPC_wind=dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/nmpc_f.txt');


%M_NMPC_wind(1:17000,19)=M_NMPC_wind(1:17000,19)+180;

hk=M_NMPC_wind(1:end,19);


M_NMPC_wind(1:end,19)=M_NMPC_wind(1:end,19)+90;
NMPC_new=M_NMPC_wind;
%M_NMPC_wind(11200+(48*50):11200+(102*50),19)=M_NMPC_wind(11200+(48*50):11200+(102*50),19)-180;




M_PAMPC_nowind=dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/pampc.txt');
M_PAMPC_wind=dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/realtime/pampc.txt');
%M_PAMPC_wind=dlmread('/home/airlab/hakim_ws/src/WTI_catkin/Results/Data/5ms/VTMPC_old.txt');

ex_VT=M_VT_wind(100:end-4000,35);  
ex_PA=M_PAMPC_wind(100:end-12000,35);  


%% preprocess data
%M_VT_wind(:,7)=M_VT_wind(:,7)+120;
%M_NMPC_wind(:,7)=M_NMPC_wind(:,7)+120;

% Remove points before starting
%for i=1:length (M_VT_nowind)-2000
%    if (((M_VT_nowind(i,8)-M_VT_nowind(i+1,8)) ==0 )  && ((M_VT_nowind(i,9)-M_VT_nowind(i+1,9)) ==0 )  &&  ((M_VT_nowind(i,10)-M_VT_nowind(i+1,10)) ==0 ))
        
%       M_VT_nowind(i,:) = [];
        
        
%    end
%end
j=1;

t1_VT_nowind=100;
t2_VT_nowind=length(M_VT_nowind)-1000;

t1_VT_wind=100;
t2_VT_wind=length(M_VT_wind)-1000;

t1_NMPC_nowind=100;
t2_NMPC_nowind=length(M_NMPC_nowind)-1000;


t1_NMPC_wind=1000;
t2_NMPC_wind=length(M_NMPC_wind)-1000;


t1_PAMPC_nowind=100;
t2_PAMPC_nowind=length(M_PAMPC_wind)-1000;

t1_PAMPC_wind=100;
t2_PAMPC_wind=length(M_PAMPC_wind)-1000;







mx_1 = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/Mesh/MeshX.txt');
my_1 = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/Mesh/MeshY.txt');
mz_1 = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/Mesh/MeshZ.txt');


%mz(:,:)=mz(:,:)+120;

px_1 = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_x.txt');       %triangles mesh centres
py_1 = dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_y.txt');
pz_1= dlmread('/home/hakim/testing_ws/src/simulation/Results/Data/Point_to_View_Trajectory/mp_d1cm_interp_z.txt');

py=px_1+x_shift;
px=py_1+y_shift;
pz=pz_1+z_shift;

my_1=my_1+22;
mx_1=mx_1+80;
mz_1=mz_1-65;


mx=my_1/5;
my=mx_1/5;
my=my+2;
mz=mz_1/15;
mz=mz-0.25;




%{ 


t=M(:,1);                     %time
x=M(:,5:7);                   %Drone position (x,y,z)
x_pa=P(1:end,5:7);  

x(:,1)=x(:,1)+x_shift;
x(:,2)=x(:,2)+y_shift;
x(:,3)=x(:,3)+z_shift;

x_pa(:,1)=x_pa(:,1)+x_shift;
x_pa(:,2)=x_pa(:,2)+y_shift;
x_pa(:,3)=x_pa(:,3)+z_shift;



p=M(:,8:10);  %Reference point (px,py,pz)
p(:,1)=p(:,1)+x_shift;
p(:,2)=p(:,2)+y_shift;
p(:,3)=p(:,3)+z_shift;



n=M(:,11:13);   


x_gp=dlmread('plotting_data/Path_half.txt');

gp_x = interp1(1:length(x_gp(:,1)), x_gp(:,1), linspace(1, length(x_gp(:,1)), length(t)), 'nearest');
gp_y = interp1(1:length(x_gp(:,2)), x_gp(:,2), linspace(1, length(x_gp(:,2)), length(t)), 'nearest');
gp_z = interp1(1:length(x_gp(:,3)), x_gp(:,3), linspace(1, length(x_gp(:,3)), length(t)), 'nearest');



pa_x = interp1(1:length(x_pa(:,1)), x_pa(:,1), linspace(1, length(x_pa(:,1)), length(t)), 'nearest');
pa_y = interp1(1:length(x_pa(:,2)), x_pa(:,2), linspace(1, length(x_pa(:,2)), length(t)), 'nearest');
pa_z = interp1(1:length(x_pa(:,3)), x_pa(:,3), linspace(1, length(x_pa(:,3)), length(t)), 'nearest');


pa(:,1)=pa_x;
pa(:,2)=pa_y;
pa(:,3)=pa_z;



x_wp(:,1)=gp_x;
x_wp(:,2)=gp_y;
x_wp(:,3)=gp_z;
x_wp(:,1)=smooth(x_wp(:,1),150);
x_wp(:,2)=smooth(x_wp(:,1),150);
x_wp(:,3)=smooth(x_wp(:,1),150);



for i=2:length(n)-1
v_n(i)=(n(i,1)-n(i-1,1))^2+(n(i,2)-n(i-1,2))^0.5;
v_n(i)=real(v_n(i));
end


obj=M(:,26); %reference surface normal (nx,ny,nz)
kkt=M(:,27);                  %KKT error  % Total cost
vel_drone=M(:,14:16);         % drone velocity (vx,vy,vz)
angles_d=M(:,17:19);          % Angles (Roll, Pith, Yaw)
rates_d=M(:,28:30);           % Angular rates(Roll_dot, Pitch_dot, Yaw_dot)
x_ref=M(:,2:4);               % Way point trajectory %%for comparision%% (x_desired, y_desired, z_desired)
x_ref(:,1)=x_ref(:,1)+x_shift;
x_ref(:,2)=x_ref(:,2)+y_shift;
x_ref(:,3)=x_ref(:,3)+z_shift;
%x_gp(:,1)=x_gp(:,1)+x_shift;
%x_gp(:,2)=x_gp(:,2)+y_shift;
%x_gp(:,3)=x_gp(:,3)+z_shift;

a= p-x;                       % Desired heading vector a (ax,ay,az)
a_ref=p-x_ref;                % Desired heading vector a for reference trajectory (ax_d,ay_d,az_d)
yaw=M(:,19);                  % yaw angle calculation
yaw=yaw*pi/180;

%% Calculate the functions used in the Cost C_h, C_d, C_r
for i=1:length(a)
s_h(i,1)=(cos(yaw(i))*a(i,1)+sin(yaw(i))*a(i,2))/norm(a(i,1:2));    % heading function calculation s_h
s_d(i,1)=(a(i,1)^2+a(i,2)^2+a(i,3)^2)^0.5;                                   % distance function calculation s_d
s_d_wp(i,1)=norm(a_ref(i,1:2));                                     % distance function for wp trajectory s_d_wp
s_r(i,1)=a(i,1)*n(i,1)+a(i,2)*n(i,2);                               % region of inerest function
s_p(i,1)=a(i,1)*n(i,2)-a(i,2)*n(i,1);                               % perpendicularity measure
end



C_h= W_h*(s_h-1).^2;
C_d= W_d*(s_d-10).^2;
C_r=  W_r*(s_r+12.5).^2;

%}

%% Plot Trajectory 3-D 
fig_han = figure('name','Position 3D','units', 'normalized', 'outerposition', [0 1 1 1]);
% Create axes
axes1 = axes('Parent',fig_han,...
    'Position',[0.110393013100438 0.171280831210491 0.823908296943232 0.73182479175905]);

%axes1 = axes('Parent',fig_han,...
%    'Position',[0. 0.11128083121 0.12390 0.18182479175905]);

hold(axes1,'on');
set(gcf,'color','w');
%figure 
%set(gcf,'color','w');
%grid minor
%xlim([0 510])
%plot3(px,py,pz,'LineWidth', 3,'color',color_point_traj);
%hold on

%plot3(M_VT_nowind(1:end-5800,5),M_VT_nowind(1:end-5800,6),M_VT_nowind(1:end-5800,7),'--', 'LineWidth', 3,'color',color_VTNMPC);
%hold on



plot3(M_VT_wind(600:8000,5),M_VT_wind(600:8000,6),M_VT_wind(600:8000,7),'k', 'LineWidth', 3,'color',color_VTNMPC_nw);
hold on
scatter3(M_VT_wind(3000,5),M_VT_wind(3000,6),M_VT_wind(3000,6),'o')
hold on

%plot3(M_VT_wind_now(100:end-2000,5),M_VT_wind_now(100:end-2000,6),M_VT_wind_now(100:end-2000,7),'--','k', 'LineWidth', 3,'color',color_VTNMPC);

hold on

%plot3(M_NMPC_nowind(100:end-2000,5),M_NMPC_nowind(100:end-2000,6),M_NMPC_nowind(100:end-2000,7),'--', 'LineWidth', 3,'color',color_NMPC_nw);




%hold on

plot3(M_NMPC_wind(11200:18500,5),M_NMPC_wind(11200:18500,6),M_NMPC_wind(11200:18500,7),'k', 'LineWidth', 3,'color',color_NMPC);

hold on

plot3(M_PAMPC_wind(1700:end-3000,5),M_PAMPC_wind(1700:end-3000,6),M_PAMPC_wind(1700:end-3000,7),'k', 'LineWidth', 3,'color',color_PAMPC);


%pbaspect([1 1 1.2])

hold on
patch(mx',my',mz','w','EdgeColor','k','FaceAlpha',1);


hold on

X = [-2,0,0,-2];
Y = [1,1,1,1];
Z = [0.2,0.2,1.2,1.2];
h1=fill3(X,Y,Z,[1 0 0])
hold on
X = [-2,0,0,-2];
Y = [1,1,3,3];
Z = [1.2,1.2,1.2,1.2];
h2=fill3(X,Y,Z,[1 0 0])
X = [-2,-2,-2,-2];
Y = [1,3,3,1];
Z = [0.2,0.2,1.2,1.2];
h3=fill3(X,Y,Z,[1 0 0])

alpha(0.05)



hold on

X = [0,2,2,0];
Y = [1,1,1,1];
Z = [0.7,0.7,2,2];
h1=fill3(X,Y,Z,[0 1 0])
hold on
X = [0,2,2,0];
Y = [1,1,3,3];
Z = [2,2,2,2];
h2=fill3(X,Y,Z,[0 1 0])
X = [0,0,0,0];
Y = [1,3,3,1];
Z = [0.7,0.7,2,2];
h3=fill3(X,Y,Z,[0 1 0])

alpha(0.05)


X = [2,4,4,2];
Y = [1,1,1,1];
Z = [1,1,2.4,2.4];
h1=fill3(X,Y,Z,[0 0 1])
hold on
X = [2,4,4,2];
Y = [1,1,3,3];
Z = [2.4,2.4,2.4,2.4];
h2=fill3(X,Y,Z,[0 0 1])
X = [2,2,2,2];
Y = [1,3,3,1];
Z = [1.2,1.2,2.4,2.4];
h3=fill3(X,Y,Z,[0 0 1])

alpha(0.05)




%X = [-4,4,4,-4];
%Y = [1,1,1,1];
%Z = [0.2,0.2,1,1];
%h1=fill3(X,Y,Z,[0 0 1])
%hold on
%X = [-4,4,4,-4];
%Y = [1,1,3,3];
%Z = [1,1,1,1];
%h2=fill3(X,Y,Z,[0 0 1])
%X = [-4,-4,-4,-4];
%Y = [1,3,3,1];
%Z = [0.2,0.2,1,1];
%h3=fill3(X,Y,Z,[0 0 1])

%alpha(0.1)


%x = [-4 -2 0; -4 2 0; -4 2 2.5; -4 -2 2.5];
%x = [0 0 0; 0 0 0; 0 0 0;0 0 0];

%y = [-4 2 0; 4 2 0; -4 2 2.5; 4 2 2.5];

%z = [-4 -2 2.5; 4 -2 2.5; 4 2 2.5; -4 2 2.5];
%c = [2 2 2; 2 2 2; 2 2 2; 2 2 2];


%x = [-4 -2 0; -4 -2 2; -4 2 2.5; -4 2 0];
%y = [4 -2 2.5; -4 -2 2.5; -4 -2 0; 4 -2 0];
%z = [4 -2 2.5; 4 2 2.5; -4 2 2.5; -4 -2 2.5];
%c = [2 2 2; 2 2 2; 2 2 2; 2 2 3];
%fill3(x,y,z,c)








xlabel('{\it x}-axis (m)');
ylabel(['{\it y}-axis';'         (m)']);
zlabel('{\it z}-axis (m)');

%zlim([1.7 3.9]);




set(gcf,'color','w');
ax_han = gca;
set(ax_han,'FontSize',30)
%leg_han = legend('point trajectory ','VT-NMPC no wind','VT-NMPC wind','NMPC no wind','NMPC wind');
%leg_han = legend('VT-NMPC (no wind)','VT-NMPC (wind)','NMPC (no wind)','NMPC (wind)');
leg_han = legend('VT-NMPC ','NMPC');
leg_han = legend('VT-NMPC' , 'NMPC','PAMPC');

%leg_han = legend('point trajectory ','VT-NMPC','PAMPC','NMPC');
set(leg_han,'FontSize',30,'Location','northeast','Orientation','horizontal');
view(axes1,[-30.0836105158727 31.8655476563905]);
vec_pos = get(get(gca, 'XLabel'), 'Position');
set(get(gca, 'XLabel'), 'Position', vec_pos + [2 1.25 0.1]);
vec_pos = get(get(gca, 'YLabel'), 'Position');
set(get(gca, 'YLabel'), 'Position', vec_pos + [3.2 30 0]);
vec_pos = get(get(gca, 'ZLabel'), 'Position');
set(get(gca, 'ZLabel'), 'Position', vec_pos + [0.1 0 0]);
grid off
%saveas(gcf, [save_dest,'fig_pos_3D_sim_'], 'epsc');






%% Distance
d_VT_w=((M_VT_wind(:,5)-M_VT_wind(:,8)).^2+(M_VT_wind(:,6)-M_VT_wind(:,9)).^2+(M_VT_wind(:,7)-M_VT_wind(:,10)).^2).^0.5;



d_VT_now=((M_VT_nowind(:,5)-M_VT_nowind(:,8)).^2+(M_VT_nowind(:,6)-M_VT_nowind(:,9)).^2+(M_VT_nowind(:,7)-M_VT_nowind(:,10)).^2).^0.5;




d_NMPC_w=((M_NMPC_wind(:,5)-M_NMPC_wind(:,8)).^2+(M_NMPC_wind(:,6)-M_NMPC_wind(:,9)).^2+(M_NMPC_wind(:,7)-M_NMPC_wind(:,10)).^2).^0.5;



d_NMPC_now=((M_NMPC_nowind(:,5)-M_NMPC_nowind(:,8)).^2+(M_NMPC_nowind(:,6)-M_NMPC_nowind(:,9)).^2+(M_NMPC_nowind(:,7)-M_NMPC_nowind(:,10)).^2).^0.5;


d_PAMPC_w=((M_PAMPC_wind(:,5)-M_PAMPC_wind(:,8)).^2+(M_PAMPC_wind(:,6)-M_PAMPC_wind(:,9)).^2+(M_PAMPC_wind(:,7)-M_PAMPC_wind(:,10)).^2).^0.5;

figure

plot ([1:length(d_VT_w)-600]/50,d_VT_w(600:end-1),'LineWidth', 2,'color',color_VTNMPC_nw)

hold on

plot ([1:length(d_NMPC_w)-11200]/50,d_NMPC_w(11200:end-1),'LineWidth', 2,'color',color_NMPC)

hold on

plot ([1:7300]/50,d_PAMPC_w(1700:8999),'LineWidth', 2,'color',color_PAMPC)

hold on


xlim([0 150])
%ylim([5 8.5])
%legend ('VTNMPC' ,'NMPC')

leg_han = legend('VT-NMPC' , 'NMPC','PAMPC');



set(leg_han,'FontSize',10,'Location','northeast','Orientation','horizontal');
xlabel('Time [s]');
ylabel('Distance from blade [m]');
ax_han = gca;
set(ax_han,'FontSize',10)
grid on

%% Plot heading cost 

cc=M_NMPC_wind(:,19)-180;
for i =1:1:length (M_VT_nowind)
a1(i)=((M_VT_nowind(i,8) -M_VT_nowind(i,5))^2 + (M_VT_nowind(i,9) -M_VT_nowind(i,6))^2)^0.5;
%a2=((M_VT_wind(:,8) -M_VT_wind(:,5)).^2 + (M_VT_wind(:,9) -M_VT_wind(:,6)).^2).^0.5;
%a3=((M_NMPC_wind(:,8) -M_NMPC_wind(:,5)).^2 + (M_NMPC_wind(:,9) -M_NMPC_wind(:,6)).^2).^0.5;
s_i_vt_nw(i) =(cos(M_VT_nowind(i,19)*(pi/180)) .* ( M_VT_nowind(i,8) -M_VT_nowind(i,5)) + sin(M_VT_nowind(i,19)*(pi/180)).*(M_VT_nowind(i,9)-M_VT_nowind(i,6)))/a1(i);

end


for i =1:1:length (M_VT_wind)
a2(i)=((M_VT_wind(i,8) -M_VT_wind(i,5))^2 + (M_VT_wind(i,9) -M_VT_wind(i,6))^2)^0.5;
%a2=((M_VT_wind(:,8) -M_VT_wind(:,5)).^2 + (M_VT_wind(:,9) -M_VT_wind(:,6)).^2).^0.5;
%a3=((M_NMPC_wind(:,8) -M_NMPC_wind(:,5)).^2 + (M_NMPC_wind(:,9) -M_NMPC_wind(:,6)).^2).^0.5;
s_i_vt(i) =(cos(M_VT_wind(i,19)*(pi/180)) * ( M_VT_wind(i,8) -M_VT_wind(i,5)) + sin(M_VT_wind(i,19)*(pi/180))*(M_VT_wind(i,9)-M_VT_wind(i,6)))/a2(i);

end


for i =1:1:length (M_NMPC_wind)
a3(i)=((M_NMPC_wind(i,8) -M_NMPC_wind(i,5))^2 + (M_NMPC_wind(i,9) -M_NMPC_wind(i,6))^2)^0.5;
%a2=((M_VT_wind(:,8) -M_VT_wind(:,5)).^2 + (M_VT_wind(:,9) -M_VT_wind(:,6)).^2).^0.5;
%a3=((M_NMPC_wind(:,8) -M_NMPC_wind(:,5)).^2 + (M_NMPC_wind(:,9) -M_NMPC_wind(:,6)).^2).^0.5;
s_1(i) =(cos(M_NMPC_wind(i,19)*(pi/180)) * ( M_NMPC_wind(i,8) -M_NMPC_wind(i,5)) + sin(M_NMPC_wind(i,19)*(pi/180))*(M_NMPC_wind(i,9)-M_NMPC_wind(i,6)))/a3(i);



s_2(i)=(cos(cc(i)*(pi/180)) * ( M_NMPC_wind(i,8) -M_NMPC_wind(i,5)) + sin(cc(i)*(pi/180))*(M_NMPC_wind(i,9)-M_NMPC_wind(i,6)))/a3(i);


s_3(i)=(cos(hk(i)*(pi/180)) * ( M_NMPC_wind(i,8) -M_NMPC_wind(i,5)) + sin(hk(i)*(pi/180))*(M_NMPC_wind(i,9)-M_NMPC_wind(i,6)))/a3(i);

s_4(i)=(cos((hk(i)+180)*(pi/180)) * ( M_NMPC_wind(i,8) -M_NMPC_wind(i,5)) + sin((hk(i)+180)*(pi/180))*(M_NMPC_wind(i,9)-M_NMPC_wind(i,6)))/a3(i);





s_i_nmpc(i)=max([s_1(i) s_2(i) s_3(i) s_4(i)]);


if    s_i_nmpc(i)== s_2(i)
        NMPC_new(i,19)=cc(i);
else

    if   s_i_nmpc(i)== s_3(i)
        NMPC_new(i,19)=hk(i);
    else
        if s_i_nmpc(i)== s_4(i)
              NMPC_new(i,19)=hk(i)+180;
        else
        NMPC_new(i,19)=M_NMPC_wind(i,19);
        end
           

    end
end


end



for i =1:1:length (M_NMPC_nowind)
a3(i)=((M_NMPC_nowind(i,8) -M_NMPC_nowind(i,5))^2 + (M_NMPC_nowind(i,9) -M_NMPC_nowind(i,6))^2)^0.5;
%a2=((M_VT_wind(:,8) -M_VT_wind(:,5)).^2 + (M_VT_wind(:,9) -M_VT_wind(:,6)).^2).^0.5;
%a3=((M_NMPC_wind(:,8) -M_NMPC_wind(:,5)).^2 + (M_NMPC_wind(:,9) -M_NMPC_wind(:,6)).^2).^0.5;
s_i_nmpc_nw(i) =(cos(M_NMPC_nowind(i,19)*(pi/180)) * ( M_NMPC_nowind(i,8) -M_NMPC_nowind(i,5)) + sin(M_NMPC_nowind(i,19)*(pi/180))*(M_NMPC_nowind(i,9)-M_NMPC_nowind(i,6)))/a3(i);





s_2(i) =(cos(cc(i)*(pi/180)) * ( M_NMPC_nowind(i,8) -M_NMPC_nowind(i,5)) + sin(cc(i)*(pi/180))*(M_NMPC_nowind(i,9)-M_NMPC_nowind(i,6)))/a3(i);

if s_2(i)>s_i_nmpc_nw(i)

    s_i_nmpc_nw(i)=s_2(i);
    i

end
end

for i =1:1:length (M_PAMPC_wind)
a3(i)=((M_PAMPC_wind(i,8) -M_PAMPC_wind(i,5))^2 + (M_PAMPC_wind(i,9) -M_PAMPC_wind(i,6))^2)^0.5;
%a2=((M_VT_wind(:,8) -M_VT_wind(:,5)).^2 + (M_VT_wind(:,9) -M_VT_wind(:,6)).^2).^0.5;
%a3=((M_NMPC_wind(:,8) -M_NMPC_wind(:,5)).^2 + (M_NMPC_wind(:,9) -M_NMPC_wind(:,6)).^2).^0.5;
s_i_pampc(i) =(cos(M_PAMPC_wind(i,19)*(pi/180)) * ( M_PAMPC_wind(i,8) -M_PAMPC_wind(i,5)) + sin(M_PAMPC_wind(i,19)*(pi/180))*(M_PAMPC_wind(i,9)-M_PAMPC_wind(i,6)))/a3(i);

end



%plot ([1:length(d_VT_w)-600]/50,d_VT_w(600:end-1),'LineWidth', 2,'color',color_VTNMPC_nw)
hold on
%plot ([1:length(d_NMPC_w)-11200]/50,d_NMPC_w(11200:end-1),'LineWidth', 2,'color',color_NMPC)
hold on
%plot ([1:7300]/50,d_PAMPC_w(1700:8999),'LineWidth', 2,'color',color_PAMPC)
hold on


figure 

%plot ([1: length(s_i_vt_nw)-173]/50,s_i_vt_nw(173:end-1),'--','LineWidth', 3,'Color',color_VTNMPC)
%hold on 

if M_PAMPC_wind(i,5)> -2 && M_PAMPC_wind(i,5)< 0
plot ([1: length(s_i_vt)-600]/50,s_i_vt(600:end-1),'LineWidth', 2, 'Color',color_VTNMPC_nw, '*')
else
    plot ([1: length(s_i_vt)-600]/50,s_i_vt(600:end-1),'LineWidth', 2, 'Color',color_VTNMPC_nw)

end

%hold on
%plot ([1: length(s_i_nmpc_nw)-50]/50,s_i_nmpc_nw(50:end-1), '--','LineWidth', 3,'Color',color_NMPC_nw)
 



hold on
plot ([1: length(s_i_nmpc)-11200]/50,s_i_nmpc(11200:end-1),'LineWidth', 2,'Color',color_NMPC)

hold on

plot ([1:7300]/50,s_i_pampc(1700:8999),'LineWidth', 2,'Color',color_PAMPC)



leg_han = legend('VT-NMPC (no wind)','VT-NMPC (wind)','NMPC (no wind)','NMPC (wind)');
leg_han = legend('VT-NMPC','NMPC', 'PAMPC');
%leg_han = legend('VT-NMPC', 'PAMPC');
set(leg_han,'FontSize',10,'Location','northeast','Orientation','horizontal');
ylim([0.85 1])
xlabel('Time [s]');
ylabel('{Centering metric (CM)}');
ax_han = gca;
set(ax_han,'FontSize',10)
ax_han = gca;
set(ax_han,'FontSize',10)
xlim([0 150])
ylim([0 1])
grid on
%% Plot orthoganility cost

for i =1:1:length (M_VT_nowind)
 ax=M_VT_nowind(i,8) -M_VT_nowind(i,5) ;
 ay=  M_VT_nowind(i,9) -M_VT_nowind(i,6) ;
 az= M_VT_nowind(i,10) -M_VT_nowind(i,7) ;
 nx= M_VT_nowind(i,11);
 ny= M_VT_nowind(i,12);
 
 A1=ax-ax*nx*nx-ay*ny*nx;
  
 A2=ay-ax*nx*ny-ay*ny*ny;
 A3= az*az;
 o_vt_nw(i)=(A1^2+A2^2+A3^2)^0.5;
end


for i =1:1:length (M_VT_wind)
 ax=M_VT_wind(i,8) -M_VT_wind(i,5) ;
 ay=  M_VT_wind(i,9) -M_VT_wind(i,6) ;
 az= M_VT_wind(i,10) -M_VT_wind(i,7) ;
 nx= M_VT_wind(i,11);
 ny= M_VT_wind(i,12);
 
 A1=ax-ax*nx*nx-ay*ny*nx;
  
 A2=ay-ax*nx*ny-ay*ny*ny;
 A3= az*az;
 o_vt_w(i)=(A1^2+A2^2+A3^2)^0.5;

end


for i =1:1:length (M_NMPC_wind)
 ax=M_NMPC_wind(i,8) -M_NMPC_wind(i,5) ;
 ay=  M_NMPC_wind(i,9) -M_NMPC_wind(i,6) ;
 az= M_NMPC_wind(i,10) -M_NMPC_wind(i,7) ;
 nx= M_NMPC_wind(i,11);
 ny= M_NMPC_wind(i,12);
 
 A1=ax-ax*nx*nx-ay*ny*nx;
  
 A2=ay-ax*nx*ny-ay*ny*ny;
 A3= az*az;
 o_nmpc_w(i)=(A1^2+A2^2+A3^2)^0.5;
end


for i =1:1:length (M_NMPC_nowind)
 ax=M_NMPC_nowind(i,8) -M_NMPC_nowind(i,5) ;
 ay=  M_NMPC_nowind(i,9) -M_NMPC_nowind(i,6) ;
 az= M_NMPC_nowind(i,10) -M_NMPC_nowind(i,7) ;
 nx= M_NMPC_nowind(i,11);
 ny= M_NMPC_nowind(i,12);
 
 A1=ax-ax*nx*nx-ay*ny*nx;
  
 A2=ay-ax*nx*ny-ay*ny*ny;
 A3= az*az;
 o_nmpc_nw(i)=(A1^2+A2^2+A3^2)^0.5;
end


for i =1:1:length (M_PAMPC_wind)
 ax=M_PAMPC_wind(i,8) -M_PAMPC_wind(i,5) ;
 ay=  M_PAMPC_wind(i,9) -M_PAMPC_wind(i,6) ;
 az= M_PAMPC_wind(i,10) -M_PAMPC_wind(i,7) ;
 nx= M_PAMPC_wind(i,11);
 ny= M_PAMPC_wind(i,12);
 
 A1=ax-ax*nx*nx-ay*ny*nx;
  
 A2=ay-ax*nx*ny-ay*ny*ny;
 A3= az*az;
 o_pampc_w(i)=(A1^2+A2^2+A3^2)^0.5;
end



figure 

%plot ([1: length(s_i_vt_nw)-173]/50,s_i_vt_nw(173:end-1),'--','LineWidth', 3,'Color',color_VTNMPC)
%hold on 
%plot ([1: length(s_i_vt)-600]/50,s_i_vt(600:end-1),'LineWidth', 2, 'Color',color_VTNMPC_nw)

%hold on
%plot ([1: length(s_i_nmpc_nw)-50]/50,s_i_nmpc_nw(50:end-1), '--','LineWidth', 3,'Color',color_NMPC_nw)
 



hold on
%plot ([1: length(s_i_nmpc)-11200]/50,s_i_nmpc(11200:end-1),'LineWidth', 2,'Color',color_NMPC)

hold on

%plot ([1:7300]/50,s_i_pampc(1700:8999),'LineWidth', 2,'Color',color_PAMPC)


figure 
%plot ([1: length(o_vt_nw)-1200]/50,o_vt_nw(1200:end-1),'--' ,'LineWidth', 3,'Color',color_VTNMPC)

hold on

plot ([1: length( o_vt_w)-600]/50, (1-o_vt_w(600:end-1)/7),'LineWidth', 2,'Color',color_VTNMPC_nw,'o')

plot ([1: length( o_vt_w)-600]/50, (1-o_vt_w(600:end-1)/7),'LineWidth', 2,'Color',color_VTNMPC_nw)
%plot ([1: length( o_vt_w)-850]/50, (o_vt_w(850:end-1)),'LineWidth', 3,'Color',color_VTNMPC_nw)
hold on


%plot ([1: length(o_nmpc_nw)-1200]/50,o_nmpc_nw(1200:end-1),'--','LineWidth', 3,'Color',color_NMPC_nw)

hold on 
plot ([1: length(o_nmpc_w)-11200]/50,(1-o_nmpc_w(11200:end-1)/7),'LineWidth', 2,'Color',color_NMPC)
hold on
plot ([1:7500]/50,(1-o_pampc_w(1700:9199)/7),'LineWidth', 2,'Color',color_PAMPC)


%plot ([1: length(o_nmpc_w)-900]/50,(o_nmpc_w(900:end-1)),'LineWidth', 3,'Color',color_NMPC)

leg_han = legend('VT-NMPC (no wind)','VT-NMPC (wind)','NMPC (no wind)','NMPC (wind)');
leg_han = legend('VT-NMPC','NMPC ','PAMPC');

set(leg_han,'FontSize',10,'Location','northeast','Orientation','horizontal');

ax_han = gca;

xlabel('Time [s]');
ylabel('{Orthogonality distance metric (ODM)}');
xlim([0 145])
ylim([0.92 1])

ax_han = gca;
set(ax_han,'FontSize',10)
grid on




s1=sum(o_vt_nw(173:end-13000))/length(o_vt_nw(173:end-13000));


s1=sum(o_vt_w(1000:end-100))/length(o_vt_w(1000:end-50));
s1=sum(s_i_vt(1000:end-100))/length(s_i_vt(1000:end-50));

s1=sum(o_nmpc_nw(50:end-16000))/length(o_nmpc_nw(50:end-16000));
s1=sum(o_nmpc_w(1000:end-13000))/length(o_nmpc_w(1000:end-13000));
s1=sum(s_i_nmpc(1000:end-13000))/length(s_i_nmpc(1000:end-13000));
s1=sum(s_i_pampc(600:end-100))/length(s_i_nmpc(600:end-100));



%s1=sM

s1=sum(d_VT_w(1000:end-50))/length(d_VT_w(1000:end-50));

s1=sum(d_NMPC_now(50:end-16000))/length(d_NMPC_now(50:end-16000));
s1=sum(d_NMPC_w(1000:end-13000))/length(d_NMPC_w(1000:end-13000));
%s_i_nmpc


%close all
%figure 
%set(gcf,'color','w');
%plot3(M_NMPC_wind(1000,5),M_NMPC_wind(1000,6),M_NMPC_wind(1000,7),'w', 'LineWidth', 5.5,'color',color_NMPC);




%hold on
%patch(mx',my',mz','w','EdgeColor','k','FaceAlpha',1);



%{
%% Plot the costs

for i =1:1:length (M_VT_wind)
 ax=M_VT_wind(i,8) -M_VT_wind(i,5) ;
 ay=  M_VT_wind(i,9) -M_VT_wind(i,6) ;
 az= M_VT_wind(i,10) -M_VT_wind(i,7) ;
 nx= M_VT_wind(i,11);
 ny= M_VT_wind(i,12);
 

 s_ri(i)=ax*nx+ay*ny;
end



W_h=120;
W_d=70;
W_ri=30;
W_o=70;



C_h=(s_i_vt-1).^2*W_h;
C_d=(d_VT_w-7).^2*W_d;
C_ri=(s_ri+7).^2*W_ri;
C_o=(o_vt_w).^2*W_o;


C_w1=((M_VT_wind(:,28)).^2).^0.5*(pi/180);
C_w2=((M_VT_wind(:,29)).^2).^0.5*(pi/180);
C_w3=((M_VT_wind(:,30)).^2).^0.5*(pi/180);

%close all
figure
% plot
plot ([3000:length(C_h)-3800], C_h(3000:length(C_h)-3800),'LineWidth', 3,'Color','k')

hold on

plot ([3000:length(C_d)-3800], C_d(3000:length(C_d)-3800),'LineWidth', 3,'Color','r')
hold on

plot ([3000:length(C_ri)-3800], C_ri(3000:length(C_ri)-3800),'LineWidth', 3,'Color','b')

hold on
plot ([3000:length(C_o)-3800], C_o(3000:length(C_o)-3800),'LineWidth', 3,'Color','g')

legend ('C_h', 'C_d','C_ri', 'C_o')
figure 
plot ([3000:length(C_w1)-3800], C_w1(3000:length(C_w1)-3800),'LineWidth', 3,'Color','k')
hold on
plot ([3000:length(C_w2)-3800], C_w2(3000:length(C_w2)-3800),'LineWidth', 3,'Color','m')
hold on
%plot ([3000:length(C_w3)-3800], C_w3(3000:length(C_w3)-3800),'LineWidth', 3,'Color','c')

legend ('roll','pitch')

%}




for i =1:1:length (M_VT_wind)
 ax=M_VT_wind(i,8) -M_VT_wind(i,5) ;
 ay=  M_VT_wind(i,9) -M_VT_wind(i,6) ;
 
 nx= M_VT_wind(i,11);
 ny= M_VT_wind(i,12);
 

 s3_vt_nw(i)=-(ax*nx+ay*ny);
end
figure
plot(s3_vt_nw)
hold on
plot(d_VT_w)



%vx=M_VT_wind(500:33900,14);
%vy=M_VT_wind(500:33900,15);
%vz=M_VT_wind(500:33900,16);


%v_abs=vx.^2+vy.^2+vz.^2;
%v_abs=v_abs.^0.5;

%plot(v_abs)

figure 

plot3(R_VT_nowind(600:end,5),R_VT_nowind(600:end,6),R_VT_nowind(600:end,7),'k', 'LineWidth', 3,'color',color_VTNMPC_nw);

hold on
plot3(R_pampc(1700:end-3000,5),R_pampc(1700:end-3000,6),R_pampc(1700:end-3000,7),'k', 'LineWidth', 3,'color',color_PAMPC);
hold on
hold on

plot3(R_NMPC_nowind(11200:end,5),R_NMPC_nowind(11200:end,6),R_NMPC_nowind(11200:end,7),'k', 'LineWidth', 3,'color',color_NMPC);
legend('VTNMPC','PAMPC','NMPC')

xlabel ('x [m]')
ylabel ('y [m]')
zlabel ('z [m]')



 dlmwrite('nmpc_mat.txt',NMPC_new)



