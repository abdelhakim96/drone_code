
clear all
clc
close all

%% Real tests
PA_1=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/pampc/run3.txt');
PA_2=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/pampc/run3.txt');
PA_3=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/pampc/run3.txt');

VT_1=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/vtnmpc/run4.txt');
VT_2=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/vtnmpc/run4.txt');
VT_3=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/vtnmpc/run4.txt');

N_1=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/nmpc/run6.txt');
N_2=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/nmpc/run6.txt');
N_3=dlmread('/home/hakim/Desktop/drone_code/dji_m100_trajectory/Exp_data/statistical/nmpc/run6.txt');


n=7400;
N_1=N_1(1:n,:);
N_2=N_2(1:n,:);
N_3=N_3(1:n,:);

PA_1=PA_1(1:n,:);
PA_2=PA_2(1:n,:);
PA_3=PA_3(1:n,:);


VT_1=VT_1(1:n,:);
VT_2=VT_2(1:n,:);
VT_3=VT_3(1:n,:);

N= (N_1+N_2+N_3)/3;
PA= (PA_1+PA_2+PA_3)/3;
VT= (VT_1+VT_2+VT_3)/3;


%d_PA=((PA(:,5)-PA(:,8)).^2+(PA(:,6)-PA(:,9)).^2+(PA(:,7)-PA(:,10)).^2).^0.5;
%d_N=((N(:,5)-N(:,8)).^2+(N(:,6)-N(:,9)).^2+(N(:,7)-N(:,10)).^2).^0.5;
%d_VT=((VT(:,5)-VT(:,8)).^2+(VT(:,6)-VT(:,9)).^2+(VT(:,7)-VT(:,10)).^2).^0.5;

d_PA=((PA(:,5)-PA(:,8)).^2+(PA(:,6)-PA(:,9)).^2).^0.5;
d_N=((N(:,5)-N(:,8)).^2+(N(:,6)-N(:,9)).^2).^0.5;
d_VT=((VT(:,5)-VT(:,8)).^2+(VT(:,6)-VT(:,9)).^2).^0.5;


figure 
hold on
plot(d_N,'LineWidth', 2,'color',[0 0 1])
hold on
plot(d_PA,'LineWidth', 2,'color',[1 0 0])
hold on
plot(d_VT,'LineWidth', 2,'color',[0 1 0])
hold on

yl = yline(0.5,'--','reference','LineWidth',2);
hold on

%yline(0.35,'LineWidth', 2,'color',[0 0 0])
hold on

%yline(0.65,'LineWidth', 2,'color',[0 0 0])
hold on

leg_han = legend( 'NMPC','PAMPC','VT-NMPC');
set(leg_han,'FontSize',10,'Location','northeast','Orientation','horizontal');
xlabel('Time [s]');
ylabel('Distance from blade [m]');
ax_han = gca;
set(ax_han,'FontSize',10)
grid on




for i =1 :n
   
    if d_PA(i)<0.6 && d_PA(i)> 0.4
        s_PA(i) = 1;
    else
        s_PA(i) =0;
    end
end


for i =1 :n
   
    if d_VT(i)<0.6 && d_VT(i)> 0.4
        s_VT(i) = 1;
    else
        s_VT(i) =0;
    end
end



for i =1 :n
   
    if d_N(i)<0.6 && d_N(i)> 0.4
        s_N(i) = 1;
    else
        s_N(i) =0;
    end
end

safety_VT=(sum(s_VT(:))/n)*100;
safety_PA=(sum(s_PA(:))/n)*100;
safety_N=(sum(s_N(:))/n)*100;
e_VT=(abs(mean(d_VT)-0.5)/0.5)*100;
e_PA=(abs(mean(d_PA)-0.5)/0.5)*100;
e_N=(abs(mean(d_N)-0.5)/0.5)*100;
 