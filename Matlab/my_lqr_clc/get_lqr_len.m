tic
clear all
clc

% 定义机器人机体参数
syms R_w                % 驱动轮半径
syms R_l                % 驱动轮轮距/2
syms l_l l_r            % 左右腿长
syms l_wl l_wr          % 驱动轮质心到左右腿部质心距离
syms l_bl l_br          % 机体质心到左右腿部质心距离
syms l_c                % 机体质心到腿部关节中心点距离
syms m_w m_l m_b        % 驱动轮质量 腿部质量 机体质量
syms I_w                % 驱动轮转动惯量           (自然坐标系法向)
syms I_ll I_lr          % 驱动轮左右腿部转动惯量    (自然坐标系法向，实际上会变化)
syms I_b                % 机体转动惯量             (自然坐标系法向)
syms I_z                % 机器人z轴转动惯量        (简化为常量)

%定义中间变量
syms Sb dSb ddSb hb dhb ddhb slr dslr ddslr sll hlr dhlr ddhlr hll dhll ddhll 
%动力学中间变量
syms Fwsr Fwsl Fbsl Fbsr Fwhl Fwhr Fbhl Fbhr fl fr
% 定义其他独立变量并补充其导数
syms theta_wl   theta_wr   % 左右驱动轮转角
syms dtheta_wl  dtheta_wr 
syms ddtheta_wl ddtheta_wr ddtheta_ll ddtheta_lr ddtheta_b

% 定义状态向量
syms S ds dds phi dphi ddphi theta_ll dtheta_ll theta_lr dtheta_lr theta_b dtheta_b dsll ddsll 

% 定义控制向量
syms T_lwl T_lwr T_bll T_blr

% 输入物理参数：重力加速度
syms g

%方程运动学约束
S = R_w*(theta_wl+theta_wr)/2;
phi = R_w*(theta_wr - theta_wl)/(2*R_l) + l_r*sin(theta_lr)/(2*R_l) - l_l*sin(theta_ll)/(2*R_l);
Sb = R_w*(theta_wl + theta_wr)/2 + (l_l*sin(theta_ll) + l_r*sin(theta_lr))/2;
hb = (l_r*cos(theta_lr)+l_l*cos(theta_ll))/2;
Slr = R_w*theta_wr + l_wr*sin(theta_lr);
Sll = R_w*theta_wl + l_wl*sin(theta_ll);
hlr = hb - l_br*cos(theta_lr);
hll = hb - l_bl*cos(theta_ll);
%约束方程
dSb = R_w*dtheta_wl/2 + R_w*dtheta_wr/2 + l_l*cos(theta_ll)*dtheta_ll/2 + l_r*cos(theta_lr)*dtheta_lr/2;
ddSb = R_w*ddtheta_wl/2 + R_w*ddtheta_wr/2 + l_l*cos(theta_ll)*ddtheta_ll/2 + l_r*cos(theta_lr)*ddtheta_lr/2 ...
    -l_l*sin(theta_ll)*dtheta_ll*dtheta_ll/2 - l_r*sin(theta_lr)*dtheta_lr*dtheta_lr/2;

dphi = -R_w*dtheta_wl/(2*R_l) + R_w*dtheta_wr/(2*R_l) - l_l*cos(theta_ll)*dtheta_ll/(2*R_l) + l_r*cos(theta_lr)*dtheta_lr/(2*R_l);
ddphi = -R_w*ddtheta_wl/(2*R_l) + R_w*ddtheta_wr/(2*R_l) - l_l*cos(theta_ll)*ddtheta_ll/(2*R_l) + l_r*cos(theta_lr)*ddtheta_lr/(2*R_l) ...
+ l_l*sin(theta_ll)*dtheta_ll*dtheta_ll/(2*R_l) - l_r*sin(theta_lr)*dtheta_lr*dtheta_lr/(2*R_l);

dhb = -l_l*sin(theta_ll)*dtheta_ll/2 - l_r*sin(theta_lr)*dtheta_lr/2;
ddhb = -l_l*sin(theta_ll)*ddtheta_ll/2 - l_r*sin(theta_lr)*ddtheta_lr/2 ...
-l_l*cos(theta_ll)*dtheta_ll*dtheta_ll/2 - l_r*cos(theta_lr)*dtheta_lr*dtheta_lr/2;

dslr = R_w*dtheta_wr + l_wr*cos(theta_lr)*dtheta_lr;
ddslr = R_w*ddtheta_wr + l_wr*cos(theta_lr)*ddtheta_lr - l_wr*sin(theta_lr)*dtheta_lr*dtheta_lr;

dsll = R_w*dtheta_wl + l_wl*cos(theta_ll)*dtheta_ll;
ddsll = R_w*ddtheta_wl + l_wl*cos(theta_ll)*ddtheta_ll - l_wl*sin(theta_ll)*dtheta_ll*dtheta_ll;

dhll = dhb + l_bl*sin(theta_ll)*dtheta_ll;
ddhll = ddhb + l_bl*sin(theta_ll)*ddtheta_ll + l_bl*cos(theta_ll)*dtheta_ll*dtheta_ll;

dhlr = dhb + l_br*sin(theta_lr)*dtheta_lr;
ddhlr = ddhb + l_br*sin(theta_lr)*ddtheta_lr + l_br*cos(theta_lr)*dtheta_lr*dtheta_lr;

%中间变量约束
Fwhl = m_l*(ddhll + ddhlr) + (m_b*ddhb)/2 + m_l*g + m_b*g/2;
Fwhr = m_l*(ddhll + ddhlr) + (m_b*ddhb)/2 + m_l*g + m_b*g/2;
Fbhl = Fwhl - m_l*g - m_l*ddhll;
Fbhr = Fwhr - m_l*g - m_l*ddhlr;
fl = (T_lwl - I_w*ddtheta_wl)/R_w;
fr = (T_lwr - I_w*ddtheta_wr)/R_w;
Fwsl = fl - m_w*R_w*ddtheta_wl;
Fwsr = fr - m_w*R_w*ddtheta_wr;
Fbsl = Fwsl - m_l*ddsll;
Fbsr = Fwsr - m_l*ddslr;


%动力学方程十五则
eq10 = m_w*R_w*ddtheta_wl - fl - Fwsl;
eq11 = m_w*R_w*ddtheta_wr - fr - Fwsr;
eq12 = I_w*ddtheta_wl - T_lwl - fl*R_w;
eq13 = I_w*ddtheta_wr - T_lwr - fr*R_w;
eq1 = m_l*ddslr - (Fwsr - Fbsr);
eq2 = m_l*ddsll - (Fwsl - Fbsl);
eq3 = m_l*ddhll -(Fwhl - Fbhl - m_l*g);
eq4 = m_l*ddhlr - (Fwhr - Fbhr - m_l*g);
eq5 = I_ll*ddtheta_ll - ((Fwhl*l_wl + Fbhl*l_bl)*sin(theta_ll) - (Fwsl*l_bl + Fbsl*l_bl)*cos(theta_ll)-T_lwl+T_bll);
eq6 = I_lr*ddtheta_lr - ((Fwhr*l_wr + Fbhr*l_br)*sin(theta_lr) - (Fwsl*l_br + Fbsr*l_br)*cos(theta_lr)-T_lwr+T_blr);
eq7 = m_b*ddSb - (Fbsl + Fbsr);
eq8 = m_b*ddhb - (Fbhl + Fbhr);
eq9 = I_b*ddtheta_b - (-(T_bll + T_blr) - (Fbsl + Fbsr)*l_c*cos(theta_b) + (Fbhl + Fbhr)*l_c*sin(theta_b));
eq14 = I_z*ddphi - ((-fl + fr)*R_l); 
eq15 = Fwhl - Fwhr;
eq16 = S - R_w*(theta_wl + theta_wr)/2;

eqations = [eq10, eq11, eq12, eq13, eq5, eq6, eq9];
vars =  [theta_ll, theta_lr, theta_b];
%小角度近似
for v = vars
    eqations = subs(eqations, sin(v),v);
    eqations = subs(eqations, cos(v),1);
end
disp(eqations);

target = [ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b];
% %查矩阵的秩
% [Aeq, beq] = equationsToMatrix(eqations, target);
% disp(rank(Aeq));
disp("clc_start")
temp = solve([eq10, eq11, eq12, eq13, eq5, eq6, eq9],[ddtheta_wl, ddtheta_wr, ddtheta_ll, ddtheta_lr, ddtheta_b]);
disp("clc_end")