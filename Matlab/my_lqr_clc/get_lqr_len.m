tic
clear all
clc

% 定义机器人机体参数
syms R_w                % 驱动轮半径
syms R_l                % 驱动轮轮距/2
syms l_l l_r            % 左右腿长
syms l_wl l_wr          % 驱动轮质心到左右腿部质心距离
syms l_bl l_br          % 髋关节中点到左右腿部质心距离
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

%-------------------------------------------------------------------------------------------------%
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

%进行小角度近似后
ddphi = R_w*(-ddtheta_wl+ddtheta_wr)/(2*R_l) - l_l*theta_ll*ddtheta_ll/(2*R_l) + l_r*ddtheta_lr/(2*R_l);
ddhll = ddhb + l_bl*theta_ll*ddtheta_ll;
ddhlr = ddhb + l_br*theta_lr*ddtheta_lr;
ddSb = R_w*(ddtheta_wl+ddtheta_wr)/2 + l_l*ddtheta_ll/2 + l_r*ddtheta_lr/2;
ddhb = -l_l*theta_ll*ddtheta_ll/2 - l_r*theta_lr*ddtheta_lr/2;
ddslr = R_w*ddtheta_wr + l_wr*ddtheta_lr;
ddsll = R_w*ddtheta_wl + l_wl*ddtheta_ll;
%计算后的动力学的中间变量
%%按照从后往前的顺序替换可以保证所有内容都被替换为各个二阶导的基本方程
fl = (T_lwl - I_w*ddtheta_wl)/R_w;
fr = (T_lwr - I_w*ddtheta_wr)/R_w;
Fwsl = fl - m_w*R_w*ddtheta_wl;
Fwsr = fr - m_w*R_w*ddtheta_wr;
Fbsl = Fwsl - m_l*ddsll;
Fbsr = Fwsr - m_l*ddslr;
Fbhl = (m_b*ddhb + m_b*g - (m_l*ddhlr - m_l*ddhll))/2;
Fbhr = (m_b*ddhb + m_b*g + (m_l*ddhlr - m_l*ddhll))/2;
Fwhl = m_l*ddhll + m_l*g + Fbhl;
Fwhr = m_l*ddhlr + m_l*g + Fbhr;

%动力学方程十五则
eq1 = m_w*R_w*ddtheta_wl - (fl - Fwsl) == 0;
eq2 = m_w*R_w*ddtheta_wr - (fr - Fwsr) == 0;
eq3 = I_w*ddtheta_wl - (T_lwl - fl*R_w) == 0;
eq4 = I_w*ddtheta_wr - (T_lwr - fr*R_w) == 0;
eq5 = m_l*ddslr - (Fwsr - Fbsr) == 0;
eq6 = m_l*ddsll - (Fwsl - Fbsl) == 0;
eq7 = m_l*ddhll - (Fwhl - Fbhl - m_l*g) == 0;
eq8 = m_l*ddhlr - (Fwhr - Fbhr - m_l*g) == 0;
eq9 = I_ll*ddtheta_ll - ((Fwhl*l_wl + Fbhl*l_bl)*sin(theta_ll) - (Fwsl*l_wl + Fbsl*l_bl)*cos(theta_ll)-T_lwl+T_bll) == 0;
eq10 = I_lr*ddtheta_lr - ((Fwhr*l_wr + Fbhr*l_br)*sin(theta_lr) - (Fwsr*l_wr + Fbsr*l_br)*cos(theta_lr)-T_lwr+T_blr) == 0;
eq11 = m_b*ddSb - (Fbsl + Fbsr) == 0;
eq12 = m_b*ddhb - (Fbhl+Fbhr-m_b*g) == 0;
eq13 = I_b*ddtheta_b - (-(T_bll+T_blr)-(Fbsl+Fbsr)*l_c*cos(theta_b)+(Fbhl+Fbhr)*l_c*sin(theta_b)) == 0;
eq14 = I_z*ddphi - (-fl+fr)*R_l == 0;
eq15 = Fwhl-Fwhr == 0;

%%建立替换规则的元胞数组，通过管理rules来实现管理替换变量
rules = {
    Fwhr,m_l*ddhlr + m_l*g + Fbhr;
    Fwhl , m_l*ddhll + m_l*g + Fbhl;
    Fbhr , (m_b*ddhb + m_b*g + (m_l*ddhlr - m_l*ddhll))/2;
    Fbhl , (m_b*ddhb + m_b*g - (m_l*ddhlr - m_l*ddhll))/2;
    Fbsr , Fwsr - m_l*ddslr;
    Fbsl , Fwsl - m_l*ddsll;
    Fwsr , fr - m_w*R_w*ddtheta_wr;
    Fwsl , fl - m_w*R_w*ddtheta_wl;
    fr , (T_lwr - I_w*ddtheta_wr)/R_w;
    fl , (T_lwl - I_w*ddtheta_wl)/R_w;
    ddphi , R_w*(-ddtheta_wl+ddtheta_wr)/(2*R_l) - l_l*theta_ll*ddtheta_ll/(2*R_l) + l_r*ddtheta_lr/(2*R_l);
    ddhll , ddhb + l_bl*theta_ll*ddtheta_ll;
    ddhlr , ddhb + l_br*theta_lr*ddtheta_lr;
    ddSb , R_w*(ddtheta_wl+ddtheta_wr)/2 + l_l*ddtheta_ll/2 + l_r*ddtheta_lr/2;
    ddhb , -l_l*theta_ll*ddtheta_ll/2 - l_r*theta_lr*ddtheta_lr/2;
    ddslr , R_w*ddtheta_wr + l_wr*ddtheta_lr;
    ddsll , R_w*ddtheta_wl + l_wl*ddtheta_ll;
    sin(theta_wr),theta_wr;
    sin(theta_wl),theta_wl;
    sin(theta_ll),theta_ll;
    sin(theta_lr),theta_lr;
    sin(theta_b),theta_b;
    cos(theta_wr),1;
    cos(theta_wl),1;
    cos(theta_ll),1;
    cos(theta_lr),1;
    cos(theta_b),1;
    dtheta_wr, 0;
    dtheta_wl, 0;
    dtheta_ll, 0;
    dtheta_lr, 0;
    dtheta_b, 0;
};
change = [eq14,eq9,eq10,eq11,eq13];
var = [ddtheta_wl,ddtheta_wr,ddtheta_ll,ddtheta_lr,ddtheta_b];
for i = 1:size(rules,1)
    change = subs(change,rules(i,1),rules(i,2));
end

answer = solve(change,var)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%此处输入机器人的实际参数
sym_2_real = {
    R_w, 0.06; %轮半径
    R_l, 0.15; %机体半径
    l_wl, l_l/2; %驱动轮质心到左右腿质心距离
    l_wr, l_r/2; %默认为腿长一半
    l_bl, l_l/2; %髋关节中点到腿质心距离
    l_br, l_r/2; %默认为腿长一半
    l_c , 0.005; %身体质心到髋关节中点连线距离
    m_w , 0.8; %轮的质量
    m_l , 0.168; %腿的质量
    m_b , 4.56; %机体质量
    I_w , 566.352*1.3/1000/1000; %轮的转动惯量
    I_ll, (l_l/2)^2*m_l; %腿的转动惯量，轴心为髋关节中点
    I_lr, (l_r/2)^2*m_l;
    I_b , l_c^2*m_b;
    I_z , 0.03;
    };
%%变量：R_l
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
toc