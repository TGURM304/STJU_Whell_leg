function len_K_clc(len_left, len_right) 
    %调试用
    % len_left = 0.12;
    % len_right = 0.12;
    syms len_left len_right
    % 各类质量物理属性
    syms mw ml mb Iw Ill Ilr Ib Izz
    % 各类长度属性
    syms Rw Rl l_l l_r lwl lwr lbl lbr lc 
    %%我们默认质心位于腿的中点
    Ill = ml*len_left*len_left/12;
    Ilr = ml*len_right*len_right/12;
    lwl = len_left/2;
    lwr = len_right/2;
    lbl = lwl;
    lbr = lwr;
    l_l = len_left;
    l_r = len_right;
    % 定义时间变量
    syms t real
    syms g
    g = 9.8;
    %输出
    syms Tlwl Tlwr Tbll Tblr

    %中间变量
    syms fl fr Fwsl Fwsr Fbsl Fbsr Fwhl Fwhr Fbhl Fbhr

    % 各类会随着时间变化的值 - 定义为时间函数
    syms theta_wr(t) theta_wl(t) theta_lr(t) theta_ll(t) Sb(t) theta_b(t) phi(t)
    
    % =============================================
    % 关键修改：在计算导数之前就应用小角度近似
    % =============================================
    
    % 先定义原始的运动学表达式
    phi = Rw*(theta_wr(t) - theta_wl(t))/(2*Rl) + l_r*sin(theta_lr(t))/(2*Rl) - l_l*sin(theta_ll(t))/(2*Rl);
    Sb = Rw*(theta_wl(t)+theta_wr(t))/2 + (l_l*sin(theta_ll(t))+l_r*sin(theta_lr(t)))/2;
    hb = (l_r*cos(theta_lr(t))+l_l*cos(theta_ll(t)))/2;  
    Slr = Rw*theta_wr(t)+lwr*sin(theta_lr(t));
    Sll = Rw*theta_wl(t)+lwl*sin(theta_ll(t));
    hll = hb - lbr*cos(theta_lr(t));
    hlr = hb - lbl*cos(theta_ll(t));
    
    
    % =============================================
    % 现在计算基于近似后表达式的导数
    % =============================================
    
    dot_phi = diff(phi, t);
    ddot_phi = diff(phi,t,2);

    dot_Sb = diff(Sb,t);
    ddot_Sb = diff(Sb,t,2);

    dot_hb = diff(hb,t);
    ddot_hb = diff(hb,t,2);

    dot_Slr = diff(Slr,t);
    ddot_Slr = diff(Slr,t,2);
    dot_Sll = diff(Sll,t);
    ddot_Sll = diff(Sll,t,2);
    
    dot_hll = diff(hll,t);
    ddot_hll = diff(hll,t,2);
    dot_hlr = diff(hlr,t);
    ddot_hlr = diff(hlr,t,2);
        
    % 应用小角度近似到基础表达式
    % 定义替换规则
    old_expr = [sin(theta_ll(t)), cos(theta_ll(t)), ...
                sin(theta_lr(t)), cos(theta_lr(t)), ...
                sin(theta_b(t)), cos(theta_b(t))];
    
    new_expr = [theta_ll(t), 1, ...
                theta_lr(t), 1, ...
                theta_b(t), 1];
    
    % 应用小角度近似
    phi = subs(phi, old_expr, new_expr);
    Sb = subs(Sb, old_expr, new_expr);
    hb = subs(hb, old_expr, new_expr);
    Slr = subs(Slr, old_expr, new_expr);
    Sll = subs(Sll, old_expr, new_expr);
    hll = subs(hll, old_expr, new_expr);
    hlr = subs(hlr, old_expr, new_expr);

    dot_phi = subs(dot_phi, old_expr, new_expr);
    ddot_phi = subs(ddot_phi, old_expr, new_expr);

    dot_Sb = subs(dot_Sb, old_expr, new_expr);
    ddot_Sb = subs(ddot_Sb, old_expr, new_expr);

    dot_hb = subs(dot_hb, old_expr, new_expr);
    ddot_hb = subs(ddot_hb, old_expr, new_expr);

    dot_Slr = subs(dot_Slr, old_expr, new_expr);
    ddot_Slr = subs(ddot_Slr, old_expr, new_expr);
    dot_Sll = subs(dot_Sll, old_expr, new_expr);
    ddot_Sll = subs(ddot_Sll, old_expr, new_expr);
    
    dot_hll = subs(dot_hll, old_expr, new_expr);
    ddot_hll = subs(ddot_hll, old_expr, new_expr);
    dot_hlr = subs(dot_hlr, old_expr, new_expr);
    ddot_hlr = subs(ddot_hlr, old_expr, new_expr);
    %%%%%%%%%%%%%%%
    % 以上为运动学方程的表达式
    % 下面开始推导动力学方程的表达式
    %%%%%%%%%%%%%%%
    eq91 = mw*Rw*diff(theta_wl(t),t,2) == fl-Fwsl;
    eq92 = mw*Rw*diff(theta_wr(t),t,2) == fr-Fwsr;
    eq101 = Iw*diff(theta_wl(t),t,2) == Tlwl - fl*Rw;
    eq102 = Iw*diff(theta_wr(t),t,2) == Tlwr - fr*Rw;
    eq111 = ml*ddot_Sll == Fwsl - Fbsl;
    eq112 = ml*ddot_Slr == Fwsr - Fbsr;
    eq121 = ml*ddot_hll == Fwhl - Fbhl - ml*g;
    eq122 = ml*ddot_hlr == Fwhr - Fbhr - ml*g;
    eq131 = Ill*diff(theta_ll(t),t,2) == (Fwhl*lwl + Fbhl*lbl)*sin(theta_ll(t)) - (Fwsl*lbl + Fbsl*lbl)*cos(theta_ll(t)) - Tlwl + Tbll;
    eq132 = Ilr*diff(theta_lr(t),t,2) == (Fwhr*lwr + Fbhr*lbr)*sin(theta_lr(t)) - (Fwsl*lbr + Fbsl*lbr)*cos(theta_lr(t)) - Tlwr + Tblr;
    eq14 = mb*ddot_Sb == Fbsl+Fbsr;
    eq15 = mb*ddot_hb == Fbhl+Fbhr;
    eq16 = Ib*diff(theta_b(t),t,2) == -(Tbll + Tblr) - (Fbsl + Fbsr)*lc*cos(theta_b(t)) + (Fbhl + Fbhr)*lc*sin(theta_b(t));
    eq17 = Izz * diff(phi,t,2) == (-fl + fr)*Rl;
    eq18 = Fwhl == Fwhr;
    
    % =============================================
    % 对动力学方程也应用小角度近似
    % =============================================
    
    % 收集所有动力学方程
    dynamics_eqs = [eq91, eq92, eq101, eq102, eq111, eq112, eq121, eq122, ...
                   eq131, eq132, eq14, eq15, eq16, eq17, eq18];
    
    % 应用小角度近似
    approx_dynamics_eqs = subs(dynamics_eqs, old_expr, new_expr);
    
    % 将结果分配回原变量
    eq91 = approx_dynamics_eqs(1);
    eq92 = approx_dynamics_eqs(2);
    eq101 = approx_dynamics_eqs(3);
    eq102 = approx_dynamics_eqs(4);
    eq111 = approx_dynamics_eqs(5);
    eq112 = approx_dynamics_eqs(6);
    eq121 = approx_dynamics_eqs(7);
    eq122 = approx_dynamics_eqs(8);
    eq131 = approx_dynamics_eqs(9);
    eq132 = approx_dynamics_eqs(10);
    eq14 = approx_dynamics_eqs(11);
    eq15 = approx_dynamics_eqs(12);
    eq16 = approx_dynamics_eqs(13);
    eq17 = approx_dynamics_eqs(14);
    eq18 = approx_dynamics_eqs(15);
    
    % =============================================
    % 显示干净的结果
    % =============================================
    fprintf('小角度近似后的方程 (仅对 theta_ll, theta_lr, theta_b):\n\n');
    
    equation_names = {'eq91', 'eq92', 'eq101', 'eq102', 'eq111', 'eq112', ...
                     'eq121', 'eq122', 'eq131', 'eq132', 'eq14', 'eq15', ...
                     'eq16', 'eq17', 'eq18'};
    
    for i = 1:length(equation_names)
        fprintf('%s:\n', equation_names{i});
        fprintf('  %s\n\n', char(approx_dynamics_eqs(i)));
    end
    
    % 显示近似后的运动学表达式
    fprintf('近似后的运动学表达式:\n');
    fprintf('phi = %s\n', char(phi));
    fprintf('Sb = %s\n', char(Sb));
    fprintf('hb = %s\n', char(hb));
    fprintf('Slr = %s\n', char(Slr));
    fprintf('Sll = %s\n', char(Sll));
    fprintf('hll = %s\n', char(hll));
    fprintf('hlr = %s\n\n', char(hlr));
    
    fprintf('近似后的导数表达式:\n');
    fprintf('dot_phi = %s\n', char(dot_phi));
    fprintf('ddot_phi = %s\n', char(ddot_phi));
    fprintf('dot_Sb = %s\n', char(dot_Sb));
    fprintf('ddot_Sb = %s\n', char(ddot_Sb));

    disp("Finish - 小角度近似已正确应用")
    disp(ddot_Sll);
end