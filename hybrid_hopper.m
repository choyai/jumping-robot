ti = 0;
tf = 30;
g = 9.80665;

mr = 0.2;
ma = 1;
m = mr+ma;                   
ls = 0.2;
lr = 0.3;

qi = [ls;lr/2]; %[xp;xa]
vi = [0;2*pi*(lr/2)];
xi = [qi;vi];
x_d = 1;
t_v = [];
x_v = [];

iteration = 30;

for i = 1:iteration
    options = odeset('Events',@(t,x)event(t,x,controller(t,x,f(x,x_d,ls)),x_d,ls),'Refine',4);
    [t,x,te,xe,ie] = ode45(@(t,x) dynamics(t,x,controller(t,x,f(x,x_d,ls)),x_d,f(x,x_d,ls)),[ti tf],xi,options);
    disp(ie)
    t_v = [t_v;t];
    x_v = [x_v;x];
    ti = te;
    
    
    Ja = [0 1];
    M = [m ma;ma 1];
    
    if ie == 1
        xi = xe;
        x_d = ~x_d;
    else ie == 2
%         F = -(Ja*inv(M)*Ja')*inv(Ja*xe(3:4)); 
        F = (Ja*(M\(Ja')))\(Ja*xe(3:4)');
        v_a = xe(3:4)'-M\(Ja'*F);
%         v_a = [0;0];
        
        xi = [xe(1:2)';v_a]
    end

    
end
hold on
plot(t_v,x_v(:,1))
% plot(t_v,x_v(:,2))
hold off

function dx = dynamics(t,x,u,x_d,f)
    qd = x(3:4);
    mr = 0.2;
    ma = 1;
    m = mr+ma;
    g = 9.80665;
    lr = 0.3;
    
    M = [m ma;ma 1];
    G = [m*g;ma*g];
    S = [1 0;0 1];
    Js = [1 0];
    
    vd = M\(S*u-G+Js'*f);

    dx = [qd;vd];
end
function [value,isterminal,direction] = event(t,x,u,x_d,ls)
    lr = 0.3;
    value = [x(1)-ls;x(2);lr-x(2)];
    isterminal = [1;1;1];
    if x_d == 0 %ground
        direction_0 = 1;
    elseif x_d == 1 %ballistic
        direction_0 = -1;
    end
    direction = [direction_0;-1;-1];
end
function u = controller(t,x,f)
      
    lr = 0.3;
    A = lr/2;
    q = A*sin(2*pi*t)+lr/2;
    qd = 2*pi*cos(2*pi*t);
    qdd = -4*pi^2*A*sin(2*pi*t);
    Kp = 100;
    Kd = 5;
    u = [0;qdd+Kp*(q-x(2))+Kd*(qd-x(4))];

end
function fs = f(x,x_d,ls)
    k = 5800;
    b = 10;
    if x_d == 0 %ground
        fs = k*(ls-x(1))-b*x(3);
    elseif x_d== 1 %ballistic
        fs = 0;
    end
end


