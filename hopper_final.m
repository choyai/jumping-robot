ti = 0;
tf = 30;
g = 9.80665;

mr = 0.2;
ma = 1;
m = mr+ma;                   
ls = 0.2;
lr = 0.5;

qi = [ls+100.1;lr/2]; %[xp;xa]
vi = [0;4*pi*(lr/2)];
xi = [qi;vi];
x_d = 1;
t_v = [];
x_v = [];

iteration = 30;
for i = 1:iteration
     options = odeset('Events',@(t,x)event(t,x,controller(t,x),x_d),'Refine',4);
    [t,x,te,xe,ie] = ode45(@(t,x) dynamics(t,x,controller(t,x),x_d),[ti tf],xi,options);
    x_v = [x_v;x];
    t_v = [t_v;t];
    
    disp(i)
    
    if isempty(te)
        break
    end
    ti = te;
    
    disp([x_d ie])
    if x_d == 0
        if ie == 1
            xi = xe';
            x_d = 1;
        end
    elseif x_d == 1
        if ie == 1
            xi = xe';
            x_d = 0;
        elseif ie == 2
            F = (Ja*(M\(Ja')))\(Ja*xe(3:4)');
            v_a = xe(3:4)'-M\(Ja'*F)
            xi = [xe(1:2)';v_a];
            x_d = 3;
        elseif ie == 3
            F = (Ja*(M\(Ja')))\(Ja*xe(3:4)')
            v_a = xe(3:4)'-M\(Ja'*F)
            xi = [xe(1:2)';v_a];
            x_d = 2;
        end
    elseif x_d == 2
        xi = xe';
        x_d = 1;
    elseif x_d == 3
        xi = xe';
        x_d = 1;
    end          
end
hold on
plot(t_v,x_v(:,1),'b');
plot(t_v,x_v(:,2),'g');
hold off


function dx = dynamics(t,x,u,x_d)
    qd = x(3:4);
    mr = 0.2;
    ma = 1;
    m = mr+ma;
    g = 9.80665;
    lr = 0.5;
    
    M = [m ma;ma 1];
    G = [m*g;ma*g];
    S = [1 0;0 1];
    Js = [1 0];
    ls = 0.2;
    Ja = [0 1];
    if x_d == 0
        f = fs(t,x,ls);
        vd = M\(S*u-G+Js'*f);
    elseif x_d == 1
        vd = M\(S*u-G);
    elseif x_d == 2
        F = -(Ja*M*(Ja'))\(Ja*(M\u));
        vd = M\(S*u-G+Ja'*F);
    elseif x_d == 3
        F = -(Ja*M*(Ja'))\(Ja*(M\u));
        vd = M\(S*u-G+Ja'*F);
%     elseif x_d == 4
%         
%         Fc = -(J*M*(J'))\(J*(M\u));
%         vd = M\(S*u-G+Ja'*Fc);
%     elseif x_d == 5
    end
    dx = [qd;vd]; 
end
function f = fs(t,x,ls)
    k = 5800;
    b = 10;
    f = -k*(ls-x(1))-b*(x(3));
end
function [value,isterminal,direction] = event(t,x,u,x_d)
    mr = 0.2;
    ma = 1;
    m = mr+ma;
    g = 9.80665;
    lr = 0.5;
    
    M = [m ma;ma 1];
    G = [m*g;ma*g];
    S = [1 0;0 1];
    Js = [1 0];
    ls = 0.2;
    Ja = [0 1];
    
    if x_d == 0
        value = ls-x(1);
        isterminal = 1;
        direction = -1;
    elseif x_d == 1
        value = [x(1)-ls;x(2);lr-x(2)];
        isterminal = [1;1;1];
        direction = [-1;-1;-1];
    elseif x_d == 2
        F = -(Ja*M*(Ja'))\(Ja*(M\u));
        value = F;
        isterminal = 1;
        direction = 1;
    elseif x_d == 3
        F = -(Ja*M*(Ja'))\(Ja*(M\u));
        value = F;
        isterminal = 1;
        direction = -1;
    end
end


function u = controller(t,x)
      
    lr = 0.3;
    A = lr/2;
    q = A*sin(4*pi*t)+lr/2;
    qd = 4*pi*cos(2*pi*t);
    qdd = -16*pi^2*A*sin(2*pi*t);
    Kp = 100;
    Kd = 5;
    u = [0;qdd+Kp*(q-x(2))+Kd*(qd-x(4))];

end