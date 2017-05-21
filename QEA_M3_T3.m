%% Attempt at moving robot in Bridge of Death Motion

clear;
hold off;
clf;
syms a l t;
assume(t, 'real');
assume(a, 'positive'); assume(l, 'positive');

rx = -2*a*((l-cos(t))*cos(t) + (1-l))
ry = 2*a*(l-cos(t))*sin(t);
r = [rx; ry; 0]
R = simplify(norm(r))
dr = diff(r, t)

%c)
Th = simplify(dr/norm(dr))
dTh = diff(Th, t) %Unnormalized N
Nh = simplify(dTh/norm(dTh))
w = cross(Th, dTh)
v = R * w

a = .4
l = .4
m=2*pi
reso = .02
t = [0:reso:m];
rx_p = matlabFunction(vpa(subs(rx)))
ry_p = matlabFunction(vpa(subs(ry)))
T_p = vpa(subs(Th))
N_p = vpa(subs(Nh))
w_p = matlabFunction(vpa(subs(w)))
v_p = matlabFunction(vpa(subs(v)))
R_p = vpa(subs(R))
v_p = v_p()
w_p = w_p()
hold on;
%zs = zeros(size(rx_p()));
quiver(rx_p(), ry_p(), T_p(1,:), T_p(2,:), 'b-')
quiver(rx_p(), ry_p(), N_p(1,:), N_p(2,:), 'r-')
limiter = .33723;
d = .24;
linvel = v_p(3,:) * limiter
angvel = w_p(3,:) * limiter
Vl = linvel - (angvel*d)/2
Vr = linvel + (angvel*d)/2

%%%%%%%%%%%%%%%%%%%%%
pub = rospublisher('/raw_vel');

strtmsg = rosmessage(pub);
stopmsg = rosmessage(pub);

% get the robot moving
stopmsg.Data = [0, 0];

tic;
send(pub, strtmsg);


while 1
    strtmsg.Data = [Vl(round(toc/t)), Vr(round(toc/t))];
    if toc > m
        send(pub, stopmsg)
        break
    end
end