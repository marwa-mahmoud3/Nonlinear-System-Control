clear;
clc;

% Initialize Parameter
T = 30; 
m = 80; 
p1 = 4; 
p2 = 2;  
q1 = 8; 
q2 = 10; 
c1 = 25; 
c2 = 15;
t0 = 0; 
h = T/m;

% Calculate alpha and beta
alpha = m*(m-1)/2;
beta = m*(m-1)*(2*m-1)/6;

% SXmbloic represetation
syms t

% Calculate v1 and v2
v1 = @(t) 1/(1+t);
v2 = @(t) 1/(1+3*t);

% Calculate x1_hat and x2_hat
x1_hat = @(t) 1/(40+cos(t));
x2_hat = @(t) 1/(20+cos(t));

% Calcullate dx1_ha and dx2_hat
dx1_hat =matlabFunction(diff(x1_hat(t)));
dx2_hat =matlabFunction(diff(x2_hat(t)));

% Calculate u1_hat and u2_hat 
u1_hat = @(t) (1/x2_hat(t))*(v1(t)*x1_hat(t) - dx2_hat(t));
u2_hat = @(t) (1/(1 - x2_hat(t) - x2_hat(t)))*(v2(t)*x1_hat(t) + dx1_hat(t) + dx2_hat(t));


% Calculate L1(t),L2(t),l1(t)and l2(t)
L1 = @(t) p1*power(h,2)*(alpha -beta*h*(v1(t)+v2(t)+u2_hat(t)))+2*h*m*((h*p1/4)+(c1/2))*(1- h*m*(v1(t)+v2(t)+u2_hat(t)));
L2 = @(t) beta*p1*power(h,3)*(u1_hat(t)-u2_hat(t))+2*power(h,2)*power(m,2)*((h*p1/4)+(c1/2))*(u1_hat(t)-u2_hat(t));

l1 = @(t) p1*power(h,2)*(alpha -beta*h*(v1(t)+v2(t)+u2_hat(t)))-beta*p2*power(h,3)*v1(t)+2*h*m*((h*p1/4)+(c1/2))*(1- h*m*(v1(t)+v2(t)+u2_hat(t)))-2*power(h,2)*power(m,2)*((h*p2/4)+(c2/2))*v1(t);
l2 = @(t) beta*p1*power(h,3)*(u1_hat(t)-u2_hat(t))+(alpha -beta*h*u1_hat(t))+(2*power(h,2)*power(m,2)*((h*p1/4)+(c1/2))*(u1_hat(t)-u2_hat(t)))-2*h*m*((h*p2/4)+(c2/2))*(1-h*m*u1_hat(t));

% Calculate a1(t),a2(t),a3(t),a4(t) and a5(t)
a1 =@(x) (h*q1/4)+(((p1/2)+(p2/2))*beta*power(h,3)+((h*p1/4)+(c1/2))*power(h,2)*power(m,2)+((h*p2/4)+(c2/2))*power(h,2)*power(m,2))*power(x(2),2);
a2 =@(x) (h*q2/4)+(beta*p1*power(h,3)/2)+((h*p1/4)+(c1/2))*2*power(h,2)*power(m,2)*power((1-x(1)-x(2)),2);
a3 =@(x) (beta*p1*power(h,3)+((h*p1/4)+(c1/2))*2*power(h,2)*power(m,2))*(1-x(1)-x(2))*x(2);

a4 =@(t,x) l1(t)*x(2)*(x(1)-x1_hat(t)) + l2(t)*x(2)*(x(2)-x2_hat(t));
a5 =@(t,x) L1(t)*(1-x(1)-x(2))*(x(1)-x1_hat(t)) + L2(t)*(1-x(1)-x(2))*(x(2)-x2_hat(t));

% Nonlinear  Differential Equations
F =@(t,x) [-(v1(t)+v2(t))*x(1) + (u1_hat(t) + (2*a2(x)*a4(t,x)-a3(x)*a5(t,x))/(power(a3(x),2)-4*a1(x)*a2(x)))*x(2) + (u2_hat(t) + (2*a1(x)*a5(t,x)-a3(x)*a4(t,x))/(power(a3(x),2)-4*a1(x)*a2(x)))*(1-x(1)-x(2)) 
    -(u1_hat(t)+(2*a2(x)*a4(t,x)-a3(x)*a5(t,x))/(power(a3(x),2)-4*a1(x)*a2(x)))*x(2) + v1(t)*x(1)];


[t, x] = ode45(F, [t0 T], [0 0]);
x1=x(:,1);
x2=x(:,2);

%% Plot x1(t) vs t
figure
plot(t, x1)
hold on
for i = 1:numel(t)
    x1hat(i) = x1_hat(t(i));
end
plot(t,x1hat')
grid on
xlabel('Time $t$','interpreter','latex')
ylabel('$x1(t)$ and $\hat{x1}(t)$','interpreter','latex')
legend('$x1(t)$','$\hat{x1}(t)$','interpreter','latex')

%% Plot x2(t) vs t
figure
plot(t, x2)
hold on
for i = 1:numel(t)
    x2hat(i) = x2_hat(t(i));
end
plot(t,x2hat)
grid on
xlabel('Time $t$','interpreter','latex')
ylabel('$x2(t)$ and $\hat{x2}(t)$','interpreter','latex')
legend('$x2(t)$','$\hat{x2}(t)$','interpreter','latex')


% Optimal control variables
for i = 1:numel(t)
    u1(i) = u1_hat(t(i)) + (2*a2(x)*a4(t(i),x)-a3(x)*a5(t(i),x))/(power(a3(x),2)-4*a1(x)*a2(x));
    u2(i) = u2_hat(t(i)) + (2*a1(x)*a5(t(i),x)-a3(x)*a4(t(i),x))/(power(a3(x),2)-4*a1(x)*a2(x));
end

%% Plot u1(t) vs t
figure
plot(t, u1)
hold on
for i = 1:numel(t)
    u1hat(i) = u1_hat(t(i));
end
plot(t, u1hat)
grid on
xlabel('Time $t$','interpreter','latex')
ylabel('$u1(t)$ and $\hat{u1}(t)$','interpreter','latex')
legend('$u1(t)$','$\hat{u1}(t)$','interpreter','latex')


%% Plot u2(t) vs t
figure
plot(t, u2)
hold on
for i = 1:numel(t)
    u2hat(i) = u2_hat(t(i));
end
plot(t, u2hat)
grid on
xlabel('Time $t$','interpreter','latex')
ylabel('$u2(t)$ and $\hat{u2}(t)$','interpreter','latex')
legend('$u2(t)$','$\hat{u2}(t)$','interpreter','latex')