% Компенсація ефекту інтегрального перерегулювання
clear all

% Перевірка середовища виконання
check_octave;


%Початкові умови
x0=[0;0;0;0];
% Моделювання ідеалізованого PID регулювання
[t,y]=ode45(@model_pid,[0:0.1:60],x0);

% Моделювання обмеженного PID регулювання
[t1,y1]=ode45(@model_bound,[0:0.1:60],x0);

% Моделювання обмеженного PID регулювання за схемою clamping
[t2,y2]=ode45(@model_clamping,[0:0.1:60],x0);

% Моделювання обмеженного PID регулювання за схемою back-calculation
[t3,y3]=ode45(@model_backcalculation,[0:0.1:60],x0);


figure(1)
subplot(2,1,1)
plot(t,y(:,3));
hold on;
plot(t1,y1(:,3));
plot(t2,y2(:,3));
plot(t3,y3(:,3));
grid on
title("Значення регульованого параметру");
xlabel("Час, с");
subplot(2,1,2)
plot(t, y(:,4));
hold on;
plot(t1, y1(:,4));
plot(t2, y2(:,4));
plot(t3, y3(:,4));
legend("Classic PID","Bound PID","Clamping PID","Back Calculation PID");
grid on;
title("Значення інтегральної складової");
xlabel("Час, с");
