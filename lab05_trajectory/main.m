% Дослідження траекторії руху ЛА

% Очистка змінних середовища
clear all
ode_opts = odeset('MaxStep',1e0);
tflight=0:1:800;

% Проміжні точки маршруту
flight_plan_x=[0 250 750 1250 1500];
flight_plan_y=[0 0   500 0    0];

% Стартова позиція
start_pos=[flight_plan_x(1),flight_plan_y(1),0];

% Формування польотного плану
flight_plan=zeros(1,4);
for i=1:length(flight_plan_x)-1
  flight_plan(i,:)=[flight_plan_x(i),...
                    flight_plan_y(i),...
                    flight_plan_x(i+1),...
                    flight_plan_y(i+1)];
end

% Модель вітрових збурень
% (1) - складова збурення вздовж осі x
% (2) - складова збурення вздовж осі y
% (3) - моментра складова збурення = 0
disturbance = @(t) [0,0,0]';

% Польот на ППМ
clear model_direct;
sim_model_direct=@(t,x) model_direct(t,x,flight_plan,disturbance);
[~,y_direct]=ode45(sim_model_direct,tflight,start_pos,ode_opts);

% Польот за ЛЗШ
clear model_angle;
sim_model_angle=@(t,x) model_angle(t,x,flight_plan,disturbance);
[~,y_angle]=ode45(sim_model_angle,tflight,start_pos,ode_opts);

% Виконати коррекцію польотного плану:
% - розрахувати мінімальний радіус повороту
% - розрахувати упередження виконання маневру
% - провести симуляцію за скорегованим планом польоту
% - порівняти результати симуляції

% Корекція польотного плану
flight_plan_corr = flight_plan;

% Мінімальний радіус повороту
% функція від параметрів машини дубінса u_bound та V
% УВАГА - враховуйте одиниці кутів - радіани або градуси
Rmin = 0;

% Упередження виконання маневру
for i=1:size(flight_plan,1)-1
  % Розрахунок азимуту ЛЗШ
  a1=atan2(flight_plan(i,4)-flight_plan(i,2),flight_plan(i,3)-flight_plan(i,1));
  a2=atan2(flight_plan(i+1,4)-flight_plan(i+1,2),flight_plan(i+1,3)-flight_plan(i+1,1));
  angle_turn=a2-a1; % Кут розвороту
  % Розрахунок упередження виконання манервру
  % функція від Rmin та angle_turn
  L=abs(Rmin);
  %
  disp([i, rad2deg(angle_turn), L]);
  flight_plan_corr(i,3)=flight_plan(i,3)-L*cos(a1);
  flight_plan_corr(i,4)=flight_plan(i,4)-L*sin(a1);
end
% Моделювання за скорегованим планом польоту

% Польот за ЛЗШ
clear model_angle;
sim_model_angle_corr=@(t,x) model_angle(t,x,flight_plan_corr,disturbance);
[t,y_angle_corr]=ode45(sim_model_angle_corr,tflight,start_pos,ode_opts);

% Візуалізація виконання польотного плану
figure(1)
plot(y_direct(:,1),y_direct(:,2),'r-','LineWidth',2); 
hold on
plot(y_angle(:,1),y_angle(:,2),'g-','LineWidth',2);
plot(flight_plan(:,[1,3])',flight_plan(:,[2,4])','b*--','LineWidth',1);
hold off
axis equal
grid on
legend('Польот на ППМ','Польот за ЛЗШ','План польоту');
title('Порівняння траекторій польоту за ППМ та ЛЗШ');

figure(2)
plot(y_direct(:,1),y_direct(:,2),'r-','LineWidth',2); hold on;
plot(y_angle_corr(:,1),y_angle_corr(:,2),'g-','LineWidth',2); hold on;
plot(flight_plan_corr(:,[1,3])',flight_plan_corr(:,[2,4])','b*--','LineWidth',1); 
hold off
axis equal
grid on
legend('Польот на ППМ','Польот на ЛЗШ (скоригований)','План польоту');
title('Порівняння траекторій польоту за ППМ та ЛЗШ');

figure(3)
plot(y_angle(:,1),y_angle(:,2),'r-','LineWidth',2); hold on;
plot(y_angle_corr(:,1),y_angle_corr(:,2),'g-','LineWidth',2); hold on;
plot(flight_plan_corr(:,[1,3])',flight_plan_corr(:,[2,4])','b*--','LineWidth',1);
hold off
axis equal
grid on
legend('Польот за ЛЗШ','Польот за ЛЗШ (скоригований)','План польоту');
title('Порівняння траекторій польоту за ЛЗШ');







