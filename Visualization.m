view(135,20)    %Starting view
AL = 5;         %Define graph axis limits
grid on
L0=3;           %Rotatary arm length
L1=2;           %Pendulum length

Xh=[0 ; L0]';
Yh=[0 ; 0]';
Zh=[0 ; 0]';

Xv=[Xh(2) ; L0]';
Yv=[Yh(2) ; 0]';
Zv=[Zh(2) ; -L1]';

hold on
Harm  = fill3(Xh,Yh,Zh,'b'); 
Varm  = fill3(Xv,Yv,Zv,'g'); 
s=8;
M=scatter3(Xv(2),Yv(2),Zv(2),s,'filled','MarkerFaceColor','b','MarkerEdgeColor','k');
axis([-AL AL -AL AL -AL AL]);
Theta1=0;
Theta0=0;
c = [0 0 0];
TXT=title('Time: ');
for t=1:20:size(simTheta0,1)
    TXT2=sprintf('Time:%.2f',simt(t));
    set(TXT,'String',TXT2);
    
    Theta0=simTheta0(t);
    Theta1 =simTheta1(t);
    
    Xh(2)= L0*cos(Theta0);
    Yh(2)=L0*sin(Theta0);
    
    Xva = 0;                
    Yva = L1*sin(Theta1);    
    Zva = L1*cos(Theta1);   
    
    Xvb = Xva*cos(Theta0)-Yva*sin(Theta0)+L0*cos(Theta0);
    Yvb = Xva*sin(Theta0)+Yva*cos(Theta0) + L0*sin(Theta0);
    Zvb = Zva;
        
    Xv=[Xh(2);Xvb]';
    Yv=[Yh(2);Yvb]';
    Zv=[ 0   ;Zvb]';
    
    set(Harm,'XData',Xh);
    set(Harm,'YData',Yh);
    set(Harm,'ZData',Zh);
    %set(Harm,'FaceVertexCData',C);
    
    set(Varm,'XData',Xv);
    set(Varm,'YData',Yv);
    set(Varm,'ZData',Zv);
    %set(Varm,'FaceVertexCData',C);
    rem(t,30);

    %scatter3(Xv(2),Yv(2),Zv(2),s,'filled','MarkerFaceColor',c,'MarkerEdgeColor','k');
              
    set(M,'XData',Xv(2));
    set(M,'YData',Yv(2));
    set(M,'ZData',Zv(2));
    
    drawnow;
%     pause(0.0005); 
end

figure(2); hold on;
plot(simt, simTheta0)
title('Angle of the arm');
xlabel('Time [s]');
ylabel('Theta0 [rad]');

figure(3); hold on;
plot(simt, simdTheta0)
title('Angular velocity of the arm');
xlabel('Time [s]');
ylabel('dTheta0 [rad/s]');

figure(4); hold on;
plot(simt, simTheta1)
title('Angle of the pendulum');
xlabel('Time [s]');
ylabel('Theta1 [rad]');

figure(5); hold on;
plot(simt, simdTheta1)
title('Angular Velocity of the pendulum');
xlabel('Time [s]');
ylabel('dTheta1 [rad/s]');

figure(6); hold on;
plot(simt, simE)
title('Regulated Pendulum Energy: E');
xlabel('Time [s]');
ylabel('Energy');

figure(7); hold on;
plot(simt, simV)
title('Lyapunov Function: V');
xlabel('Time [s]');
ylabel('V');

figure(8); hold on;
plot(simt, simTau)
title('Control law');
xlabel('Time [s]');
ylabel('Tau');

figure(9); hold on;
plot(simTheta1, simdTheta1)
title('Pendulum Angle and Angular Velocity');
xlabel('Angle [rad]');
ylabel('Angular Velocity [rad/s]');

figure(10); hold on;
plot(simTheta0, simdTheta0)
title('Arm Angle and Angular Velocity');
xlabel('Angle [rad]');
ylabel('Angular Velocity [rad/s]');
