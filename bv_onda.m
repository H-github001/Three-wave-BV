% Driver code for the Three Wave´s circuit with breeding.
% 
% This code integrates the three wave system
%
clear all;
close all;
clc;
% specify dimension
dim=3; 
% specify rk4 stepsize 
h2=0.001; %STEP=0.01;
h=h2;
tfinal=10.0;
% specify transient time
numtrans=1000;
% specify integration time
%numstep=(tfinal)/h2;
numstep=3750;
% specify initial conditions
x0 = [52.9122293488 -1.00963285610450 1.0];
% specify perturbation
dx0 = [.1 .1 .1];
xin=x0;
bst=8; %10;
A= 8.0;   %3.0;
par(1) = A;
eta= 28.128;
par(2) = eta;
omega=1.0;
par(3) = omega;
dim = 3;    
par = [A eta omega];
func='simulaonda';
for i=1:numtrans
%*********************************************************
% % remove transient time
%********************************************************
x = xin;   
f = feval(func, x, par,dim);  
c1 = h .* f'; 
x = xin + c1 /2;

f = feval(func, x, par,dim);   
c2 = h .* f';   
x = xin + c2 /2; 

f = feval(func, x, par,dim);   
c3 = h .* f';
x = xin + c3;

f = feval(func, x, par,dim);  
c4 = h .* f';   
xout = xin + (c1 + 2.*c2 + 2.*c3 + c4)./6;
xin=xout;
x=xout;
state1(i,:)=xout;
end

% remove transient time

Y = state1;

%  figure(2);
%  plot3(Y(:,1),Y(:,2),Y(:,3))
%  xlabel('x(t)');
%  ylabel('y(t)');
%  zlabel('z(t)');
% break;
% for i=1:numtrans
%     xout=stepit(func,x,par,h2,dim);
%     x=xout;
%     state(i,:)=xout;
% end


% redefine initial conditions
x0 = x;

% control run
xin=x0';
q=1;
for i=1:numstep
    for j=1:bst
        xout=stepit(func,xin,par,h2,dim);
        xin=xout;
        %x=xout;

        traj(q,:)=xin';
        q=q+1;
    end
    state(i,:)=xin';
end
% figure;
% plot3(state(:,1),state(:,2),state(:,3)) 
% xlabel('Wave A1(t)');
% ylabel('Wave A2(t)');
% zlabel('Wave A3(t)');
% title('"Three Wave solution"')
% figure;
% plot3(traj(1:2300,1),traj(1:2300,2),traj(1:2300,3),'c')
% hold on
% plot3(traj(2300:2650,1),traj(2300:2650,2),traj(2300:2650,3),'r')
% plot3(traj(2650:4950,1),traj(2650:4950,2),traj(2650:4950,3),'b-.')
% plot3(traj(4950:5300,1),traj(4950:5300,2),traj(4950:5300,3),'y')
% plot3(traj(5300:5900,1),traj(5300:5900,2),traj(5300:5900,3),'m--')
% plot3(traj(5900:6300,1),traj(5900:6300,2),traj(5900:6300,3),'g--')
% plot3(traj(6300:7300,1),traj(6300:7300,2),traj(6300:7300,3),'k-.')
% xlabel('Wave A1(t)');
% ylabel('Wave A2(t)');
% zlabel('Wave A3(t)');
% title('"Three Wave solution"')
% break;

%breeding
xin = x0 + dx0;         % add perturbation
d0 = sqrt(dx0*dx0');     % magnitude of perturbation
x=xin';
soma=0;
for i=1:numstep

    for j=1:bst
        xout=stepit(func,x,par,h2,dim);
        xin=xout;
        x = xin;
    end
    
    dx = x' - state(i,:);
    erro(i)= sum(0.5 .* (dx.^2)); 
    g = sqrt(dx*dx')/d0;
    lambdag(i,:)=log(g)/bst;
    dx = dx/g;
    xin = state(i,:) + dx;  % add the scaled part to the control at next t
    x=xin';
    d0 = sqrt(dx*dx');
end

%  FIGURES
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % figure(1);
% % plot3(state(:,1),state(:,2),state(:,3),'r--') 
% %   xlabel('x(t)');
% %   ylabel('y(t)');
% %   zlabel('z(t)');
% %  title('"Three Wave solution"')
%

% % Plot trajectory
% figure(2)
% plot3(traj(:,1),traj(:,2),traj(:,3));
% title('"Three Wave solution X Perturbation Trajectory"')
% %view([0,90])
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% 
% % Butterfly painted with growth rate
thresh=0.064;
thresh=.035;
mthresh=0.04;
mthresh=.017;
%state=state';

%circle(blue) < 0.002
%square(magenta) < 0.017
%diamond(green) <0.035
%star(red) >=0.035
% % figure(2);
% % comet3(traj(:,1),traj(:,2),traj(:,3))
% figure(3) 
% plot3(traj(:,1),traj(:,2),traj(:,3)); %,'k','MarkerSize',1);
% hold on;
% for i=1:numstep
%   %if sim == 1 
%     if lambdag(i)< 0.002
%         plot3(state(i,1),state(i,2),state(i,3),'ob','MarkerSize',3);
%         %plot3(state(i,1),state(i,2),state(i,3),'ok','MarkerSize',2);
%         %plot(state(i,1),state(i,2),'.');
%     end
%     if lambdag(i)>=0.002 & lambdag(i)< mthresh
%         plot3(state(i,1),state(i,2),state(i,3),'sm','MarkerSize',3);
%         %plot3(state(i,1),state(i,2),state(i,3),'sk','MarkerSize',2);
%         %plot(state(i,1),state(i,2),'g.');
%     end
%     if lambdag(i)>=mthresh & lambdag(i)< thresh
%         plot3(state(i,1),state(i,2),state(i,3),'dg','MarkerSize',3);
%         %plot3(state(i,1),state(i,2),state(i,3),'dk','MarkerSize',2);
%         %plot(state(i,1),state(i,2),'y.');
%     end
%     if lambdag(i)>=thresh
%         plot3(state(i,1),state(i,2),state(i,3),'*r','MarkerSize',3);
%         %plot3(state(i,1),state(i,2),state(i,3),'*k','MarkerSize',3)
%         %plot(state(i,1),state(i,2),'r.');
%     end
% %    sim = 2;
% %    else
% %         sim = 1;
% %    end    
%    if i < numstep
%         hold on;
%    end
% end
% %view([-20 20])
% xlabel('X Axis')
% ylabel('Y Axis')
% zlabel('Z Axis')
% title('Bred Vector Growth Rate');
% %drawnow;
% hold off;
s=size(lambdag,1);
dif1 = zeros(s,2);
kchg=0;
fred=0;
kred=0;
chg=0;
%sim=1;
% Plot BV Growth Rate over x-evolution
figure(4)
%subplot(2,1,1)
x=state(:,1);
xtraj=traj(:,1);
s=size(xtraj,1);
zero=zeros(s,1);
for i=2500*bst:numstep*bst
    plot(i,xtraj(i),'.k','MarkerSize',1);
    plot(i,zero(i),'-k');
    hold on
end
for i=2500:numstep
    if i>1
        dif1(i,1)=abs(state(i,2)-state(i-1,2));
        dif1(i,2)=lambdag(i);
        if abs(dif1(i,1)) < 0.0001
            %dif1(i,1)
            fred = 0;
            kchg=kchg+1;
        end
    end
     %if sim == 1
     if lambdag(i)< 0.002     
         plot(i*bst,x(i),'ob','MarkerSize',3);
         %plot(i*bst,x(i),'ok','MarkerSize',2)         
%       plot(i*bst,x(i),'marker','square','markersize',3,...
%                  'linestyle','-','color','k','linewidth',1);
       %'markeredgecolor','k','markerfacecolor',[.6 0 .6],...
     end     
     if lambdag(i)>=0.002 & lambdag(i)< mthresh  
        plot(i*bst,x(i),'sm','MarkerSize',3);
        %plot(i*bst,x(i),'sk','MarkerSize',2);
     end   
     if lambdag(i)>=mthresh & lambdag(i)< thresh
        plot(i*bst,x(i),'dg','MarkerSize',3);
        %plot(i*bst,x(i),'dk','MarkerSize',2);
     end 
     if lambdag(i)>=thresh
         if fred == 0 
            kred = kred +1;
            fred = 1; 
        end
        plot(i*bst,x(i),'*r','MarkerSize',3);
        %plot(i*bst,x(i),'*k','MarkerSize',3);
     end
%      sim = 2;
%     else
%         sim = 1;
%     end    
    if i < numstep
        hold on;
    end
end
ylabel('A1 Wave-var')
%xlabel('Time')
title('A1 wave vs Time, Painted with Growth');

kred
kchg

% subplot(2,1,2)
% for i=1:1000%numstep*bst
% % plot(i*bst,erro(i),'-ko','LineWidth',2,...
% %                 'MarkerEdgeColor','k',...
% %                 'MarkerFaceColor',[.49 1 .63],...
% %                 'MarkerSize',3)
% plot(i*bst,erro(i),'>k','MarkerSize',2)
% hold on
% end
% ylabel('RMSE ')
% xlabel('Time')
% title('RMSE of perturbation')

