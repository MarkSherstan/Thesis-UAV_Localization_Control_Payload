function [] = plotter(ii,t,Ej,Yj,Uj,Rj,U)
  figure(2)

  subplot(1,3,1);
  plot(t,Ej,'LineWidth',1.5);
  title('Error, Ej','FontSize',16);
  ylabel('Error Response (mA)','FontSize',16);
  ylim([-125 25]);
  xlim([0 17.5])

  subplot(1,3,2);
  plot(t,Uj,t,U,'-k','LineWidth',1.5);
  title({['Iteration: ', num2str(ii)],'Input, Uj'},'FontSize',16);
  xlabel('Time (s)','FontSize',16);
  ylabel('Input PWM','FontSize',16);
  ylim([-25 1200]);
  xlim([0 17.5])

  subplot(1,3,3);
  plot(t,Yj,t,Rj,'-k','LineWidth',1.5);
  title('Output, Yj','FontSize',16);
  ylabel('Output Response (mA)','FontSize',16);
  ylim([-25 400]);
  xlim([0 17.5])

  pause(0.1);
end
