function drawRobot_with_tension(X,p)
    % This function draws the robot with configuration q and parameters p
    % INPUTS:
    %   x = [16, 1] = column vector of a single robot configuration
    %   p = parameter struct

    % Compute the points that will be used for plotting
    P = getPoints_with_tension(X,p.l); %forward kinematics
        
    x = P(:,1);
    y = P(:,2);
    z = P(:,3);
    
    % Axis:
    bnds = 0.5;
    xBnd = [-bnds,bnds];
    yBnd = [-bnds,bnds];
    zBnd = [0,1];

    % Colors:
    colorString = [150 150 150]/255; %gray
    colorQuad = [200,60,60]/255; %red
    colorBall = [60,60,200]/255; %blue
    colorPoints = 'black';

    % Set up the figure
    hold off;

    % Plot the ground:
    [xGround, yGround] = meshgrid(-bnds:0.1:bnds); % Generate x and y data
    zGround = zeros(size(xGround, 1)); % Generate z data
    surf(xGround, yGround, zGround) % Plot the surface    
    colormap([cool(64);gray(64)]);
    %plot(xBnd,[0,0],'LineWidth',1,'Color',colorGround);

    hold on;

    % Plot the links:
    plot3(x([1,2]),y([1,2]),z([1,2]),'LineWidth',1,'Color',colorString);
    plot3(x([1,3]),y([1,3]),z([1,3]),'LineWidth',3,'Color',colorQuad);
    plot3(x([1,4]),y([1,4]),z([1,4]),'LineWidth',3,'Color',colorQuad);
    plot3(x([1,5]),y([1,5]),z([1,5]),'LineWidth',3,'Color',colorQuad);
    plot3(x([1,6]),y([1,6]),z([1,6]),'LineWidth',3,'Color',colorQuad);

    % Plot the joints:
    plot3(x(1), y(1), z(1),'k.','MarkerSize',5,'LineWidth',5,'color',colorPoints);
    plot3(x(2), y(2), z(2),'k.','MarkerSize',20,'LineWidth',20,'color',colorBall);
    plot3(x(3), y(3), z(3),'k.','MarkerSize',5,'LineWidth',5,'color',colorPoints);
    plot3(x(4), y(4), z(4),'k.','MarkerSize',5,'LineWidth',5,'color',colorPoints);
    plot3(x(5), y(5), z(5),'k.','MarkerSize',5,'LineWidth',5,'color',colorPoints);
    plot3(x(6), y(6), z(6),'k.','MarkerSize',5,'LineWidth',5,'color',colorPoints);

    % Format the axis:
%     view(90,15);
%     axis([xBnd,yBnd,zBnd]); 
%     axis equal; axis manual;
    
%     % Save Frame
%     global counter
%     print('-f1000',num2str(counter),'-dpng')
%     counter = counter + 1;
end