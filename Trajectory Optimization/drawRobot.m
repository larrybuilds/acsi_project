function drawRobot(X)
    % This function draws the robot with configuration q and parameters p
    % INPUTS:
    %   x = [16, 1] = column vector of a single robot configuration
    %   p = parameter struct
    global counter

    % Compute the points that will be used for plotting
    P = getPoints(X); %forward kinematics
        
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
    colorPoints = [0,0,0]/255;
    
    alphaValues = linspace(0,1,110);
    alphaValue = alphaValues(counter);
    alphaValue = 1;
    alpha(alphaValue);
    
    colorString = [colorString alphaValue]; %gray
    colorQuad = [colorQuad alphaValue]; %gray

    % Set up the figure
%     hold off;

    % Plot the ground:
    [xGround, yGround] = meshgrid(-bnds:0.1:bnds); % Generate x and y data
    zGround = zeros(size(xGround, 1)); % Generate z data
    surf(xGround, yGround, zGround) % Plot the surface    
    colormap([cool(64);gray(64)]);

    hold on;

    % Plot the links:
    plot3(x([1,2]),y([1,2]),z([1,2]),'LineWidth',1,'Color',colorString);
    plot3(x([1,3]),y([1,3]),z([1,3]),'LineWidth',3,'Color',colorQuad);
    plot3(x([1,4]),y([1,4]),z([1,4]),'LineWidth',3,'Color',colorQuad);
    plot3(x([1,5]),y([1,5]),z([1,5]),'LineWidth',3,'Color',colorQuad);
    plot3(x([1,6]),y([1,6]),z([1,6]),'LineWidth',3,'Color',colorQuad);

    % Plot the joints:
    scatter3(x(1), y(1), z(1),20,'.','MarkerFaceColor',colorPoints,'MarkerEdgeColor',colorPoints,'MarkerFaceAlpha',alphaValue,'MarkerEdgeAlpha',alphaValue);
    scatter3(x(2), y(2), z(2),20,'o','MarkerFaceColor',colorBall,'MarkerEdgeColor',colorBall,'MarkerFaceAlpha',1,'MarkerEdgeAlpha',1);
    scatter3(x(3), y(3), z(3),20,'.','MarkerFaceColor',colorPoints,'MarkerEdgeColor',colorPoints,'MarkerFaceAlpha',alphaValue,'MarkerEdgeAlpha',alphaValue);
    scatter3(x(4), y(4), z(4),20,'.','MarkerFaceColor',colorPoints,'MarkerEdgeColor',colorPoints,'MarkerFaceAlpha',alphaValue,'MarkerEdgeAlpha',alphaValue);
    scatter3(x(5), y(5), z(5),20,'.','MarkerFaceColor',colorPoints,'MarkerEdgeColor',colorPoints,'MarkerFaceAlpha',alphaValue,'MarkerEdgeAlpha',alphaValue);
    scatter3(x(6), y(6), z(6),20,'.','MarkerFaceColor',colorPoints,'MarkerEdgeColor',colorPoints,'MarkerFaceAlpha',alphaValue,'MarkerEdgeAlpha',alphaValue);

    % Format the axis:
    view(-90,15);
    axis([xBnd,yBnd,zBnd]); 
    axis equal; axis manual;
    
    % Counter
    counter = counter + 1;
end