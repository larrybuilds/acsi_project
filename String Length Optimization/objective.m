function [dObj, dObjGrad] = objective(x,u)
    % This function computes the torque-squared objective function and its
    % gradients.
    R1 = 10;
    R2 = 10;
    R3 = 10;
    R4 = 10;

    Q1 = 0.01;
    Q2 = 0.01;
    Q3 = 0.01;
    Q4 = 0.1;
    Q5 = 0.1;
    Q6 = 1;
    Q7 = 0.01;
    Q8 = 0.01;
    Q9 = 0.01;
    Q10 = 0.01;
    Q11 = 0.01;
    Q12 = 0.01;
    Q13 = 0.01;
    Q14 = 0.01;
    Q15 = 0.01;
    Q16 = 0.01;
    
    if nargout == 1 % numerical gradients
        dObj = autoGen_objective(x(1,:),x(2,:),x(3,:),x(4,:),...
                                 x(5,:),x(6,:),x(7,:),x(8,:),...
                                 x(9,:),x(10,:),x(11,:),x(12,:),...
                                 x(13,:),x(14,:),x(15,:),x(16,:),...
                                 u(1,:),u(2,:),u(3,:),u(4,:),R1,R2,R3,R4,...
                                 Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16);
    else  %Analytic gradients
        [dObj,fz,fzi] = autoGen_objective(u(1,:),u(2,:),u(3,:),u(4,:),R,Q);
        dObjGrad = zeros(20,length(dObj));  % 20 = 16 + 4 = state + control
        dObjGrad(fzi,:) = fz;
    end
end