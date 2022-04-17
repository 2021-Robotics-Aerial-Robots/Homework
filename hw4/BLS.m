function [t] = BLS(q, t, acc, error, J)
    % q is variable
    % t is step size
    
    alpha = 0.08;
    beta = 0.65;
    
    e = error( q(1), q(2), q(3), q(4), acc(1), acc(2), acc(3));
    J_q = J( q(1), q(2), q(3), q(4) );
    gradient_c = J_q' * e;
    
    % origin function
    e_ori = error( q(1)+t*(-gradient_c(1)), q(2)+t*(-gradient_c(2)), q(3)+t*(-gradient_c(3)), q(4)+t*(-gradient_c(4)), acc(1), acc(2), acc(3));
    ori_ans = e_ori' * e_ori;

    % adjust function
    adj_ans = e' * e - alpha * t * (gradient_c(1)^2 + gradient_c(2)^2 + gradient_c(3)^2 + gradient_c(4)^2);
    
    while ori_ans > adj_ans
         t = t * beta;
         
         e_ori = error( q(1)+t*(-gradient_c(1)), q(2)+t*(-gradient_c(2)), q(3)+t*(-gradient_c(3)), q(4)+t*(-gradient_c(4)), acc(1), acc(2), acc(3));
         ori_ans = e_ori' * e_ori;
         adj_ans = e' * e - alpha * t * (gradient_c(1)^2 + gradient_c(2)^2 + gradient_c(3)^2 + gradient_c(4)^2);
    end

end
