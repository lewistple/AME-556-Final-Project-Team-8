function J = solveJacobian(p, theta_in, q1_in, q2_in, q3_in, q4_in)

syms x y theta q1 q2 q3 q4

J_sym = [(11*cos(q1 + q2 + theta))/50 + (11*cos(q1 + theta))/50, (11*cos(q1 + q2 + theta))/50,                                                      0,                            0;
         (11*sin(q1 + q2 + theta))/50 + (11*sin(q1 + theta))/50, (11*sin(q1 + q2 + theta))/50,                                                      0,                            0;
                                                              0,                            0,                                                      0,                            0;
                                                              0,                            0, (11*cos(q3 + q4 + theta))/50 + (11*cos(q3 + theta))/50, (11*cos(q3 + q4 + theta))/50;
                                                              0,                            0, (11*sin(q3 + q4 + theta))/50 + (11*sin(q3 + theta))/50, (11*sin(q3 + q4 + theta))/50;
                                                              0,                            0,                                                      0,                            0];

% [(11*sin(q1 + q2))/50 + (11*cos(q1))/50,  (11*sin(q1 + q2))/50,                                      0,                    0]
% [(11*sin(q1))/50 - (11*cos(q1 + q2))/50, -(11*cos(q1 + q2))/50,                                      0,                    0]
% [                                     0,                     0,                                      0,                    0]
% [                                     0,                     0, (11*cos(q3 + q4))/50 + (11*cos(q3))/50, (11*cos(q3 + q4))/50]
% [                                     0,                     0, (11*sin(q3 + q4))/50 + (11*sin(q3))/50, (11*sin(q3 + q4))/50]
% [                                     0,                     0,                                      0,                    0]


 
J = double(subs(J_sym, [x y theta q1 q2 q3 q4], [p(1) p(2) theta_in q1_in q2_in q3_in q4_in]));

end