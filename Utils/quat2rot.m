function R = quat2rot(q)


    % Compute elements of the rotation matrix
    r11 = q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;
    r21 = 2 * (q(2) * q(3) + q(1) * q(4));
    r31 = 2 * (q(2) * q(4) - q(1) * q(3));
    r12 = 2 * (q(2) * q(3) - q(1) * q(4));
    r22 = q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2;
    r32 = 2 * (q(3) * q(4) + q(1) * q(2));
    r13 = 2 * (q(2) * q(4) + q(1) * q(3));
    r23 = 2 * (q(3) * q(4) - q(1) * q(2));
    r33 = q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2;

    % ----------
    % r11 = 1 - 2*(q(3)^2 + q(4)^2);
  % Form the rotation matrix
    R = [r11, r12, r13; 
         r21, r22, r23; 
         r31, r32, r33];



end