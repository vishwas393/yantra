function T = transformation_matrix(del, d, lam, l)
T = [[cos(del) -sin(del)*cos(lam) sin(del)*sin(lam) l*cos(del)];
        [ sin(del) cos(del)*cos(lam) -cos(del)*sin(lam) l*sin(del)];
        [ 0        sin(lam)           cos(lam)          d];
        [0         0                  0                 1]];
end