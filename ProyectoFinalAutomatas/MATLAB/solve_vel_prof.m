function x = solve_vel_prof(A,B,C)
x0 = [1,1,1];
x = lsqnonlin(@root3d,x0,zeros(size(x0))); 
    function F = root3d(x)
        F(1) = x(1)*x(2) + x(1)*x(3) - A;
        F(2) = 2*x(3) + x(2) -B;
        F(3) = x(1)/x(3) - C;
    end

end