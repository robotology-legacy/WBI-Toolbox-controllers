function x = invSkew(S)
%invSkew Function generates a 3X1 vector from a 3X3 Skew Symmetric matrix 

    x = [-S(2,3)
          S(1,3)
         -S(1,2)];

end

