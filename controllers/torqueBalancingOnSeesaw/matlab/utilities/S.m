function skewSymmetric = S(v)
	skewSymmetric = zeros(3, 3);
	%-z, y, -x
	skewSymmetric(1, 2) = -v(3);
	skewSymmetric(1, 3) =  v(2);
	skewSymmetric(2, 3) = -v(1);


	skewSymmetric(2, 1) = - skewSymmetric(1, 2);
	skewSymmetric(3, 1) = - skewSymmetric(1, 3);
	skewSymmetric(3, 2) = - skewSymmetric(2, 3);

end
