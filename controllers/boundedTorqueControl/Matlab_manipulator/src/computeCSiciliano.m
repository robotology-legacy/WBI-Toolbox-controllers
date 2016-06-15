function C = computeCSiciliano(q, qd, model)

m2 = model.m2;
a1 = model.a1;
l2 = model.l2;

h = - m2 * a1 * l2 * sin(q(2));
C11 = h * qd(2);
C12 = h * (qd(1) + qd(2));
C21 = -h * qd(1);
C22 = 0;
C = [C11 C12; C21 C22];

end