function G = computeGSiciliano(q, model)

m1 = model.m1;
m2 = model.m2;
a1 = model.a1;
l1 = model.l1;
l2 = model.l2;
g = model.g;

%G matrix
G1 = (m1 * l1 + m2 * a1) * g * cos(q(1)) + m2 * l2 * g * cos(q(1) + q(2));
G2 = m2 * l2 * g * cos(q(1) + q(2));
G = [G1; G2];

end