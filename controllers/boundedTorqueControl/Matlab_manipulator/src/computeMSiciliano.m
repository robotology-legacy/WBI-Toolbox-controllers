function Mhat = computeMSiciliano(q, model)

m1 = model.m1;
m2 = model.m2;
a1 = model.a1;
l1 = model.l1;
l2 = model.l2;
Il1 = model.Il1;
Il2 = model.Il2;

%M matrix
M11 = Il1 ...
    + m1 * l1^2 ...
    + Il2 ... 
    + m2 * (a1^2 + l2^2 + 2 * a1 * l2 * cos(q(2)));
M12 = Il2 ...
    + m2 * (l2^2 + a1 * l2 * cos(q(2)));
M22 = Il2 ...
    + m2 * l2^2;

Mhat = [M11 M12; M12 M22];

end