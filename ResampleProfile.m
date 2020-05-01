function x = ResampleProfile(x, Nfe)
xn = [];
Nlarge = 100;
for ii = 1 : (length(x) - 1)
    xn = [xn, linspace(x(ii), x(ii+1), Nlarge)];
end
xn = [xn, x(end)];
x = xn(round(linspace(1, length(xn), Nfe)));
end