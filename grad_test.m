%% Gradient plot

[X, Y] = meshgrid(linspace(-5,5,1e3), linspace(-5,5,1e3));

x_p = -5:.1:2;
y_p = -5:.1:2;

x_g = 2;
y_g = 2;

sigma_g = 4;
sigma_p = 0.2;
for i = 1:length(x_p);
    grad1 = 1*exp(-((X-x_g).^2./sigma_g^2+(Y-y_g).^2./sigma_g^2));
    grad2 = exp(-((X-x_p(i)).^2./sigma_p^2+(Y-y_p(i)).^2./sigma_p^2));

    figure(1), clf
    mesh(X,Y,grad1+grad2)
    pause(.01)
end