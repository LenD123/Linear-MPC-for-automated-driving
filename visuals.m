function [pred, t] = visuals(X, YPred, N)
    figure(1)
    hold on
    velstr = sprintf('Geschwindigkeit: %.01f',X(4));
    steerstr = sprintf('Lenkwinkel: %.1f°', X(5)*180/pi);
    t(1) = text(1.2, 1.8, velstr);
    t(2) = text(1.2, 1.7, steerstr);
    pred = plot(YPred(1:2:N*2), YPred(2:2:N*2), 'r-', 'linewidth',1.5);
    carBox(X, car.width_car, car.length_car)
    hold on
end

