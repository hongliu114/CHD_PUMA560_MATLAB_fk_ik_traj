function theta_random = generateRandomInitialGuess(qlims)
% 生成随机的初始猜测值
    theta_random = zeros(1, 6);
    for i = 1:6
        theta_random(i) = qlims(i,1) + (qlims(i,2) - qlims(i,1)) * rand();
    end
end