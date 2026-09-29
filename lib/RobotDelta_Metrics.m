function M = RobotDelta_Metrics(xyz, theta, G)
%ROBOTDELTA_METRICS  Chi so kiem tra cho tung tu the (moi hang = 1 diem, moi cot = 1 chan).
%   M = RobotDelta_Metrics(xyz, theta, G)     xyz: Nx3 [m], theta: Nx3 [deg]
%   M.tilt  [deg] goc nghieng bi mat trau = goc thanh truyen lech khoi mat phang vuong goc
%                 truc chot (gia su 2 dau mat trau cua thanh lap song song)
%   M.gamma [deg] goc giua canh tay va thanh truyen (gan 0 hoac 180 = ky di)
%   M.dL    [m]   sai lech chieu dai thanh truyen (~0 neu IK dung)

n = size(xyz,1);
M.tilt = zeros(n,3); M.gamma = zeros(n,3); M.dL = zeros(n,3);
for i = 1:3
    d = cross(G.e1(i,:), G.ez(i,:));                  % truc dong co = truc chot bi
    for k = 1:n
        th = theta(k,i)*pi/180;
        u  = cos(th)*G.e1(i,:) - sin(th)*G.ez(i,:);   % huong canh tay
        r  = xyz(k,:) + G.b(i,:) - G.F(i,:) - G.La(i)*u;   % thanh truyen: dau tay -> ban dong
        L  = norm(r);
        M.tilt(k,i)  = asind(abs(dot(d,r))/L);
        M.gamma(k,i) = acosd(dot(u,r)/L);
        M.dL(k,i)    = L - G.Lr(i);
    end
end
end
