%% RobotDelta_Geometry.m -- TU SINH boi build_RobotDelta_Sim.m (2026-09-28 22:53). KHONG sua tay.
% Hinh hoc dong hoc do tu CAD, don vi m / deg. Moi hang = 1 chan.
% He toa do robot: goc = tam de, o cao do truc dong co, z huong len (truc song song World).
G.origin     = [-0.008 0.06385640646 0.0385];  % goc he toa do robot, tinh trong he World cua CAD
G.La         = [0.1941648784 0.1941648784 0.1941648784];  % chieu dai canh tay tren (truc dong co -> tam 2 bi)
G.Lr         = [0.3742349999 0.3742349999 0.3742350001];  % chieu dai thanh truyen (tam bi -> tam bi)
G.wArm       = [0.068 0.068 0.068];  % khoang cach 2 bi tren canh tay
G.wPlat      = [0.068 0.068 0.068];  % khoang cach 2 bi tren ban dong (= wArm neu la hinh binh hanh)
G.F          = [-0.02694785222 0.0178562038 -3.226585665e-15;0.04321549903 0.02265262593 -2.102484853e-15;-0.00199 -0.08135640646 5.322131624e-15];  % diem tren truc dong co (mat phang canh tay)
G.e1         = [-0.8660254038 0.5 0;0.8660254038 0.5 0;1.277444095e-14 -1 0];  % phuong ngang huong ra ngoai cua canh tay
G.ez         = [-1.44973158e-14 -2.511008754e-14 1;-4.091765768e-14 7.087146203e-14 1;-5.941563962e-15 -7.590015798e-29 1];  % phuong "len" trong mat phang canh tay
G.b          = [-0.02598076212 0.01499999998 -9.12333542e-11;0.0259807621 0.01500000002 -8.14300849e-11;1.91983443e-11 -0.03 1.726633281e-10];  % vi tri tam cap bi tren ban dong so voi TCP
G.q0         = [-26.28587312 -39.23182087 -12.22130899];  % goc khop dong co tai tu the CAD [deg]
G.th0        = [38.17453116 51.12047891 24.10996702];  % goc canh tay duoi phuong ngang tai tu the CAD [deg]
G.s          = [-1 -1 -1];  % chieu: theta = th0 + s*(q - q0)
G.branch     = [1 1 1];  % nhanh nghiem IK
G.tcpLocal   = [-0.008980762114 0.04444486373 2.120334803e-17];  % TCP trong khung ban dong
G.tcpRot     = [-1 -6.399452496e-10 -1.886640844e-10;6.399451802e-10 -1 5.755445565e-09;-1.886640844e-10 5.755445495e-09 1];  % xoay TCP cho trung huong World
G.tcp_cad    = [-0.04881021217 -0.1105623539 -0.4313523914];  % TCP tai tu the CAD
G.home       = [0 0 -0.4313523914];  % vi tri nghi (tam vung lam viec)
