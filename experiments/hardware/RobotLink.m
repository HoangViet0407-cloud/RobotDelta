classdef RobotLink < handle
%ROBOTLINK  Giao tiep PC <-> vi dieu khien robot delta qua Serial (KHUNG MAU - phai khop firmware).
%
%   link = RobotLink("COM3", 115200);   % hoac RobotLink(obj) voi obj co writeline/readline (gia lap)
%   link.home();                        % bat driver + chay ve cong tac home
%   L = link.run(t, theta, io);         % gui quy dao goc [deg] (+ tin hieu bom hut), ghi log
%   delete(link);                       % tat driver, dong cong
%
%   GIAO THUC (ASCII, 1 lenh / dong, MCU tra loi DUNG 1 dong cho moi lenh):
%     PC -> MCU                               MCU -> PC
%     "E 1" | "E 0"   bat / tat driver        "ok"
%     "H"             ve home                 "ok"  (khi da xong)
%     "T <ms>"        chu ky noi suy          "ok"
%     "P <th1> <th2> <th3> <io>"              "S <ms> <th1> <th2> <th3> <trong>"  (them 1 diem vao bo dem)
%     "?"             hoi trang thai          "S <ms> <th1> <th2> <th3> <trong>"
%     loi                                     "err <noi dung>"
%   th  : goc canh tay [deg], cung quy uoc mo hinh (duoi phuong ngang = duong).
%         Firmware doi sang buoc: buoc = (th - theta_home) * buoc_moi_do * chieu_quay.
%   S   : thoi gian MCU [ms], goc hien tai (dem buoc hoac encoder), so cho trong trong bo dem.

    properties
        io              % serialport hoac doi tuong gia lap co writeline / readline
        bufSize = 0     % kich thuoc bo dem diem dat cua MCU
    end

    methods
        function obj = RobotLink(port, baud)
            if ischar(port) || isstring(port)
                obj.io = serialport(port, baud);
                configureTerminator(obj.io, "LF");
                obj.io.Timeout = 30;
                pause(2);                               % Arduino tu reset khi mo cong
                flush(obj.io);
            else
                obj.io = port;
            end
        end

        function r = cmd(obj, s)
            writeline(obj.io, s);
            r = strtrim(string(readline(obj.io)));
            if startsWith(r, "err"), error('RobotLink: "%s" -> %s', s, r); end
        end

        function home(obj)
            obj.cmd("E 1");
            obj.cmd("H");
        end

        function s = status(obj)
            s = RobotLink.parseS(obj.cmd("?"));
        end

        function L = run(obj, t, th, io)
            if nargin < 4, io = zeros(numel(t),1); end
            dt = t(2) - t(1);
            obj.cmd(sprintf("T %d", round(1000*dt)));
            st = obj.status(); obj.bufSize = st(5);
            L = zeros(numel(t) + 100, 5); nL = 0;
            for k = 1:numel(t)
                st = RobotLink.parseS(obj.cmd(sprintf("P %.3f %.3f %.3f %d", th(k,1), th(k,2), th(k,3), io(k))));
                nL = nL + 1; L(nL,:) = st;
                while st(5) == 0                        % bo dem day -> cho robot chay bot
                    pause(dt/2); st = obj.status(); nL = nL + 1; L(nL,:) = st;
                end
            end
            while st(5) < obj.bufSize                   % cho chay het bo dem
                pause(dt); st = obj.status(); nL = nL + 1; L(nL,:) = st;
            end
            L = array2table(L(1:nL,:), 'VariableNames', {'t_ms','theta1','theta2','theta3','trong'});
        end

        function delete(obj)
            if ~isempty(obj.io)
                try, obj.cmd("E 0"); catch, end
                obj.io = [];
            end
        end
    end

    methods (Static)
        function s = parseS(r)
            s = sscanf(extractAfter(r, "S "), '%f').';
            assert(numel(s) == 5, 'RobotLink: tra loi khong dung dinh dang: %s', r);
        end
    end
end
