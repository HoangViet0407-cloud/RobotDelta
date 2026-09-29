function r = rd_root()
%RD_ROOT  Thu muc goc du an RobotDelta, tinh tu vi tri cua chinh file nay.
%   Moi duong dan trong du an deu tinh tu day -> copy du an sang may / thu muc khac van chay.
r = fileparts(fileparts(mfilename('fullpath')));
end
