%% Get values from the data file
%  C.LABAR on 04/09/14



file_name=fopen('data.log');
data=fscanf(file_name,'%f',[98 Inf]);

q.signals.values=zeros(25,length(data(1,:)));
qd.signals.values=zeros(25,length(data(1,:)));
qdd.signals.values=zeros(25,length(data(1,:)));

q.time=[0:0.1:length(data(1,:))/10-0.1];
qd.time=[0:0.1:length(data(1,:))/10-0.1];
qdd.time=[0:0.1:length(data(1,:))/10-0.1];

for i=1:length(data(1,:))
    q.signals.values(1:3,i)=data(54:56,i);   %Torso 3 joints
    q.signals.values(4:8,i)=data(6:10,i);    %Left arm 5 joints (without hand)
    q.signals.values(9:13,i)=data(13:17,i);  %Rigth arm 5 joints (without hand)
    q.signals.values(14:19,i)=data(57:62,i); %Left leg 6 joints
    q.signals.values(20:25,i)=data(63:68,i); %Rigth leg 6 joints
    
    qd.signals.values(1:3,i)=data(20:22,i);
    qd.signals.values(4:8,i)=data(23:27,i);
    qd.signals.values(9:13,i)=data(30:34,i);
    qd.signals.values(14:19,i)=data(72:77,i);
    qd.signals.values(20:25,i)=data(78:83,i);
    
    qdd.signals.values(1:3,i)=data(37:39,i);
    qdd.signals.values(4:8,i)=data(40:44,i);
    qdd.signals.values(9:13,i)=data(47:51,i);
    qdd.signals.values(14:19,i)=data(87:92,i);
    qdd.signals.values(20:25,i)=data(93:98,i);
end

q.signals.values=q.signals.values'/180*pi;
qd.signals.values=qd.signals.values'/180*pi;
qdd.signals.values=qdd.signals.values'/180*pi;
