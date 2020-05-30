function [u, visablePntsIdx] = projectToCam(K, Tr, pnts, imgSize)

Tr = Tr(1:3,:);
nPnts = size(pnts, 2);
% P = reshape(data.calib(:, 3), 4,3)';
% Tr = reshape(data.calib(:,5), 4,3)';
pntsTr = Tr * [pnts(1:3, :); ones(1, nPnts)];
% pntsTr = pntsTr(:, pntsTr(3,:) > 0.1);
u = K * pntsTr;
% visablePntsIdx = (u(3,:)>0);
u = u ./ repmat(u(3,:), 3, 1);
visablePntsIdx = (u(1,:)>=1) & ...
                 (u(1,:)<=imgSize(2)) & ... 
                 (u(2,:)>=1) & ...
                 (u(2,:)<=imgSize(1));
u = u(:, visablePntsIdx); 

end