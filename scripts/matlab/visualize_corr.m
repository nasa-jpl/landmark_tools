%  Copyright 2024 California Institute of Technology
%
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
%
%      http://www.apache.org/licenses/LICENSE-2.0
%
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.

function visualize_corr(I, output_prefix, title_str, limits, mask)

m = mean(I(~mask));
sigma = std(I(~mask));
if(limits==-1)
    limits = [m-4*sigma, m+4*sigma];
end

fig = figure; 
imagesc(I); 
colormap(brewermap([], 'BrBG'));
title(title_str);
caxis(limits);
c = colorbar;
cbarrow;
ylabel(c, 'Meters');
set(findall(gcf,'-property','FontSize'),'FontSize',14);
axis equal;
[h, w] = size(I);
xlim([0, w]);
ylim([0, h]);
xlabel("Pixels");
ylabel("Pixels");
savefig([output_prefix '.fig']);
saveas(fig, [output_prefix '.png']);

fig = figure;
subplot(1,4,1:3);
%edges = linspace(limits(1), limits(2), 256);
histogram(I(~mask)); %, edges);

xlabel(title_str);
ylabel('Pixel Count');
hold on;

m = mean(I(~mask));
sigma = std(I(~mask));
xline(m, 'r-', sprintf('mean = %2.2f', m));
xline(m-3*sigma, 'k-', sprintf('mean-3\\sigma = %2.2f', m-3*sigma));
xline(m+3*sigma, 'k-', sprintf('mean+3\\sigma = %2.2f', m+3*sigma));

subplot(1,4,4);
text(0,0.5, sprintf('min : %2.2f\nmax : %2.2f\nmean : %2.2f\nstd : %2.2f', min(I(~mask)), max(I(~mask)), m, sigma));
axis off;

set(findall(gcf,'-property','FontSize'),'FontSize',14);
savefig([output_prefix '-histogram.fig']);
saveas(fig, [output_prefix '.tif', '-histogram.png']);

end
