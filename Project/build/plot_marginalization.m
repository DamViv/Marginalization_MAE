clc; clear; close all;

fileID = fopen('marginalization.txt','r');
formatSpec = '%f';
sizeA = [9 Inf];
%18
I_marg = fscanf(fileID, formatSpec, sizeA);
fclose(fileID);

fileID = fopen('sparcification.txt','r');
formatSpec = '%f';
sizeA = [9 Inf];
I_spars = fscanf(fileID, formatSpec, sizeA);
fclose(fileID);


figure;
clims = [-6, 6];

subplot 121;
imagesc(log(abs(I_marg)), clims);
title("Marginalized, Dense Information matrix")

subplot 122;
imagesc(log(abs(I_spars)), clims);
title("Sparsified Information matrix based on factor descent");
