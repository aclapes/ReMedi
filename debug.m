close all;
clear all;

im = imread('0503.png');

figure(1);
imshow(im);

im(im < 17500) = im(im < 17500) + 1;

imwrite(im, '0503pi.png');

nonIdxMat = imread('nonIdxMat.png');
playerIdxMat = imread('playerIdxMat.png');

nonIdx = bitsrl(im,3);
B = sum(sum(nonIdx ~= nonIdxMat));

playerIdx = im - (nonIdx * 8);

figure(2);
imshow(playerIdx,[]);