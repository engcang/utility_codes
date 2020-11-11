# -*- coding: utf-8 -*-
"""
Created on Wed May 20 12:34:12 2020

@author: Mason EungChang Lee
"""


import os

def convert_filename_from_folder(folder):
    for filename in os.listdir(folder):
        for idx in range(len(filename)):
            if filename[idx]=="E":
                #print(filename[idx:])
                new_name = filename[idx:]
                os.rename(folder+'/'+filename, folder+'/'+new_name)
                break

#convert_filename_from_folder("C:/Users/user/Desktop/회로과제/0Homework 4-390125") # 경로 /로 표시
convert_filename_from_folder(r'C:\Users\user\Desktop\회로과제\EE201(B)-Homework 4-390125') #경로 \로 표시 대신 r붙이기