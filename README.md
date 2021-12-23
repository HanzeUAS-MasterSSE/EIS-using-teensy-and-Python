# EIS-using-teensy-and-Python

This repository contains code based on the paper and code for the paper:

 'Electrochemical Impedance Spectroscopy System Based on a Teensy Board',
Leila Es Sebar, Leonardo Iannucci, Emma Angelini, Sabrina Grassini, and Marco Parvis,
IEEE TRANSACTIONS ON INSTRUMENTATION AND MEASUREMENT, VOL. 70, 2021

All the Python code is written from scratch.

The teensy ino code is rewritten extensively, but is based on code kindly provided by Marco Parvis.

The main entry point is the EIS_DataAcquisition.ipynb Jupyter notebook. It illustrates how to make a measurement, provided a circuit  inline with the illustrations in the wiring folder is present and connected through the COM port to the computer running the notebook.
