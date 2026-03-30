#!/usr/bin/env python3

class PID:
    K = None
    Ti = None
    Td = None
    T = None
    e_past = 0
    u_past = 0

    def __init__(self,T=0.5,K=0,Ti=100000000, Td= 0):
        self.K = K
        self.Ti = Ti
        self.Td = Td
        self.T = T
    
    def pid(self,y, y_zad):
        e = y_zad - y
        u = self.K*e + self.u_past + self.K/self.Ti*self.T*(self.e_past + e)/2 + self.K*self.Td*(e-self.e_past)/self.T
        self.u_past = self.u_past + self.K/self.Ti*self.T*(self.e_past + e)/2
        self.e_past = e
        return u

    def p(self,y, y_zad):
        e = y_zad - y
        u = self.K*e
        return u

    def pi(self,y, y_zad):
        e = y_zad - y
        u = self.K*e + self.u_past + self.K/self.Ti*self.T*(self.e_past + e)/2
        self.u_past = self.u_past + self.K/self.Ti*self.T*(self.e_past + e)/2
        self.e_past = e
        return u
    
    def pd(self,y, y_zad):
        e = y_zad - y
        u = self.K*e + self.K*self.Td*(e-self.e_past)/self.T
        self.e_past = e
        return u

    
