#!/usr/bin/env python3

def f(x):
   if (x >= 12):
       return f(x-4) -3
   elif 6 < x< 12:
       return f(x-3) + 2)
   elif x <= 6:
       return x*x
   
x = f(20)
print f(x)
    
