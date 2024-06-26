#################################################
# Criado por Victor Cruz de Oliveira
# Estudante de graduação da UnB 
# Data 24 de novembro de 2022
#
# This function receives a floating-point number
# and converts it to a binary floating-point representation
# using the IEEE-754-1985 standard.
# EW: number of bits of exponent word
# FW: number of bits of mantissa word
#################################################

from math import floor
from numpy import zeros

def dec2bin(number : int) -> str:      # Function to convert integer to binary
    nbin = bin(number if number>=0 else number+(1<<32))
    return nbin[2:]

def float2bin(EW : int, FW : int, A : float) -> str:

    h: list = []

    bias = ((2**EW)/2)-1
    n_exp = 0
    
    # Takes signal and represents the signal bit
    if A >= 0:
        s = '0'
    else:    
        s = '1'

    x = floor(abs(A))
    xbin = dec2bin(x)
    
    # Takes the fractional part and converts to binary
    B = abs(A)-x
    M=[]
    for i in range(8*FW):
        
        B = B*2
        
        if B >= 1:
            M.append('1')
            B = B-1
        else:
            M.append('0')
    M = ''.join(M)
            
    
    # Counts the number of bits for the exponent word
    if A == 0:
        n_exp = -bias
        Mn = zeros(FW+EW+1)
        Mn = list(map(int, Mn))
        Mn = map(str, Mn)
        Mn = list(Mn)
    elif xbin[0] == '1':
        n_exp = len(xbin)-1
    else:
        for i in range(5*FW): # To check
            if M[i]== '0':
                n_exp = n_exp - 1
            else:
                n_exp = n_exp-1
                break
        Mn=M[i+1:5*FW]

        
    # Adds bias to exponent and converts to binary
    n_exp = n_exp + bias
    exp = dec2bin(int(n_exp))
    nz=len(exp)
    for i in range(EW):
        if nz < EW:
            exp = '0' + exp 
            nz=len(exp)
            
    # Contatenates s|exp|xbin(2:length)M
    k=len(xbin);
    if abs(A) >= 1: 
        x2=xbin[1:k]
        h.extend(s)
        h.extend(exp)
        h.extend(x2)
        h.extend(M)
        h = ''.join(h)
    else:
        h.extend(s)
        h.extend(exp)
        h.extend(Mn)
        h = ''.join(h)

    return h[:FW+EW+1]

#print(float2bin(8, 18, 0.65742342109482301))
