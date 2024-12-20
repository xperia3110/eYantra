'''
# Team ID:          eYRC##3882
# Theme:            WarehouseDrone
# Author List:      e-Yantra1, e-Yantra2
# Filename:         fibonacci.py
# Functions:        print_fibonacci_series, main
# Global variables: None
'''

import os, sys

def print_fibonacci_series(num_elements):
    '''
    Purpose:
    ---
    Prints the first num_elements of the Fibonnaci series.
    The next element of the series is given by:
        next_element = current_element + prev_element
    The code loops for num_elements and prints out the next element.
    
    Input Arguments:
    ---
    `num_elements` :  [ int ]
        number of elements in Fibonacci series to be printed
    
    Returns:
    ---
    None
    
    Example call:
    ---
    print_fibonacci_series(10)
    '''
    first = 0
    second = 1
    print('First ', num_elements, 'terms of Fibonacci series are:')
    print(first)
    print(second)
    
    for i in range(num_elements-2):
        # the next element is equal to the sum of the current element (second variable)
        # and the previous element (first variable)
        next_element = first + second
        # first element becomes the second element and second element becomes the
        # next element for the next loop iteration
        first = second
        second = next_element
        print(next_element)

def main():
    '''
    Purpose:
    ---
    Asks the user to input the number of elements required from the Fibonacci Series
    and call the function print_fibonacci_series.
    
    Input Arguments:
    ---
    None
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called automatically by the Operating System
    '''
    # Ask the user to input the number of elements required
    num_elements = input("Enter the number of terms: ")
    num_elements = int(num_elements)
    
    # Call the function print_fibonacci_series to print the first
    # num_elements of the Fibonacci Series
    print_fibonacci_series(num_elements)

# Function Name:    main (built in)
#        Inputs:    None
#       Outputs:    None
#       Purpose:    To call the main() function to print the Fibonacci series.
if __name__ == "__main__":
    main()
